"""
Session and battle business logic service
"""

import asyncio
import logging
import random
import uuid
from datetime import UTC, datetime
from typing import Dict, List, Optional

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_shared.config import MULTI_ASSISTANT_SYSTEM_PROMPT
from vlaarena_shared.models import Battle, Message, Session, Turn

from ..repositories import BattleRepository, SessionRepository, VoteRepository
from .llm_client import get_llm_client
from .model_service import get_model_service


logger = logging.getLogger(__name__)


async def get_session_messages(
    db: AsyncSession,
    session_id: str,
) -> List[Dict[str, str]]:
    """
    Assemble session-wide conversation with multi-assistant responses.

    This function creates a conversation history for LLM API calls by:
    1. Adding a system prompt explaining the multi-assistant setup
    2. Iterating through all battles in the session (ordered by seq_in_session)
    3. For each turn in each battle, adding the user input and assistant responses

    The result is a conversation where multiple assistant messages follow each
    user message, representing responses from different models across battles.

    Args:
        db: Database session
        session_id: Session ID

    Returns:
        List of messages in OpenAI chat format:
        [
            {"role": "system", "content": "..."},
            {"role": "user", "content": "..."},
            {"role": "assistant", "content": "..."},  # Battle 1 left
            {"role": "assistant", "content": "..."},  # Battle 1 right
            {"role": "user", "content": "..."},
            {"role": "assistant", "content": "..."},  # Battle 2 left
            {"role": "assistant", "content": "..."},  # Battle 2 right
            ...
        ]
    """
    # Start with system prompt
    messages = [{"role": "system", "content": MULTI_ASSISTANT_SYSTEM_PROMPT}]

    # Fetch all messages for the session, sorted by ordering fields
    result = await db.execute(
        select(Message)
        .filter(Message.session_id == session_id)
        .order_by(
            Message.battle_seq_in_session,
            Message.turn_seq,
            Message.seq_in_turn,
        )
    )
    all_messages = result.scalars().all()

    # Fetch all turns for the session to get user inputs
    result = await db.execute(select(Turn).filter(Turn.session_id == session_id).order_by(Turn.seq))
    turns = result.scalars().all()

    # Create turn_id -> user_input mapping
    turn_map = {turn.turn_id: turn.user_input for turn in turns}

    # Assemble conversation
    current_turn_id = None
    for message in all_messages:
        # Add user message when turn changes
        if message.turn_id != current_turn_id:
            current_turn_id = message.turn_id
            user_input = turn_map.get(current_turn_id)
            if user_input:
                messages.append({"role": "user", "content": user_input})

        # Add assistant message
        messages.append({"role": "assistant", "content": message.content})

    return messages


async def create_session_with_battle(
    prompt: str,
    db: AsyncSession,
    user_id: Optional[str] = None,
) -> Dict:
    """
    Create new session with first battle

    Flow:
    1. Create session record
    2. Select 2 random models
    3. Call LLMs in parallel
    4. Create battle with conversation
    5. Return anonymous responses

    Args:
        prompt: User's initial prompt
        db: Database session
        user_id: Optional user ID (UUID string for anonymous users)

    Returns:
        Dict with session_id, battle_id, message_id, responses

    Raises:
        Exception: If LLM API fails or model selection fails
    """
    logger.info(f"Creating session with prompt: {prompt[:50]}... (user_id={user_id})")

    # Initialize repositories
    session_repo = SessionRepository(db)
    battle_repo = BattleRepository(db)

    # 1. Create session
    session_id = f"session_{uuid.uuid4().hex[:12]}"
    session = Session(
        session_id=session_id,
        title=prompt[:200],  # Use first 200 chars as title
        user_id=user_id,  # Store user_id (None for anonymous without ID)
        created_at=datetime.now(UTC),
        last_active_at=datetime.now(UTC),
    )
    session = await session_repo.create(session)

    logger.info(f"Session created: {session_id}")

    # 2. Select 2 random models
    model_service = get_model_service()
    model_a, model_b = model_service.select_models_for_battle()

    # 3. Randomly assign left/right positions (prevent position bias)
    if random.random() < 0.5:
        left_model, right_model = model_a, model_b
    else:
        left_model, right_model = model_b, model_a

    logger.info(f"Models selected: left={left_model.id}, right={right_model.id}")

    # 4. Call LLMs in parallel
    llm_client = get_llm_client()

    messages = [{"role": "user", "content": prompt}]

    try:
        # Parallel API calls
        left_task = llm_client.chat_completion(left_model, messages)
        right_task = llm_client.chat_completion(right_model, messages)

        left_response, right_response = await asyncio.gather(left_task, right_task)

        logger.info(
            f"LLM responses received: "
            f"left={left_response.latency_ms}ms, "
            f"right={right_response.latency_ms}ms"
        )

    except Exception as e:
        logger.error(f"LLM API call failed: {e}")
        # Rollback session creation
        await db.rollback()
        raise Exception(f"Failed to get LLM responses: {str(e)}")

    # 5. Create battle
    battle_id = f"battle_{uuid.uuid4().hex[:12]}"

    battle = Battle(
        battle_id=battle_id,
        session_id=session_id,
        left_model_id=left_model.id,
        right_model_id=right_model.id,
        seq_in_session=0,  # First battle in session
        status="ongoing",
        created_at=datetime.now(UTC),
        updated_at=datetime.now(UTC),
    )
    battle = await battle_repo.create(battle)

    # 6. Create Turn record
    turn_id = f"turn_{uuid.uuid4().hex[:12]}"
    turn = Turn(
        turn_id=turn_id,
        session_id=session_id,
        battle_id=battle_id,
        battle_seq_in_session=0,  # First battle
        seq=0,  # First turn in battle
        user_input=prompt,
        created_at=datetime.now(UTC),
    )
    db.add(turn)

    # 7. Create Message records
    left_message = Message(
        message_id=f"msg_{uuid.uuid4().hex[:12]}",
        turn_id=turn_id,
        session_id=session_id,
        battle_id=battle_id,
        battle_seq_in_session=0,
        turn_seq=0,
        seq_in_turn=0,  # Left = 0
        session_seq=0,  # First message in session
        side="left",
        content=left_response.content,
        created_at=datetime.now(UTC),
    )
    db.add(left_message)

    right_message = Message(
        message_id=f"msg_{uuid.uuid4().hex[:12]}",
        turn_id=turn_id,
        session_id=session_id,
        battle_id=battle_id,
        battle_seq_in_session=0,
        turn_seq=0,
        seq_in_turn=1,  # Right = 1
        session_seq=1,  # Second message in session
        side="right",
        content=right_response.content,
        created_at=datetime.now(UTC),
    )
    db.add(right_message)

    # Commit session, battle, turn, and messages
    await db.commit()

    logger.info(f"Battle created: {battle_id}, Turn: {turn_id}, Messages: left+right")

    # 8. Return anonymous responses
    return {
        "session_id": session_id,
        "battle_id": battle_id,
        "message_id": "msg_1",  # First message
        "responses": [
            {
                "position": "left",
                "text": left_response.content,
                "latency_ms": left_response.latency_ms,
            },
            {
                "position": "right",
                "text": right_response.content,
                "latency_ms": right_response.latency_ms,
            },
        ],
    }


async def create_battle_in_session(
    session_id: str,
    prompt: str,
    db: AsyncSession,
) -> Dict:
    """
    Create new battle in existing session

    Flow:
    1. Verify session exists
    2. Update session.last_active_at
    3. Select 2 NEW random models
    4. Call LLMs in parallel
    5. Create battle with conversation
    6. Return anonymous responses

    Args:
        session_id: Existing session ID
        prompt: User's prompt for new battle
        db: Database session

    Returns:
        Dict with session_id, battle_id, message_id, responses

    Raises:
        ValueError: If session not found
        Exception: If LLM API fails or model selection fails
    """
    logger.info(f"Creating new battle in session {session_id} with prompt: {prompt[:50]}...")

    # Initialize repositories
    session_repo = SessionRepository(db)
    battle_repo = BattleRepository(db)

    # 1. Verify session exists
    session = await session_repo.get_by_session_id(session_id)
    if not session:
        raise ValueError(f"Session not found: {session_id}")

    logger.info(f"Session found: {session_id}")

    # 2. Update session.last_active_at
    session.last_active_at = datetime.now(UTC)
    session = await session_repo.update(session)

    # 3. Get session-wide history and determine seq_in_session
    session_history = await get_session_messages(db, session_id)

    # Count existing battles to determine seq_in_session
    existing_battles = await battle_repo.get_by_session_id(session_id)
    battle_seq = len(existing_battles)

    logger.info(f"Session has {battle_seq} existing battles, new battle will be #{battle_seq}")

    # 4. Select 2 NEW random models
    model_service = get_model_service()
    model_a, model_b = model_service.select_models_for_battle()

    # Randomly assign left/right positions (prevent position bias)
    if random.random() < 0.5:
        left_model, right_model = model_a, model_b
    else:
        left_model, right_model = model_b, model_a

    logger.info(f"Models selected: left={left_model.id}, right={right_model.id}")

    # 5. Call LLMs with session-wide history
    llm_client = get_llm_client()

    # Add new prompt to session history
    messages = session_history + [{"role": "user", "content": prompt}]

    logger.info(f"Calling LLMs with session-wide history ({len(messages)} messages)")

    try:
        # Parallel API calls
        left_task = llm_client.chat_completion(left_model, messages)
        right_task = llm_client.chat_completion(right_model, messages)

        left_response, right_response = await asyncio.gather(left_task, right_task)

        logger.info(
            f"LLM responses received: "
            f"left={left_response.latency_ms}ms, "
            f"right={right_response.latency_ms}ms"
        )

    except Exception as e:
        logger.error(f"LLM API call failed: {e}")
        # Rollback session update
        await db.rollback()
        raise Exception(f"Failed to get LLM responses: {str(e)}")

    # 6. Create battle
    battle_id = f"battle_{uuid.uuid4().hex[:12]}"

    battle = Battle(
        battle_id=battle_id,
        session_id=session_id,
        left_model_id=left_model.id,
        right_model_id=right_model.id,
        seq_in_session=battle_seq,
        status="ongoing",
        created_at=datetime.now(UTC),
        updated_at=datetime.now(UTC),
    )
    battle = await battle_repo.create(battle)

    # 7. Calculate session_seq for new messages
    # Count existing messages in the session
    existing_message_count_result = await db.execute(
        select(Message).filter(Message.session_id == session_id)
    )
    existing_messages = existing_message_count_result.scalars().all()
    session_seq_start = len(existing_messages)

    # 8. Create Turn record
    turn_id = f"turn_{uuid.uuid4().hex[:12]}"
    turn = Turn(
        turn_id=turn_id,
        session_id=session_id,
        battle_id=battle_id,
        battle_seq_in_session=battle_seq,
        seq=0,  # First turn in this battle
        user_input=prompt,
        created_at=datetime.now(UTC),
    )
    db.add(turn)

    # 9. Create Message records
    left_message = Message(
        message_id=f"msg_{uuid.uuid4().hex[:12]}",
        turn_id=turn_id,
        session_id=session_id,
        battle_id=battle_id,
        battle_seq_in_session=battle_seq,
        turn_seq=0,
        seq_in_turn=0,  # Left = 0
        session_seq=session_seq_start,
        side="left",
        content=left_response.content,
        created_at=datetime.now(UTC),
    )
    db.add(left_message)

    right_message = Message(
        message_id=f"msg_{uuid.uuid4().hex[:12]}",
        turn_id=turn_id,
        session_id=session_id,
        battle_id=battle_id,
        battle_seq_in_session=battle_seq,
        turn_seq=0,
        seq_in_turn=1,  # Right = 1
        session_seq=session_seq_start + 1,
        side="right",
        content=right_response.content,
        created_at=datetime.now(UTC),
    )
    db.add(right_message)

    # Commit session update, battle, turn, and messages
    await db.commit()

    logger.info(f"Battle created: {battle_id}, Turn: {turn_id}, Messages: left+right")

    # 10. Return anonymous responses
    return {
        "session_id": session_id,
        "battle_id": battle_id,
        "message_id": "msg_1",  # First message of new battle
        "responses": [
            {
                "position": "left",
                "text": left_response.content,
                "latency_ms": left_response.latency_ms,
            },
            {
                "position": "right",
                "text": right_response.content,
                "latency_ms": right_response.latency_ms,
            },
        ],
    }


async def add_follow_up_message(
    battle_id: str,
    prompt: str,
    db: AsyncSession,
) -> Dict:
    """
    Add follow-up message to existing battle conversation

    Flow:
    1. Verify battle exists and is ongoing
    2. Retrieve conversation history from JSONB
    3. Build message history for each model (OpenAI format)
    4. Call LLMs with full history + new prompt
    5. Append new messages to conversation JSONB
    6. Update battle
    7. Return anonymous responses with message_count

    Args:
        battle_id: Existing battle ID
        prompt: User's follow-up prompt
        db: Database session

    Returns:
        Dict with battle_id, message_id, responses, message_count, max_messages

    Raises:
        ValueError: If battle not found or already voted
        Exception: If LLM API fails
    """
    logger.info(f"Adding follow-up message to battle {battle_id} with prompt: {prompt[:50]}...")

    # Initialize repository
    battle_repo = BattleRepository(db)

    # 1. Verify battle exists
    battle = await battle_repo.get_by_battle_id(battle_id)
    if not battle:
        raise ValueError(f"Battle not found: {battle_id}")

    # 2. Check battle status (must be ongoing)
    if battle.status != "ongoing":
        raise ValueError(f"Cannot add message to battle with status: {battle.status}")

    # Count turns in this battle
    turn_count_result = await db.execute(select(Turn).filter(Turn.battle_id == battle_id))
    turn_count = len(turn_count_result.scalars().all())

    logger.info(f"Battle found: {battle_id}, current turns: {turn_count}")

    # 3. Get model configs
    model_service = get_model_service()
    left_model = model_service.get_model(battle.left_model_id)
    right_model = model_service.get_model(battle.right_model_id)

    # 4. Build session-wide conversation history (multi-assistant approach)
    session_history = await get_session_messages(db, battle.session_id)

    # Add new user message to history
    messages = session_history + [{"role": "user", "content": prompt}]

    logger.info(f"Built session-wide conversation history: {len(messages)} messages total")

    # 5. Call LLMs with session-wide history (both models get same history)
    llm_client = get_llm_client()

    try:
        # Parallel API calls with session-wide history
        left_task = llm_client.chat_completion(left_model, messages)
        right_task = llm_client.chat_completion(right_model, messages)

        left_response, right_response = await asyncio.gather(left_task, right_task)

        logger.info(
            f"LLM responses received: "
            f"left={left_response.latency_ms}ms, "
            f"right={right_response.latency_ms}ms"
        )

    except Exception as e:
        logger.error(f"LLM API call failed: {e}")
        await db.rollback()
        raise Exception(f"Failed to get LLM responses: {str(e)}")

    # 6. Calculate session_seq for new messages
    existing_message_count_result = await db.execute(
        select(Message).filter(Message.session_id == battle.session_id)
    )
    existing_messages = existing_message_count_result.scalars().all()
    session_seq_start = len(existing_messages)

    # 7. Create Turn record
    turn_id = f"turn_{uuid.uuid4().hex[:12]}"
    turn = Turn(
        turn_id=turn_id,
        session_id=battle.session_id,
        battle_id=battle_id,
        battle_seq_in_session=battle.seq_in_session,
        seq=turn_count,  # Next turn in this battle
        user_input=prompt,
        created_at=datetime.now(UTC),
    )
    db.add(turn)

    # 8. Create Message records
    left_message = Message(
        message_id=f"msg_{uuid.uuid4().hex[:12]}",
        turn_id=turn_id,
        session_id=battle.session_id,
        battle_id=battle_id,
        battle_seq_in_session=battle.seq_in_session,
        turn_seq=turn_count,
        seq_in_turn=0,  # Left = 0
        session_seq=session_seq_start,
        side="left",
        content=left_response.content,
        created_at=datetime.now(UTC),
    )
    db.add(left_message)

    right_message = Message(
        message_id=f"msg_{uuid.uuid4().hex[:12]}",
        turn_id=turn_id,
        session_id=battle.session_id,
        battle_id=battle_id,
        battle_seq_in_session=battle.seq_in_session,
        turn_seq=turn_count,
        seq_in_turn=1,  # Right = 1
        session_seq=session_seq_start + 1,
        side="right",
        content=right_response.content,
        created_at=datetime.now(UTC),
    )
    db.add(right_message)

    # 9. Update battle updated_at
    battle.updated_at = datetime.now(UTC)
    battle = await battle_repo.update(battle)

    # Commit turn and messages
    await db.commit()

    # Calculate message count (number of turns + 1 for new turn)
    user_message_count = turn_count + 1

    logger.info(
        f"Follow-up message added to battle: {battle_id}, total user messages: {user_message_count}"
    )

    # 8. Return anonymous responses
    return {
        "battle_id": battle_id,
        "message_id": f"msg_{user_message_count}",  # Second user message is msg_2, etc.
        "responses": [
            {
                "position": "left",
                "text": left_response.content,
                "latency_ms": left_response.latency_ms,
            },
            {
                "position": "right",
                "text": right_response.content,
                "latency_ms": right_response.latency_ms,
            },
        ],
        "message_count": user_message_count,
        "max_messages": 6,  # Backend enforced limit
    }


async def vote_on_battle(
    battle_id: str,
    vote: str,
    db: AsyncSession,
) -> Dict:
    """
    Submit vote on battle and reveal model identities

    Transaction:
    1. Get battle (check exists and ongoing)
    2. Create vote record with denormalized model IDs
    3. Update battle status to 'voted'
    4. Update session last_active_at timestamp
    5. Return vote confirmation with revealed models

    Args:
        battle_id: Existing battle ID
        vote: User's vote (left_better, right_better, tie, both_bad)
        db: Database session

    Returns:
        Dict with battle_id, vote, revealed_models

    Raises:
        ValueError: If battle not found or already voted
    """
    logger.info(f"Processing vote for battle {battle_id}: {vote}")

    # Initialize repositories
    battle_repo = BattleRepository(db)
    session_repo = SessionRepository(db)
    vote_repo = VoteRepository(db)

    # 1. Get battle and validate
    battle = await battle_repo.get_by_battle_id(battle_id)
    if not battle:
        raise ValueError(f"Battle not found: {battle_id}")

    if battle.status != "ongoing":
        raise ValueError(f"Battle has already been voted: {battle_id}")

    # 2. Create vote record with denormalized model IDs
    vote_id = f"vote_{uuid.uuid4().hex[:12]}"
    from vlaarena_shared.models import Vote

    vote_record = Vote(
        vote_id=vote_id,
        battle_id=battle_id,
        session_id=battle.session_id,
        vote=vote,
        left_model_id=battle.left_model_id,
        right_model_id=battle.right_model_id,
        processing_status="pending",
        voted_at=datetime.now(UTC),
    )

    await vote_repo.create(vote_record)
    logger.info(f"Vote record created: {vote_id}")

    # 3. Update battle status to 'voted'
    battle.status = "voted"
    await battle_repo.update(battle)
    logger.info(f"Battle status updated to 'voted': {battle_id}")

    # 4. Update session last_active_at
    session = await session_repo.get_by_session_id(battle.session_id)
    if session:
        session.last_active_at = datetime.now(UTC)
        await session_repo.update(session)
        logger.info(f"Session last_active_at updated: {battle.session_id}")
    else:
        logger.warning(f"Session not found for updating last_active_at: {battle.session_id}")

    # 5. Return vote confirmation with revealed models
    return {
        "battle_id": battle_id,
        "vote": vote,
        "revealed_models": {
            "left": battle.left_model_id,
            "right": battle.right_model_id,
        },
    }


async def get_sessions_by_user(
    user_id: str,
    db: AsyncSession,
    limit: int = 50,
    offset: int = 0,
) -> Dict:
    """
    Get session list for a user with pagination

    Args:
        user_id: User ID (UUID string for anonymous users)
        db: Database session
        limit: Maximum number of sessions to return (default 50)
        offset: Number of sessions to skip (default 0)

    Returns:
        Dict with sessions list and total count
    """
    logger.info(f"Getting sessions for user {user_id} (limit={limit}, offset={offset})")

    # Initialize repository
    session_repo = SessionRepository(db)

    # Get sessions with pagination
    sessions = await session_repo.get_by_user_id(user_id, limit=limit, offset=offset)

    # Get total count
    total = await session_repo.count_by_user_id(user_id)

    logger.info(f"Found {len(sessions)} sessions for user {user_id} (total: {total})")

    # Convert to response format
    session_items = [
        {
            "session_id": session.session_id,
            "title": session.title,
            "created_at": session.created_at,
            "last_active_at": session.last_active_at,
        }
        for session in sessions
    ]

    return {
        "sessions": session_items,
        "total": total,
    }


async def get_battles_by_session(
    session_id: str,
    db: AsyncSession,
) -> Dict:
    """
    Get all battles for a session with vote information

    Args:
        session_id: Session ID
        db: Database session

    Returns:
        Dict with session_id and battles list

    Raises:
        ValueError: If session not found
    """
    logger.info(f"Getting battles for session {session_id}")

    # Initialize repositories
    session_repo = SessionRepository(db)
    BattleRepository(db)
    vote_repo = VoteRepository(db)

    # Verify session exists
    session = await session_repo.get_by_session_id(session_id)
    if not session:
        raise ValueError(f"Session not found: {session_id}")

    # Fetch all battles, turns and messages for the session (efficient: 3 queries)
    battles_result = await db.execute(
        select(Battle).filter(Battle.session_id == session_id).order_by(Battle.seq_in_session)
    )
    battles = battles_result.scalars().all()

    logger.info(f"Found {len(battles)} battles for session {session_id}")

    turns_result = await db.execute(
        select(Turn).filter(Turn.session_id == session_id).order_by(Turn.seq)
    )
    turns = turns_result.scalars().all()

    messages_result = await db.execute(
        select(Message)
        .filter(Message.session_id == session_id)
        .order_by(
            Message.battle_seq_in_session,
            Message.turn_seq,
            Message.seq_in_turn,
        )
    )
    messages = messages_result.scalars().all()

    # Group conversations by battle_id
    battle_conversations: Dict[str, List[Dict]] = {}

    for turn in turns:
        if turn.battle_id not in battle_conversations:
            battle_conversations[turn.battle_id] = []

        # Add user message
        battle_conversations[turn.battle_id].append(
            {
                "role": "user",
                "content": turn.user_input,
                "timestamp": turn.created_at.isoformat(),
            }
        )

        # Add assistant messages for this turn
        turn_messages = [m for m in messages if m.turn_id == turn.turn_id]
        for msg in sorted(turn_messages, key=lambda m: m.seq_in_turn):
            battle_conversations[turn.battle_id].append(
                {
                    "role": "assistant",
                    "content": msg.content,
                    "position": msg.side,
                    "timestamp": msg.created_at.isoformat(),
                }
            )

    # Convert to response format with vote information
    battle_items = []
    for battle in battles:
        battle_item = {
            "battle_id": battle.battle_id,
            "left_model_id": battle.left_model_id,
            "right_model_id": battle.right_model_id,
            "conversation": battle_conversations.get(battle.battle_id, []),
            "status": battle.status,
            "vote": None,
            "created_at": battle.created_at,
        }

        # If battle is voted, get vote information
        if battle.status == "voted":
            vote = await vote_repo.get_by_battle_id(battle.battle_id)
            if vote:
                battle_item["vote"] = vote.vote

        battle_items.append(battle_item)

    return {
        "session_id": session_id,
        "battles": battle_items,
    }
