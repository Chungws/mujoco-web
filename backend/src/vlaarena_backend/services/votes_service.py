"""
Votes Service - Business logic for vote submission
VLA Arena MVP - Following FastAPI 4-layer pattern
"""

import secrets
from datetime import UTC, datetime

from sqlalchemy import select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_shared.models import Battle, Session, Vote
from vlaarena_shared.schemas import VoteRequest


async def create_vote(db: AsyncSession, vote_data: VoteRequest) -> dict:
    """
    Create a vote for a battle and reveal model identities

    Args:
        db: Database session
        vote_data: Vote request data (battle_id, vote)

    Returns:
        dict with vote_id and revealed_models

    Raises:
        ValueError: If battle not found, session not found, or duplicate vote
    """
    # 1. Fetch battle
    stmt = select(Battle).where(Battle.battle_id == vote_data.battle_id)
    result = await db.execute(stmt)
    battle = result.scalar_one_or_none()

    if not battle:
        raise ValueError(f"Battle not found: {vote_data.battle_id}")

    # 2. Fetch session for denormalization
    stmt = select(Session).where(Session.session_id == battle.session_id)
    result = await db.execute(stmt)
    session = result.scalar_one_or_none()

    if not session:
        raise ValueError(f"Session not found: {battle.session_id}")

    # 3. Create vote record with denormalized data
    vote_id = f"vote_{secrets.token_urlsafe(16)}"

    vote = Vote(
        vote_id=vote_id,
        battle_id=battle.battle_id,
        session_id=session.session_id,
        robot_id=session.robot_id,  # Denormalized
        scene_id=session.scene_id,  # Denormalized
        left_model_id=battle.left_model_id,  # Denormalized
        right_model_id=battle.right_model_id,  # Denormalized
        vote=vote_data.vote,
        processing_status="pending",
        processed_at=None,
        voted_at=datetime.now(UTC),
    )

    db.add(vote)

    try:
        await db.commit()
        await db.refresh(vote)
    except IntegrityError as e:
        await db.rollback()
        # Check if it's a duplicate battle_id (unique constraint)
        if "battle_id" in str(e).lower() or "unique" in str(e).lower():
            raise ValueError(f"Battle {vote_data.battle_id} already has a vote") from e
        raise

    # 4. Return vote_id and revealed models
    return {
        "vote_id": vote.vote_id,
        "revealed_models": {
            "left": battle.left_model_id,
            "right": battle.right_model_id,
        },
    }
