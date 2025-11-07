"""
Session and battle business logic service (VLA Arena MVP)
"""

import logging
import random
import uuid
from datetime import UTC, datetime

from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_shared.models import Battle, Session

from ..repositories import BattleRepository, SessionRepository

logger = logging.getLogger(__name__)


# VLA models for MVP (2 models: OpenVLA 7B, Octo-base)
AVAILABLE_MODELS = ["openvla-7b", "octo-base"]


async def init_session(
    db: AsyncSession,
    robot_id: str,
    scene_id: str,
    user_id: str | None = None,
) -> dict:
    """
    Initialize new session with first battle (VLA Arena MVP)

    Flow:
    1. Create session record with robot_id and scene_id
    2. Select 2 random VLA models
    3. Randomly assign left/right positions (prevent position bias)
    4. Create battle record with model assignments
    5. Return session_id, battle_id, and hidden model names

    Args:
        db: Database session
        robot_id: Robot identifier (e.g., "widowx")
        scene_id: Scene identifier (e.g., "table")
        user_id: Optional user ID (for post-MVP authentication)

    Returns:
        Dict with session_id, battle_id, and hidden model names:
        {
            "session_id": "sess_abc123",
            "battle_id": "battle_def456",
            "left_model": "???",
            "right_model": "???"
        }

    Raises:
        Exception: If database operations fail
    """
    logger.info(f"Initializing session: robot={robot_id}, scene={scene_id}, user={user_id}")

    # Initialize repositories
    session_repo = SessionRepository(db)
    battle_repo = BattleRepository(db)

    try:
        # 1. Create session
        session_id = f"sess_{uuid.uuid4().hex[:12]}"
        session = Session(
            session_id=session_id,
            robot_id=robot_id,
            scene_id=scene_id,
            user_id=user_id,
            created_at=datetime.now(UTC),
            last_active_at=datetime.now(UTC),
        )
        session = await session_repo.create(session)

        logger.info(f"Session created: {session_id}")

        # 2. Select 2 random models from available models
        if len(AVAILABLE_MODELS) < 2:
            raise ValueError("Need at least 2 models for battles")

        # Randomly select 2 different models
        selected_models = random.sample(AVAILABLE_MODELS, 2)
        model_a, model_b = selected_models

        # 3. Randomly assign left/right positions (prevent position bias)
        if random.random() < 0.5:
            left_model_id, right_model_id = model_a, model_b
        else:
            left_model_id, right_model_id = model_b, model_a

        logger.info(f"Models selected: left={left_model_id}, right={right_model_id}")

        # 4. Create battle
        battle_id = f"battle_{uuid.uuid4().hex[:12]}"
        battle = Battle(
            battle_id=battle_id,
            session_id=session_id,
            left_model_id=left_model_id,
            right_model_id=right_model_id,
            seq_in_session=0,  # First battle in session
            status="ongoing",
            created_at=datetime.now(UTC),
            updated_at=datetime.now(UTC),
        )
        battle = await battle_repo.create(battle)

        logger.info(f"Battle created: {battle_id}")

        # Commit transaction
        await db.commit()

        # 5. Return response with hidden model names (blind A/B testing)
        return {
            "session_id": session_id,
            "battle_id": battle_id,
            "left_model": "???",  # Hidden until vote
            "right_model": "???",  # Hidden until vote
        }

    except Exception as e:
        logger.error(f"Failed to initialize session: {e}")
        await db.rollback()
        raise


# ====================================================================================
# STUB FUNCTIONS (lmarena-clone compatibility)
# These will be removed once we fully migrate to VLA Arena
# ====================================================================================


async def add_follow_up_message(battle_id: str, prompt: str, db: AsyncSession) -> dict:
    """Stub function for lmarena-clone compatibility - NOT IMPLEMENTED for VLA Arena"""
    raise NotImplementedError("Follow-up messages not implemented for VLA Arena MVP")


async def vote_on_battle(battle_id: str, vote: str, db: AsyncSession) -> dict:
    """Stub function for lmarena-clone compatibility - NOT IMPLEMENTED for VLA Arena"""
    raise NotImplementedError("Voting not implemented for VLA Arena MVP")


async def create_session_with_battle(
    prompt: str, db: AsyncSession, user_id: str | None = None
) -> dict:
    """Stub function for lmarena-clone compatibility - NOT IMPLEMENTED for VLA Arena"""
    raise NotImplementedError("LLM-based session creation not for VLA Arena MVP")


async def create_battle_in_session(session_id: str, prompt: str, db: AsyncSession) -> dict:
    """Stub function for lmarena-clone compatibility - NOT IMPLEMENTED for VLA Arena"""
    raise NotImplementedError("Multiple battles per session not in MVP")


async def get_sessions_by_user(
    user_id: str, db: AsyncSession, limit: int = 50, offset: int = 0
) -> dict:
    """Stub function for lmarena-clone compatibility - NOT IMPLEMENTED for VLA Arena"""
    raise NotImplementedError("Session history not in MVP")


async def get_battles_by_session(session_id: str, db: AsyncSession) -> dict:
    """Stub function for lmarena-clone compatibility - NOT IMPLEMENTED for VLA Arena"""
    raise NotImplementedError("Battle history not in MVP")
