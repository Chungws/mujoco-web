"""
Session API endpoints
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_shared.schemas import SessionInitRequest, SessionResponse

from vlaarena_backend.database import get_db
from vlaarena_backend.services.session_service import init_session

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/sessions/init", response_model=SessionResponse, status_code=status.HTTP_201_CREATED)
async def initialize_session(
    data: SessionInitRequest,
    db: AsyncSession = Depends(get_db),
):
    """
    Initialize new session with battle (VLA Arena MVP)

    Flow:
    1. User provides robot_id and scene_id
    2. System creates session record
    3. System assigns 2 random VLA models
    4. System creates battle record
    5. Returns session_id, battle_id, and hidden model names

    Args:
        data: Session initialization request with robot_id and scene_id
        db: Database session

    Returns:
        SessionResponse with session_id, battle_id, and hidden model names

    Raises:
        HTTPException 500: If database operations fail
    """
    try:
        result = await init_session(
            db=db,
            robot_id=data.robot_id,
            scene_id=data.scene_id,
        )
        return result

    except Exception as e:
        logger.error(f"Failed to initialize session: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to initialize session: {e!s}",
        ) from e
