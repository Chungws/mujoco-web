"""
Battle API endpoints
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_shared.schemas import TurnRequest, TurnResponse

from vlaarena_backend.database import get_db
from vlaarena_backend.services.turn_service import TurnService

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post(
    "/battles/{battle_id}/turns",
    response_model=TurnResponse,
    status_code=status.HTTP_201_CREATED,
)
async def create_turn(
    battle_id: str,
    data: TurnRequest,
    db: AsyncSession = Depends(get_db),
):
    """
    Create a new turn in a battle with VLA execution

    Flow:
    1. User provides instruction for existing battle
    2. System creates Turn record
    3. System executes both VLA models (left and right) in parallel
    4. System stores episodes in MongoDB
    5. Returns turn_id and episode_ids

    Args:
        battle_id: Existing battle ID
        data: Turn request with instruction
        db: Database session

    Returns:
        TurnResponse with turn_id, left_episode_id, right_episode_id, status

    Raises:
        HTTPException 404: If battle not found
        HTTPException 500: If VLA execution or internal error occurs
    """
    try:
        turn_service = TurnService()
        result = await turn_service.create_turn(battle_id, data, db)
        return result

    except ValueError as e:
        error_msg = str(e).lower()
        if "not found" in error_msg:
            logger.error(f"Battle not found: {battle_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=str(e),
            ) from e
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=str(e),
            ) from e

    except Exception as e:
        logger.error(f"Failed to create turn for battle {battle_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create turn: {e!s}",
        ) from e
