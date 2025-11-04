"""
Battle API endpoints
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_backend.database import get_db
from vlaarena_backend.services.session_service import (
    add_follow_up_message,
    vote_on_battle,
)
from vlaarena_shared.schemas import (
    FollowUpCreate,
    FollowUpResponse,
    VoteCreate,
    VoteResponse,
)


logger = logging.getLogger(__name__)

router = APIRouter()


@router.post(
    "/battles/{battle_id}/messages",
    response_model=FollowUpResponse,
    status_code=status.HTTP_201_CREATED,
)
async def add_message_to_battle(
    battle_id: str,
    data: FollowUpCreate,
    db: AsyncSession = Depends(get_db),
):
    """
    Add follow-up message to existing battle

    Flow:
    1. User submits follow-up prompt
    2. System retrieves conversation history from battle (JSONB)
    3. System calls LLM APIs with full message history (OpenAI chat format)
    4. System appends new messages to battle.conversation using || operator
    5. Returns anonymous responses with message_count

    Args:
        battle_id: Existing battle ID
        data: Follow-up message request with prompt
        db: Database session

    Returns:
        FollowUpResponse with battle_id, message_id, responses, message_count

    Raises:
        HTTPException 404: If battle not found
        HTTPException 400: If battle status is not 'ongoing' (e.g., already voted)
        HTTPException 500: If LLM API fails or internal error occurs
    """
    try:
        result = await add_follow_up_message(battle_id, data.prompt, db)
        return result

    except ValueError as e:
        error_msg = str(e).lower()
        if "not found" in error_msg:
            logger.error(f"Battle not found: {battle_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Battle not found: {battle_id}",
            )
        elif "status" in error_msg or "voted" in error_msg:
            logger.error(f"Cannot add message to battle {battle_id}: {e}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=str(e),
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=str(e),
            )

    except Exception as e:
        logger.error(f"Failed to add message to battle {battle_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to add message: {str(e)}",
        )


@router.post(
    "/battles/{battle_id}/vote",
    response_model=VoteResponse,
    status_code=status.HTTP_200_OK,
)
async def vote_on_battle_endpoint(
    battle_id: str,
    data: VoteCreate,
    db: AsyncSession = Depends(get_db),
):
    """
    Submit vote on battle and reveal model identities

    Flow:
    1. User submits vote (left_better, right_better, tie, both_bad)
    2. System creates vote record with denormalized model IDs
    3. System updates battle.status to 'voted'
    4. System updates session.last_active_at
    5. Returns vote confirmation with revealed model identities

    Args:
        battle_id: Existing battle ID
        data: Vote request with vote choice
        db: Database session

    Returns:
        VoteResponse with battle_id, vote, revealed_models

    Raises:
        HTTPException 404: If battle not found
        HTTPException 400: If battle already voted
        HTTPException 500: If internal error occurs
    """
    try:
        result = await vote_on_battle(battle_id, data.vote, db)
        return result

    except ValueError as e:
        error_msg = str(e).lower()
        if "not found" in error_msg:
            logger.error(f"Battle not found: {battle_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Battle not found: {battle_id}",
            )
        elif "already voted" in error_msg:
            logger.error(f"Battle already voted: {battle_id}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Battle has already been voted: {battle_id}",
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=str(e),
            )

    except Exception as e:
        logger.error(f"Failed to vote on battle {battle_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to submit vote: {str(e)}",
        )
