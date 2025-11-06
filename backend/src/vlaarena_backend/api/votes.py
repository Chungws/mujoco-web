"""
Votes API Router - VLA Arena MVP
Following FastAPI 4-layer pattern: schemas → service → router
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_backend.database import get_db
from vlaarena_backend.services.votes_service import create_vote
from vlaarena_shared.schemas import VoteRequest, VoteResponse


router = APIRouter()


@router.post(
    "/votes",
    response_model=VoteResponse,
    status_code=status.HTTP_201_CREATED,
    tags=["votes"],
)
async def submit_vote(
    vote_data: VoteRequest,
    db: AsyncSession = Depends(get_db),
):
    """
    Submit a vote for a battle and reveal model identities

    Args:
        vote_data: Vote request (battle_id, vote)
        db: Database session

    Returns:
        VoteResponse with vote_id and revealed model identities

    Raises:
        400: Battle not found or battle already has a vote
        422: Invalid request data
    """
    try:
        result = await create_vote(db=db, vote_data=vote_data)
        return result
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        ) from e
