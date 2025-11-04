"""
Leaderboard API endpoints
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_backend.database import get_db
from vlaarena_backend.services.leaderboard_service import LeaderboardService
from vlaarena_shared.config import settings
from vlaarena_shared.schemas import LeaderboardResponse


logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/leaderboard", response_model=LeaderboardResponse, status_code=status.HTTP_200_OK)
async def get_leaderboard(
    db: AsyncSession = Depends(get_db),
):
    """
    Get leaderboard with ELO-based rankings

    Flow:
    1. Query model_stats table from PostgreSQL
    2. Filter models with vote_count >= 5 (minimum threshold)
    3. Sort by elo_score descending (fixed rank: 1 = highest ELO)
    4. Assign ranks (1, 2, 3, ...)
    5. Calculate metadata (total models, total votes, last updated)

    Note: Sorting/filtering is handled client-side for better performance

    Args:
        db: Database session

    Returns:
        LeaderboardResponse with ranked models and metadata (sorted by ELO score)

    Raises:
        HTTPException 500: If database query fails
    """
    try:
        service = LeaderboardService(db)
        leaderboard = await service.get_leaderboard(
            min_vote_count=settings.min_votes_for_leaderboard
        )
        return leaderboard

    except Exception as e:
        logger.error(f"Failed to get leaderboard: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get leaderboard: {str(e)}",
        )
