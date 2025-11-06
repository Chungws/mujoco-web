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
    robot_id: str | None = None,
    db: AsyncSession = Depends(get_db),
):
    """
    Get leaderboard with ELO-based rankings (robot-specific or global)

    Flow:
    1. Query model_stats_by_robot (if robot_id) or model_stats_total (if None) from PostgreSQL
    2. Filter models with vote_count >= 5 (minimum threshold)
    3. Sort by elo_score descending (fixed rank: 1 = highest ELO)
    4. Assign ranks (1, 2, 3, ...)
    5. Calculate metadata (total models, total votes, last updated)

    Args:
        robot_id: Optional robot ID for robot-specific leaderboard.
                  If None, returns global leaderboard.
        db: Database session

    Returns:
        LeaderboardResponse with ranked models and metadata (sorted by ELO score)

    Raises:
        HTTPException 500: If database query fails
    """
    try:
        service = LeaderboardService(db)
        leaderboard = await service.get_leaderboard(
            min_vote_count=settings.min_votes_for_leaderboard, robot_id=robot_id
        )
        return leaderboard

    except Exception as e:
        logger.error(f"Failed to get leaderboard: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get leaderboard: {str(e)}",
        )
