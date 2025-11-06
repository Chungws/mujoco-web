"""
Leaderboard business logic service
"""

from typing import List

from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_shared.models import ModelStatsByRobot, ModelStatsTotal
from vlaarena_shared.schemas import LeaderboardResponse, ModelRanking

from ..repositories.model_stats_repository import ModelStatsRepository


class LeaderboardService:
    """Service for leaderboard operations"""

    def __init__(self, db: AsyncSession):
        self.db = db
        self.model_stats_repo = ModelStatsRepository(db)

    async def get_leaderboard(
        self, min_vote_count: int = 5, robot_id: str | None = None
    ) -> LeaderboardResponse:
        """
        Get leaderboard with model rankings

        Args:
            min_vote_count: Minimum number of votes required to appear on leaderboard
            robot_id: If provided, get robot-specific leaderboard. Otherwise, get global.

        Returns:
            LeaderboardResponse with ranked models sorted by ELO score descending
        """
        # Get models from repository (sorted by elo_score desc)
        models = await self.model_stats_repo.get_leaderboard(min_vote_count, robot_id)

        # Build rankings
        rankings = self._build_rankings(models)

        return LeaderboardResponse(rankings=rankings)

    def _build_rankings(
        self,
        models: List[ModelStatsByRobot | ModelStatsTotal],
    ) -> List[ModelRanking]:
        """
        Build model rankings from stats

        Args:
            models: List of ModelStatsByRobot or ModelStatsTotal sorted by elo_score descending

        Returns:
            List of ModelRanking
        """
        rankings = []
        for model in models:
            ranking = ModelRanking(
                model_id=model.model_id,
                name=model.model_id,  # For MVP, name = model_id
                elo_score=model.elo_score,
                vote_count=model.vote_count,
                win_rate=model.win_rate,
            )
            rankings.append(ranking)

        return rankings
