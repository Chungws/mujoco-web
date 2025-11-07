"""
ModelStats repository for leaderboard data access (robot-specific + global)
"""

from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_shared.models import ModelStatsByRobot, ModelStatsTotal

from .base import BaseRepository


class ModelStatsRepository(BaseRepository[ModelStatsTotal]):
    """Repository for ModelStats operations (supports both robot-specific and global)"""

    def __init__(self, db: AsyncSession):
        super().__init__(ModelStatsTotal, db)

    async def get_leaderboard(
        self,
        min_vote_count: int = 5,
        robot_id: str | None = None,
    ) -> list[ModelStatsByRobot | ModelStatsTotal]:
        """
        Get leaderboard models sorted by ELO score

        Args:
            min_vote_count: Minimum number of votes required to appear on leaderboard
            robot_id: If provided, get robot-specific leaderboard. Otherwise, get global.

        Returns:
            List of ModelStatsByRobot (if robot_id) or ModelStatsTotal (if None)
            sorted by elo_score descending
        """
        if robot_id:
            # Robot-specific leaderboard
            stmt = (
                select(ModelStatsByRobot)
                .where(
                    ModelStatsByRobot.robot_id == robot_id,
                    ModelStatsByRobot.vote_count >= min_vote_count,
                )
                .order_by(ModelStatsByRobot.elo_score.desc())
            )
        else:
            # Global leaderboard
            stmt = (
                select(ModelStatsTotal)
                .where(ModelStatsTotal.vote_count >= min_vote_count)
                .order_by(ModelStatsTotal.elo_score.desc())
            )

        result = await self.db.execute(stmt)
        return list(result.scalars().all())

    async def get_total_votes(self, min_vote_count: int = 5, robot_id: str | None = None) -> int:
        """
        Get total number of votes across all models meeting minimum threshold

        Args:
            min_vote_count: Minimum number of votes required
            robot_id: If provided, get robot-specific total. Otherwise, get global.

        Returns:
            Sum of vote_count for all models with vote_count >= min_vote_count
        """
        if robot_id:
            # Robot-specific total
            stmt = select(func.sum(ModelStatsByRobot.vote_count)).where(
                ModelStatsByRobot.robot_id == robot_id,
                ModelStatsByRobot.vote_count >= min_vote_count,
            )
        else:
            # Global total
            stmt = select(func.sum(ModelStatsTotal.vote_count)).where(
                ModelStatsTotal.vote_count >= min_vote_count
            )

        result = await self.db.execute(stmt)
        total = result.scalar()
        return total if total is not None else 0

    async def get_model_stats_by_robot(
        self, model_id: str, robot_id: str
    ) -> ModelStatsByRobot | None:
        """
        Get robot-specific model statistics

        Args:
            model_id: Model identifier
            robot_id: Robot identifier

        Returns:
            ModelStatsByRobot if found, None otherwise
        """
        stmt = select(ModelStatsByRobot).where(
            ModelStatsByRobot.model_id == model_id,
            ModelStatsByRobot.robot_id == robot_id,
        )
        result = await self.db.execute(stmt)
        return result.scalar_one_or_none()

    async def get_model_stats_total(self, model_id: str) -> ModelStatsTotal | None:
        """
        Get global model statistics

        Args:
            model_id: Model identifier

        Returns:
            ModelStatsTotal if found, None otherwise
        """
        stmt = select(ModelStatsTotal).where(ModelStatsTotal.model_id == model_id)
        result = await self.db.execute(stmt)
        return result.scalar_one_or_none()
