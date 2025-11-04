"""
ModelStats repository for leaderboard data access
"""

from typing import List

from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_shared.models import ModelStats

from .base import BaseRepository


class ModelStatsRepository(BaseRepository[ModelStats]):
    """Repository for ModelStats model operations"""

    def __init__(self, db: AsyncSession):
        super().__init__(ModelStats, db)

    async def get_leaderboard(
        self,
        min_vote_count: int = 5,
    ) -> List[ModelStats]:
        """
        Get leaderboard models sorted by ELO score

        Args:
            min_vote_count: Minimum number of votes required to appear on leaderboard

        Returns:
            List of ModelStats instances sorted by elo_score descending
        """
        stmt = (
            select(ModelStats)
            .where(ModelStats.vote_count >= min_vote_count)
            .order_by(ModelStats.elo_score.desc())
        )
        result = await self.db.execute(stmt)
        return list(result.scalars().all())

    async def get_total_votes(self, min_vote_count: int = 5) -> int:
        """
        Get total number of votes across all models meeting minimum threshold

        Args:
            min_vote_count: Minimum number of votes required

        Returns:
            Sum of vote_count for all models with vote_count >= min_vote_count
        """
        stmt = select(func.sum(ModelStats.vote_count)).where(
            ModelStats.vote_count >= min_vote_count
        )
        result = await self.db.execute(stmt)
        total = result.scalar()
        return total if total is not None else 0
