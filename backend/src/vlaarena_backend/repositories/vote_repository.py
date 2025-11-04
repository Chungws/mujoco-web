"""
Vote repository for database operations
"""

from typing import Optional

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_shared.models import Vote

from .base import BaseRepository


class VoteRepository(BaseRepository[Vote]):
    """Repository for Vote model operations"""

    def __init__(self, db: AsyncSession):
        super().__init__(Vote, db)

    async def get_by_vote_id(self, vote_id: str) -> Optional[Vote]:
        """
        Get vote by vote_id string

        Args:
            vote_id: Unique vote identifier (e.g., "vote_def456")

        Returns:
            Vote instance or None if not found
        """
        return await self.get_by_field("vote_id", vote_id)

    async def get_by_battle_id(self, battle_id: str) -> Optional[Vote]:
        """
        Get vote by battle_id (1:1 relationship)

        Args:
            battle_id: Battle ID string

        Returns:
            Vote instance or None if not found
        """
        return await self.get_by_field("battle_id", battle_id)

    async def get_pending_votes(self, limit: Optional[int] = None) -> list[Vote]:
        """
        Get all pending votes (not yet processed by worker)

        Args:
            limit: Optional limit on number of votes

        Returns:
            List of pending vote instances
        """
        stmt = select(Vote).where(Vote.processing_status == "pending").order_by(Vote.voted_at.asc())
        if limit:
            stmt = stmt.limit(limit)
        result = await self.db.execute(stmt)
        return list(result.scalars().all())
