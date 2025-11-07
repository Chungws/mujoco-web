"""
Battle repository for database operations
"""

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_shared.models import Battle

from .base import BaseRepository


class BattleRepository(BaseRepository[Battle]):
    """Repository for Battle model operations"""

    def __init__(self, db: AsyncSession):
        super().__init__(Battle, db)

    async def get_by_battle_id(self, battle_id: str) -> Battle | None:
        """
        Get battle by battle_id string

        Args:
            battle_id: Unique battle identifier (e.g., "battle_xyz789")

        Returns:
            Battle instance or None if not found
        """
        return await self.get_by_field("battle_id", battle_id)

    async def get_by_session_id(self, session_id: str, limit: int | None = None) -> list[Battle]:
        """
        Get all battles for a session

        Args:
            session_id: Session ID string
            limit: Optional limit on number of battles

        Returns:
            List of battle instances ordered by creation time
        """
        stmt = (
            select(Battle).where(Battle.session_id == session_id).order_by(Battle.created_at.desc())
        )
        if limit:
            stmt = stmt.limit(limit)
        result = await self.db.execute(stmt)
        return list(result.scalars().all())

    async def get_ongoing_battles(self, limit: int | None = None) -> list[Battle]:
        """
        Get all ongoing battles (not voted yet)

        Args:
            limit: Optional limit on number of battles

        Returns:
            List of ongoing battle instances
        """
        stmt = select(Battle).where(Battle.status == "ongoing").order_by(Battle.created_at.desc())
        if limit:
            stmt = stmt.limit(limit)
        result = await self.db.execute(stmt)
        return list(result.scalars().all())
