"""
Session repository for database operations
"""

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_shared.models import Session

from .base import BaseRepository


class SessionRepository(BaseRepository[Session]):
    """Repository for Session model operations"""

    def __init__(self, db: AsyncSession):
        super().__init__(Session, db)

    async def get_by_session_id(self, session_id: str) -> Session | None:
        """
        Get session by session_id string

        Args:
            session_id: Unique session identifier (e.g., "session_abc123")

        Returns:
            Session instance or None if not found
        """
        return await self.get_by_field("session_id", session_id)

    async def get_by_user_id(
        self, user_id: str, limit: int | None = None, offset: int | None = None
    ) -> list[Session]:
        """
        Get all sessions for a user

        Args:
            user_id: User ID (UUID string for anonymous users)
            limit: Optional limit on number of sessions
            offset: Optional offset for pagination

        Returns:
            List of session instances ordered by last_active_at DESC
        """
        stmt = (
            select(Session)
            .where(Session.user_id == user_id)
            .order_by(Session.last_active_at.desc())
        )
        if limit:
            stmt = stmt.limit(limit)
        if offset:
            stmt = stmt.offset(offset)
        result = await self.db.execute(stmt)
        return list(result.scalars().all())

    async def count_by_user_id(self, user_id: str) -> int:
        """
        Count total sessions for a user

        Args:
            user_id: User ID (UUID string for anonymous users)

        Returns:
            Total count of sessions for user
        """
        from sqlalchemy import func

        stmt = select(func.count()).select_from(Session).where(Session.user_id == user_id)
        result = await self.db.execute(stmt)
        return result.scalar_one()
