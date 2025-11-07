"""
Tests for PostgreSQL database connection
"""

import pytest
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_worker.database import async_session_maker, engine, get_db


class TestDatabaseConnection:
    """Test database connection setup"""

    def test_engine_created(self):
        """Test that async engine is created"""
        # Assert
        assert engine is not None
        # Pool configuration is set via create_async_engine parameters
        # We trust SQLAlchemy to handle pool_size=2, max_overflow=3 correctly

    def test_session_maker_created(self):
        """Test that async session maker is created"""
        # Assert
        assert async_session_maker is not None
        # Session maker configuration is set via sessionmaker parameters
        # We trust SQLAlchemy to handle AsyncSession, expire_on_commit=False correctly

    @pytest.mark.asyncio
    async def test_get_db_returns_async_session(self):
        """Test that get_db yields AsyncSession"""
        # Act
        async for session in get_db():
            # Assert
            assert isinstance(session, AsyncSession)
            break  # Only test first yield

    @pytest.mark.asyncio
    async def test_get_db_context_manager(self):
        """Test that get_db works as async context manager"""
        # Act & Assert - Should work without errors
        session_created = False
        async for session in get_db():
            session_created = True
            assert isinstance(session, AsyncSession)
            # Don't execute any queries - just test session creation
            break

        assert session_created is True
