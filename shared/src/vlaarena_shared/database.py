"""
Shared database connection and session management

Provides database connections for both backend API and worker with appropriate
connection pool configurations for each use case.
"""

from collections.abc import AsyncGenerator

from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlmodel import SQLModel

from vlaarena_shared.config import settings


def _create_engine_and_session_maker(
    pool_size: int, max_overflow: int, pool_timeout: int | None = None
):
    """
    Create async engine and session maker with specified pool configuration

    Args:
        pool_size: Number of connections to maintain in the pool
        max_overflow: Maximum overflow connections beyond pool_size
        pool_timeout: Timeout for getting connection from pool (optional)

    Returns:
        Tuple of (engine, session_maker)
    """
    # Check if using SQLite (for tests)
    is_sqlite = "sqlite" in settings.postgres_uri.lower()

    if is_sqlite:
        # SQLite doesn't support pool configuration
        from sqlalchemy.pool import StaticPool

        engine = create_async_engine(
            settings.postgres_uri,
            echo=False,
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
    else:
        # PostgreSQL with pool configuration
        engine = create_async_engine(
            settings.postgres_uri,
            echo=False,
            pool_size=pool_size,
            max_overflow=max_overflow,
            pool_timeout=pool_timeout,
        )

    session_maker = sessionmaker(
        engine,
        class_=AsyncSession,
        expire_on_commit=False,
        autocommit=False,
        autoflush=False,
    )

    return engine, session_maker


# Backend database (higher concurrency for API requests)
backend_engine, backend_session_maker = _create_engine_and_session_maker(
    pool_size=settings.postgres_pool_size,
    max_overflow=settings.postgres_max_overflow,
)

# Worker database (lower concurrency for batch operations)
worker_engine, worker_session_maker = _create_engine_and_session_maker(
    pool_size=settings.worker_pool_size,
    max_overflow=settings.worker_max_overflow,
    pool_timeout=settings.worker_pool_timeout,
)


async def get_backend_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Get database session for backend API operations

    Usage:
        @app.get("/endpoint")
        async def endpoint(db: AsyncSession = Depends(get_backend_db)):
            ...

    Yields:
        AsyncSession: Database session instance with auto commit/rollback
    """
    async with backend_session_maker() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()


async def get_worker_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Get database session for worker operations

    Usage:
        async for session in get_worker_db():
            result = await session.execute(...)
            await session.commit()

    Yields:
        AsyncSession: Database session instance with auto commit/rollback
    """
    async with worker_session_maker() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()


async def create_db_and_tables():
    """
    Create database tables (for testing)

    Note: In production, use Alembic migrations instead
    Uses backend_engine for table creation
    """
    async with backend_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)


async def drop_db_and_tables():
    """
    Drop all database tables (for testing cleanup)

    Uses backend_engine for table deletion
    """
    async with backend_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.drop_all)
