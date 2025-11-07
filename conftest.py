"""
Root pytest configuration for all test packages

Provides shared fixtures for:
- Mock PostgreSQL (SQLite in-memory)
- Mock MongoDB (mongomock-motor)

These fixtures are available to all test packages (backend, worker, shared).
"""

import os

# IMPORTANT: Set test database URL BEFORE importing any SQLModel/database code
os.environ["POSTGRES_URI"] = "sqlite+aiosqlite:///:memory:"
os.environ["USE_MOCK_LLM"] = "true"

import pytest_asyncio
from beanie import init_beanie
from mongomock_motor import AsyncMongoMockClient
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from sqlmodel import SQLModel
from vlaarena_shared.mongodb_models import Episode

# Test database URL - SQLite in-memory for isolated, fast tests
TEST_DATABASE_URL = "sqlite+aiosqlite:///:memory:"

# Create test engine with StaticPool for in-memory SQLite
test_engine = create_async_engine(
    TEST_DATABASE_URL,
    echo=False,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)

# Create test session factory
test_async_session_maker = sessionmaker(
    test_engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False,
)


@pytest_asyncio.fixture(scope="function")
async def db():
    """PostgreSQL database session fixture (SQLite in-memory)"""
    # Create all tables
    async with test_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    # Create session
    async with test_async_session_maker() as session:
        yield session

    # Drop all tables after test
    async with test_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.drop_all)


@pytest_asyncio.fixture(scope="function")
async def mongodb():
    """MongoDB test fixture with in-memory database using mongomock-motor"""
    # Create in-memory MongoDB client
    client = AsyncMongoMockClient()

    # Initialize Beanie with Episode document model
    await init_beanie(
        database=client.test_db,
        document_models=[Episode],
    )

    yield client.test_db

    # Cleanup: Drop all collections
    await client.drop_database("test_db")


@pytest_asyncio.fixture(scope="function")
async def mongodb_database(mongodb):
    """Alias for mongodb fixture (for backward compatibility)"""
    return mongodb
