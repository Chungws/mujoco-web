"""
Pytest configuration and fixtures for backend tests
"""

import asyncio
import os


# IMPORTANT: Set test database URL and mock LLM BEFORE importing app
# Use SQLite in-memory for fast, isolated tests
os.environ["POSTGRES_URI"] = "sqlite+aiosqlite:///:memory:"
os.environ["USE_MOCK_LLM"] = "true"
os.environ["MODELS_CONFIG_PATH"] = "config/models.yaml"  # Relative to backend dir

import pytest
import pytest_asyncio
from beanie import init_beanie
from fastapi.testclient import TestClient
from mongomock_motor import AsyncMongoMockClient
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from sqlmodel import SQLModel

from vlaarena_backend.main import app
from vlaarena_shared.mongodb_models import Episode


# Temporary: Comment out LLM client imports (not needed for VLA Arena MVP)
# from vlaarena_backend.services.llm_client import (
#     MockLLMClient,
#     reset_llm_client,
#     set_llm_client,
# )


# Test database URL - SQLite in-memory for isolated, fast tests
TEST_DATABASE_URL = "sqlite+aiosqlite:///:memory:"

# Create test engine with StaticPool for in-memory SQLite
# StaticPool keeps single connection for :memory: DB
test_engine = create_async_engine(
    TEST_DATABASE_URL,
    echo=False,
    connect_args={"check_same_thread": False},  # Required for SQLite
    poolclass=StaticPool,  # Keep single connection for :memory: DB
)

# Create test session factory
test_async_session_maker = sessionmaker(
    test_engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False,
)


@pytest.fixture(scope="session")
def event_loop():
    """Create event loop for async tests"""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest_asyncio.fixture(scope="function")
async def db():
    """Database session fixture for tests"""
    # Create all tables
    async with test_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    # Create session
    async with test_async_session_maker() as session:
        yield session

    # Drop all tables after test
    async with test_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.drop_all)


# Temporary: Comment out use_mock_llm fixture (not needed for VLA Arena MVP)
# @pytest.fixture(autouse=True)
# def use_mock_llm():
#     """
#     Automatically use mock LLM for all tests
#
#     This fixture runs for every test and ensures that tests
#     don't make real API calls to external LLM providers.
#     """
#     set_llm_client(MockLLMClient())
#     yield
#     reset_llm_client()  # Clean up after test


@pytest_asyncio.fixture(scope="function")
async def mongodb():
    """MongoDB test fixture with in-memory database using mongomock-motor

    Provides a fresh MongoDB database for each test with Beanie ODM initialized.
    All MongoDB operations in tests will use this in-memory database.
    """
    # Create in-memory MongoDB client
    client = AsyncMongoMockClient()

    # Initialize Beanie with Episode document model
    await init_beanie(
        database=client.test_db,
        document_models=[Episode],
    )

    yield client

    # Cleanup: Drop database after test
    await client.drop_database("test_db")


@pytest_asyncio.fixture(scope="function")
async def client(mongodb):
    """Test client for FastAPI app with in-memory SQLite database

    Each test gets a fresh in-memory database that is automatically
    destroyed when the test completes. No cleanup needed!
    """
    # Create all tables in in-memory DB
    async with test_engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    # Override app's database dependency to use test database
    async def override_get_db():
        async with test_async_session_maker() as session:
            try:
                yield session
                await session.commit()
            except Exception:
                await session.rollback()
                raise
            finally:
                await session.close()

    from vlaarena_backend.database import get_db

    app.dependency_overrides[get_db] = override_get_db

    # Create test client
    with TestClient(app) as test_client:
        yield test_client

    # Clear dependency overrides
    app.dependency_overrides.clear()

    # Tables are automatically dropped when in-memory DB is closed
    # No explicit cleanup needed for :memory: database
