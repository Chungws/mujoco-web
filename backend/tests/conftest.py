"""
Pytest configuration and fixtures for backend tests
"""

import os

# IMPORTANT: Set test database URL BEFORE importing app
# Use SQLite in-memory for fast, isolated tests
os.environ["POSTGRES_URI"] = "sqlite+aiosqlite:///:memory:"
os.environ["MODELS_CONFIG_PATH"] = "config/models.yaml"  # Relative to backend dir

import random

import pytest
import pytest_asyncio
import respx
from beanie import init_beanie
from fastapi.testclient import TestClient
from httpx import Response
from mongomock_motor import AsyncMongoMockClient
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from sqlmodel import SQLModel
from vlaarena_backend.main import app
from vlaarena_shared.mongodb_models import Episode

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


# Event loop fixture removed - pytest-asyncio provides this automatically
# Using default asyncio_mode = "auto" from pytest-asyncio


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

    yield client.test_db

    # Cleanup: Drop database after test
    await client.drop_database("test_db")


@pytest_asyncio.fixture(scope="function")
async def mongodb_database(mongodb):
    """Alias for mongodb fixture (for compatibility with shared tests)"""
    return mongodb


def _generate_mock_episode_response(model_id: str, instruction: str) -> dict:
    """
    Generate mock VLA episode response

    Simulates the response from VLA servers (/predict endpoint)
    Used for HTTP mocking in unit tests.
    """
    # Use model-specific seed for deterministic but different behavior
    model_seeds = {"openvla-7b": 42, "octo-base": 123, "mock-vla": 0, "octo-small": 456}
    seed = model_seeds.get(model_id, 0)
    seed += hash(instruction) % 1000
    random.seed(seed)

    # Generate variable length episode (20-50 steps)
    num_steps = random.randint(20, 50)

    # Generate 8-dim actions
    actions = [[random.uniform(-0.1, 0.1) for _ in range(8)] for _ in range(num_steps)]

    # Generate states (qpos, qvel, time)
    states = []
    for step in range(num_steps):
        qpos = [random.uniform(-1.0, 1.0) for _ in range(7)]
        qvel = [random.uniform(-0.5, 0.5) for _ in range(7)]
        states.append({"qpos": qpos, "qvel": qvel, "time": step * 0.1})

    return {"actions": actions, "states": states, "duration_ms": random.randint(100, 500)}


@pytest.fixture
def mock_vla_http():
    """
    Fixture to mock VLA server HTTP responses

    Mocks all POST requests to */predict endpoints with realistic episode data.
    This allows unit tests to run without actual VLA servers running.
    """
    with respx.mock:
        # Mock all VLA server predict endpoints
        route = respx.post(url__regex=r".*\/predict$").mock(
            side_effect=lambda request: Response(
                200,
                json=_generate_mock_episode_response(
                    model_id="mock",  # Default model_id
                    instruction=request.content.decode() if request.content else "Pick up the cube",
                ),
            )
        )
        yield route


@pytest_asyncio.fixture(scope="function")
async def client(mongodb, mock_vla_http):
    """Test client for FastAPI app with in-memory SQLite database

    Each test gets a fresh in-memory database that is automatically
    destroyed when the test completes. No cleanup needed!

    Uses mock_vla_http to mock VLA server HTTP responses.
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
