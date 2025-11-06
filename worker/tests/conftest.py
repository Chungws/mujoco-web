"""
Pytest configuration and fixtures for worker tests
"""

import os

# IMPORTANT: Set test database URL BEFORE importing any modules
# Use SQLite in-memory for fast, isolated tests
os.environ["POSTGRES_URI"] = "sqlite+aiosqlite:///:memory:"

import pytest_asyncio
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from sqlmodel import SQLModel


# Test database URL - SQLite in-memory for isolated, fast tests
TEST_DATABASE_URL = "sqlite+aiosqlite:///:memory:"


@pytest_asyncio.fixture(scope="function")
async def test_db_session():
    """Database session fixture for tests"""
    # Create engine with StaticPool for in-memory SQLite
    # StaticPool keeps single connection for :memory: DB
    engine = create_async_engine(
        TEST_DATABASE_URL,
        echo=False,
        connect_args={"check_same_thread": False},  # Required for SQLite
        poolclass=StaticPool,  # Keep single connection for :memory: DB
    )

    # Create all tables
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    # Create session
    async_session_maker = sessionmaker(
        engine,
        class_=AsyncSession,
        expire_on_commit=False,
        autocommit=False,
        autoflush=False,
    )

    async with async_session_maker() as session:
        yield session

    # Drop all tables after test
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.drop_all)

    # Dispose engine
    await engine.dispose()
