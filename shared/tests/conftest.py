"""
Pytest configuration and fixtures for shared package tests
"""

from typing import AsyncGenerator

import pytest_asyncio
from beanie import init_beanie
from motor.motor_asyncio import AsyncIOMotorClient

from vlaarena_shared.mongodb_models import Episode


@pytest_asyncio.fixture
async def mongodb_client() -> AsyncGenerator[AsyncIOMotorClient, None]:
    """
    Provide MongoDB client for tests

    Uses test database to avoid polluting production data
    """
    client = AsyncIOMotorClient("mongodb://admin:admin@localhost:27017")
    yield client
    client.close()


@pytest_asyncio.fixture
async def mongodb_database(mongodb_client: AsyncIOMotorClient):
    """
    Provide test database

    Cleans up after each test
    """
    db = mongodb_client["vlaarena_test"]

    # Initialize Beanie
    await init_beanie(database=db, document_models=[Episode])

    yield db

    # Clean up: drop all collections
    await db.drop_collection("episodes")
