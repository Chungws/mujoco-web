"""
MongoDB connection management using Beanie ODM

Provides async MongoDB connection and initialization for VLA Arena.
"""

from beanie import init_beanie
from motor.motor_asyncio import AsyncIOMotorClient

from .mongodb_models import Episode

# Global MongoDB client
mongodb_client: AsyncIOMotorClient | None = None


async def connect_mongodb(mongodb_uri: str, database_name: str) -> None:
    """
    Initialize MongoDB connection and Beanie ODM

    Args:
        mongodb_uri: MongoDB connection URI
        database_name: Database name to use

    Example:
        await connect_mongodb(
            "mongodb://admin:admin@localhost:27017",
            "vlaarena"
        )
    """
    global mongodb_client

    # Create MongoDB client
    mongodb_client = AsyncIOMotorClient(mongodb_uri)

    # Initialize Beanie with Episode document
    await init_beanie(
        database=mongodb_client[database_name],
        document_models=[Episode],
    )


async def disconnect_mongodb() -> None:
    """
    Close MongoDB connection gracefully
    """
    global mongodb_client

    if mongodb_client is not None:
        mongodb_client.close()
        mongodb_client = None


def get_mongodb_client() -> AsyncIOMotorClient:
    """
    Get MongoDB client instance

    Returns:
        AsyncIOMotorClient instance

    Raises:
        RuntimeError: If MongoDB is not connected
    """
    if mongodb_client is None:
        raise RuntimeError("MongoDB is not connected. Call connect_mongodb() first.")
    return mongodb_client
