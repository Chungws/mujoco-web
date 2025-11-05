"""
Tests for MongoDB connection module

Tests connection, disconnection, and client retrieval
"""

import pytest
from vlaarena_shared.mongodb import (
    connect_mongodb,
    disconnect_mongodb,
    get_mongodb_client,
)


@pytest.mark.asyncio
async def test_connect_mongodb():
    """Test MongoDB connection"""
    # Arrange
    mongodb_uri = "mongodb://admin:admin@localhost:27017"
    database_name = "vlaarena_test"

    # Act
    await connect_mongodb(mongodb_uri, database_name)

    # Assert
    client = get_mongodb_client()
    assert client is not None

    # Cleanup
    await disconnect_mongodb()


@pytest.mark.asyncio
async def test_disconnect_mongodb():
    """Test MongoDB disconnection"""
    # Arrange
    mongodb_uri = "mongodb://admin:admin@localhost:27017"
    database_name = "vlaarena_test"
    await connect_mongodb(mongodb_uri, database_name)

    # Act
    await disconnect_mongodb()

    # Assert
    with pytest.raises(RuntimeError, match="MongoDB is not connected"):
        get_mongodb_client()


@pytest.mark.asyncio
async def test_get_mongodb_client_not_connected():
    """Test get_mongodb_client raises error when not connected"""
    # Arrange
    # Ensure disconnected
    await disconnect_mongodb()

    # Act & Assert
    with pytest.raises(RuntimeError, match="MongoDB is not connected"):
        get_mongodb_client()


@pytest.mark.asyncio
async def test_get_mongodb_client_after_connect():
    """Test get_mongodb_client returns client after connection"""
    # Arrange
    mongodb_uri = "mongodb://admin:admin@localhost:27017"
    database_name = "vlaarena_test"
    await connect_mongodb(mongodb_uri, database_name)

    # Act
    client = get_mongodb_client()

    # Assert
    assert client is not None
    assert hasattr(client, "list_database_names")

    # Cleanup
    await disconnect_mongodb()


@pytest.mark.asyncio
async def test_reconnect_after_disconnect():
    """Test reconnecting after disconnect"""
    # Arrange
    mongodb_uri = "mongodb://admin:admin@localhost:27017"
    database_name = "vlaarena_test"

    # Act
    await connect_mongodb(mongodb_uri, database_name)
    await disconnect_mongodb()
    await connect_mongodb(mongodb_uri, database_name)

    # Assert
    client = get_mongodb_client()
    assert client is not None

    # Cleanup
    await disconnect_mongodb()
