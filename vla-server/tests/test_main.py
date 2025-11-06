"""
Tests for main FastAPI application
"""

from fastapi.testclient import TestClient

from vla_server.main import app

client = TestClient(app)


def test_root_endpoint():
    """
    Test root endpoint returns service info

    Arrange: Create test client
    Act: Call root endpoint
    Assert: Returns service name and version
    """
    # Act
    response = client.get("/")

    # Assert
    assert response.status_code == 200
    data = response.json()
    assert data["service"] == "VLA Execution Server"
    assert data["version"] == "0.1.0"
    assert data["status"] == "running"


def test_health_endpoint():
    """
    Test health check endpoint

    Arrange: Create test client
    Act: Call health endpoint
    Assert: Returns healthy status
    """
    # Act
    response = client.get("/health")

    # Assert
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "device" in data
    assert "max_steps" in data
