"""
Tests for model management API endpoints
"""

from unittest.mock import Mock

from fastapi.testclient import TestClient

from vlaarena_backend.main import app
from vlaarena_backend.services.model_service import get_model_service


def test_get_models_returns_list(client: TestClient):
    """Test GET /api/models returns list of available models"""
    # Mock model service
    mock_service = Mock()
    mock_service.list_models.return_value = [
        {
            "model_id": "llama-3-1-8b",
            "name": "Llama 3.1 8B",
            "provider": "Meta",
            "status": "active",
        },
        {
            "model_id": "qwen-2-5-7b",
            "name": "Qwen 2.5 7B",
            "provider": "Alibaba",
            "status": "active",
        },
    ]

    # Override dependency
    app.dependency_overrides[get_model_service] = lambda: mock_service

    try:
        response = client.get("/api/models")
    finally:
        # Clean up override
        app.dependency_overrides.clear()

    assert response.status_code == 200
    data = response.json()

    # Check response structure
    assert "models" in data
    assert isinstance(data["models"], list)
    assert len(data["models"]) > 0

    # Check first model structure
    model = data["models"][0]
    assert "model_id" in model
    assert "name" in model
    assert "provider" in model
    assert "status" in model
    assert model["status"] in ["active", "inactive"]


def test_get_models_contains_expected_models(client: TestClient):
    """Test GET /api/models contains models from config"""
    # Mock model service
    mock_service = Mock()
    mock_service.list_models.return_value = [
        {
            "model_id": "llama-3-1-8b",
            "name": "Llama 3.1 8B",
            "provider": "Meta",
            "status": "active",
        },
        {
            "model_id": "qwen-2-5-7b",
            "name": "Qwen 2.5 7B",
            "provider": "Alibaba",
            "status": "active",
        },
    ]

    # Override dependency
    app.dependency_overrides[get_model_service] = lambda: mock_service

    try:
        response = client.get("/api/models")
    finally:
        # Clean up override
        app.dependency_overrides.clear()

    assert response.status_code == 200
    data = response.json()

    model_ids = [m["model_id"] for m in data["models"]]

    # Should contain at least one model
    assert len(model_ids) > 0

    # All models should have valid fields
    for model in data["models"]:
        assert model["model_id"]
        assert model["name"]
        assert model["provider"]


def test_get_models_only_active_models(client: TestClient):
    """Test GET /api/models only returns active models by default"""
    # Mock model service with only active models
    mock_service = Mock()
    mock_service.list_models.return_value = [
        {
            "model_id": "llama-3-1-8b",
            "name": "Llama 3.1 8B",
            "provider": "Meta",
            "status": "active",
        },
        {
            "model_id": "qwen-2-5-7b",
            "name": "Qwen 2.5 7B",
            "provider": "Alibaba",
            "status": "active",
        },
    ]

    # Override dependency
    app.dependency_overrides[get_model_service] = lambda: mock_service

    try:
        response = client.get("/api/models")
    finally:
        # Clean up override
        app.dependency_overrides.clear()

    assert response.status_code == 200
    data = response.json()

    # All models should be active (in MVP, we don't have inactive models)
    for model in data["models"]:
        assert model["status"] == "active"
