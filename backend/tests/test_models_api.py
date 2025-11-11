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


def test_get_model_xml_success(client: TestClient):
    """Test GET /api/models/xml returns composed XML for valid robot and scene"""
    # Arrange
    robot_id = "franka"
    scene_id = "table"

    # Act
    response = client.get(f"/api/models/xml?robot_id={robot_id}&scene_id={scene_id}")

    # Assert
    assert response.status_code == 200
    assert response.headers["content-type"] == "text/plain; charset=utf-8"

    xml_content = response.text

    # Verify XML structure
    assert "<mujoco" in xml_content
    assert 'model="franka_table"' in xml_content
    assert "<compiler" in xml_content
    assert "<worldbody>" in xml_content
    assert "<actuator>" in xml_content
    assert "</mujoco>" in xml_content

    # Verify robot body is included (franka has joint names)
    assert "joint1" in xml_content
    assert "joint2" in xml_content
    assert "gripper" in xml_content

    # Verify scene body is included (table scene has table and objects)
    assert "table" in xml_content
    assert "red_cube" in xml_content or "blue_cube" in xml_content


def test_get_model_xml_robot_not_found(client: TestClient):
    """Test GET /api/models/xml returns 404 when robot not found"""
    # Arrange
    robot_id = "nonexistent_robot"
    scene_id = "table"

    # Act
    response = client.get(f"/api/models/xml?robot_id={robot_id}&scene_id={scene_id}")

    # Assert
    assert response.status_code == 404
    data = response.json()
    assert "detail" in data
    assert "not found" in data["detail"].lower()
    assert robot_id in data["detail"]


def test_get_model_xml_scene_not_found(client: TestClient):
    """Test GET /api/models/xml returns 404 when scene not found"""
    # Arrange
    robot_id = "franka"
    scene_id = "nonexistent_scene"

    # Act
    response = client.get(f"/api/models/xml?robot_id={robot_id}&scene_id={scene_id}")

    # Assert
    assert response.status_code == 404
    data = response.json()
    assert "detail" in data
    assert "not found" in data["detail"].lower()
    assert scene_id in data["detail"]


def test_get_model_xml_template_structure(client: TestClient):
    """Test GET /api/models/xml returns XML with correct template structure"""
    # Arrange
    robot_id = "franka"
    scene_id = "table"

    # Act
    response = client.get(f"/api/models/xml?robot_id={robot_id}&scene_id={scene_id}")

    # Assert
    assert response.status_code == 200
    xml_content = response.text

    # Verify template elements are present
    assert '<compiler angle="radian"/>' in xml_content
    assert '<option timestep="0.002"/>' in xml_content
    assert '<light pos="0 0 3"' in xml_content
    assert '<geom name="floor"' in xml_content

    # Verify actuators for 7 joints + gripper
    assert "motor1" in xml_content
    assert "motor7" in xml_content
    assert "gripper_motor" in xml_content
