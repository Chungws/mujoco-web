"""
Tests for VLA Arena Session API endpoints
Following TDD workflow: Red → Green → Refactor
"""

from fastapi.testclient import TestClient


class TestSessionInitAPI:
    """Test POST /api/sessions/init endpoint (VLA Arena MVP)"""

    def test_init_session_success(self, client: TestClient):
        """
        Test successful session initialization via API

        Scenario:
        1. User provides robot_id and scene_id
        2. System creates session and battle
        3. Returns session_id, battle_id, and hidden model names
        """
        # Arrange
        data = {
            "robot_id": "widowx",
            "scene_id": "table",
        }

        # Act
        response = client.post("/api/sessions/init", json=data)

        # Assert
        assert response.status_code == 201
        result = response.json()

        # Check response structure
        assert "session_id" in result
        assert "battle_id" in result
        assert "left_model" in result
        assert "right_model" in result

        # Check session_id format
        assert result["session_id"].startswith("sess_")

        # Check battle_id format
        assert result["battle_id"].startswith("battle_")

        # Check models are hidden (blind A/B testing)
        assert result["left_model"] == "???"
        assert result["right_model"] == "???"

    def test_init_session_missing_robot_id(self, client: TestClient):
        """
        Test init_session fails when robot_id is missing

        Scenario:
        1. User omits robot_id from request
        2. Returns 422 validation error
        """
        # Arrange
        data = {
            "scene_id": "table",
        }

        # Act
        response = client.post("/api/sessions/init", json=data)

        # Assert
        assert response.status_code == 422
        detail = response.json()["detail"]
        # Check that error mentions robot_id
        assert any("robot_id" in str(error).lower() for error in detail)

    def test_init_session_missing_scene_id(self, client: TestClient):
        """
        Test init_session fails when scene_id is missing

        Scenario:
        1. User omits scene_id from request
        2. Returns 422 validation error
        """
        # Arrange
        data = {
            "robot_id": "widowx",
        }

        # Act
        response = client.post("/api/sessions/init", json=data)

        # Assert
        assert response.status_code == 422
        detail = response.json()["detail"]
        # Check that error mentions scene_id
        assert any("scene_id" in str(error).lower() for error in detail)

    def test_init_session_empty_robot_id(self, client: TestClient):
        """
        Test init_session fails with empty robot_id

        Scenario:
        1. User provides empty string for robot_id
        2. Returns 422 validation error
        """
        # Arrange
        data = {
            "robot_id": "",
            "scene_id": "table",
        }

        # Act
        response = client.post("/api/sessions/init", json=data)

        # Assert
        assert response.status_code == 422

    def test_init_session_empty_scene_id(self, client: TestClient):
        """
        Test init_session fails with empty scene_id

        Scenario:
        1. User provides empty string for scene_id
        2. Returns 422 validation error
        """
        # Arrange
        data = {
            "robot_id": "widowx",
            "scene_id": "",
        }

        # Act
        response = client.post("/api/sessions/init", json=data)

        # Assert
        assert response.status_code == 422

    def test_init_session_different_robots(self, client: TestClient):
        """
        Test init_session with different robot_ids

        Scenario:
        1. Create sessions for widowx, franka, ur5 (future robots)
        2. All should succeed
        """
        # Arrange
        robot_ids = ["widowx", "franka", "ur5"]

        # Act & Assert
        for robot_id in robot_ids:
            data = {
                "robot_id": robot_id,
                "scene_id": "table",
            }
            response = client.post("/api/sessions/init", json=data)

            assert response.status_code == 201
            result = response.json()
            assert result["session_id"].startswith("sess_")
            assert result["battle_id"].startswith("battle_")

    def test_init_session_different_scenes(self, client: TestClient):
        """
        Test init_session with different scene_ids

        Scenario:
        1. Create sessions for table, kitchen, warehouse (future scenes)
        2. All should succeed
        """
        # Arrange
        scene_ids = ["table", "kitchen", "warehouse"]

        # Act & Assert
        for scene_id in scene_ids:
            data = {
                "robot_id": "widowx",
                "scene_id": scene_id,
            }
            response = client.post("/api/sessions/init", json=data)

            assert response.status_code == 201
            result = response.json()
            assert result["session_id"].startswith("sess_")
            assert result["battle_id"].startswith("battle_")

    def test_init_session_multiple_calls_create_different_sessions(self, client: TestClient):
        """
        Test that multiple calls to init_session create separate sessions

        Scenario:
        1. Call init_session multiple times with same params
        2. Each call should create a new session and battle
        """
        # Arrange
        data = {
            "robot_id": "widowx",
            "scene_id": "table",
        }

        # Act - Create 3 sessions
        responses = []
        for _ in range(3):
            response = client.post("/api/sessions/init", json=data)
            responses.append(response)

        # Assert - All should succeed with different IDs
        session_ids = set()
        battle_ids = set()

        for response in responses:
            assert response.status_code == 201
            result = response.json()

            session_ids.add(result["session_id"])
            battle_ids.add(result["battle_id"])

        # All IDs should be unique
        assert len(session_ids) == 3
        assert len(battle_ids) == 3
