"""
Tests for VLA Arena Votes API endpoints
Following TDD workflow: Red → Green → Refactor

Test Coverage:
- POST /api/votes: Success, battle not found, duplicate vote, validation errors
"""

from fastapi.testclient import TestClient


class TestVotesAPI:
    """Test POST /api/votes endpoint (VLA Arena MVP)"""

    def test_create_vote_success_left_better(self, client: TestClient):
        """
        Test successful vote submission for left model

        Scenario:
        1. User creates session and battle
        2. User votes for left model
        3. System reveals model identities
        4. Returns vote_id and revealed models
        """
        # Arrange - Create session and battle first
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        assert session_response.status_code == 201
        session_data = session_response.json()
        battle_id = session_data["battle_id"]

        vote_data = {
            "battle_id": battle_id,
            "vote": "left_better",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 201
        result = response.json()

        # Check response structure
        assert "vote_id" in result
        assert "revealed_models" in result

        # Check vote_id format
        assert result["vote_id"].startswith("vote_")

        # Check revealed models
        revealed = result["revealed_models"]
        assert "left" in revealed
        assert "right" in revealed
        assert revealed["left"] != "???"  # Models should be revealed
        assert revealed["right"] != "???"

    def test_create_vote_success_right_better(self, client: TestClient):
        """
        Test successful vote submission for right model

        Scenario:
        1. User creates session and battle
        2. User votes for right model
        3. Returns vote_id and revealed models
        """
        # Arrange
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        vote_data = {
            "battle_id": battle_id,
            "vote": "right_better",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 201
        result = response.json()
        assert result["vote_id"].startswith("vote_")
        assert "revealed_models" in result

    def test_create_vote_success_tie(self, client: TestClient):
        """
        Test successful vote submission for tie

        Scenario:
        1. User creates session and battle
        2. User votes tie
        3. Returns vote_id and revealed models
        """
        # Arrange
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        vote_data = {
            "battle_id": battle_id,
            "vote": "tie",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 201
        result = response.json()
        assert result["vote_id"].startswith("vote_")

    def test_create_vote_success_both_bad(self, client: TestClient):
        """
        Test successful vote submission for both_bad

        Scenario:
        1. User creates session and battle
        2. User votes both_bad
        3. Returns vote_id and revealed models
        """
        # Arrange
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        vote_data = {
            "battle_id": battle_id,
            "vote": "both_bad",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 201
        result = response.json()
        assert result["vote_id"].startswith("vote_")

    def test_create_vote_battle_not_found(self, client: TestClient):
        """
        Test vote creation fails when battle doesn't exist

        Scenario:
        1. User submits vote for non-existent battle
        2. Returns 400 Bad Request
        """
        # Arrange
        vote_data = {
            "battle_id": "nonexistent_battle",
            "vote": "left_better",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 400
        result = response.json()
        assert "detail" in result
        assert "battle not found" in result["detail"].lower()

    def test_create_vote_duplicate_battle(self, client: TestClient):
        """
        Test vote creation fails when battle already has a vote

        Scenario:
        1. User creates session and battle
        2. User submits first vote - succeeds
        3. User submits second vote for same battle - fails
        4. Returns 400 Bad Request
        """
        # Arrange
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        vote_data_1 = {
            "battle_id": battle_id,
            "vote": "left_better",
        }
        vote_data_2 = {
            "battle_id": battle_id,
            "vote": "right_better",
        }

        # Act - First vote succeeds
        response_1 = client.post("/api/votes", json=vote_data_1)
        assert response_1.status_code == 201

        # Act - Second vote fails
        response_2 = client.post("/api/votes", json=vote_data_2)

        # Assert
        assert response_2.status_code == 400
        result = response_2.json()
        assert "detail" in result
        assert "already has a vote" in result["detail"].lower()

    def test_create_vote_missing_battle_id(self, client: TestClient):
        """
        Test vote creation fails when battle_id is missing

        Scenario:
        1. User omits battle_id from request
        2. Returns 422 validation error
        """
        # Arrange
        vote_data = {
            "vote": "left_better",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 422
        detail = response.json()["detail"]
        assert any("battle_id" in str(error).lower() for error in detail)

    def test_create_vote_missing_vote(self, client: TestClient):
        """
        Test vote creation fails when vote is missing

        Scenario:
        1. User omits vote from request
        2. Returns 422 validation error
        """
        # Arrange
        vote_data = {
            "battle_id": "battle_123",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 422
        detail = response.json()["detail"]
        assert any("vote" in str(error).lower() for error in detail)

    def test_create_vote_invalid_vote_value(self, client: TestClient):
        """
        Test vote creation fails with invalid vote value

        Scenario:
        1. User provides invalid vote value (not left_better/right_better/tie/both_bad)
        2. Returns 422 validation error
        """
        # Arrange
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        vote_data = {
            "battle_id": battle_id,
            "vote": "invalid_vote_type",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        assert response.status_code == 422

    def test_create_vote_empty_battle_id(self, client: TestClient):
        """
        Test vote creation fails with empty battle_id

        Scenario:
        1. User provides empty battle_id
        2. Returns 400 Bad Request (battle not found)
        """
        # Arrange
        vote_data = {
            "battle_id": "",
            "vote": "left_better",
        }

        # Act
        response = client.post("/api/votes", json=vote_data)

        # Assert
        # Empty string may fail at validation or service layer
        assert response.status_code in [400, 422]
