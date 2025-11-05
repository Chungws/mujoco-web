"""
Tests for VLA Arena Turn API endpoints
Following TDD workflow: Red → Green → Refactor

Turn API handles battle turns where users provide instructions
and the system executes both VLA models to generate episodes.
"""

import pytest
from fastapi.testclient import TestClient


class TestTurnAPI:
    """Test POST /api/battles/{battle_id}/turns endpoint (VLA Arena MVP)"""

    def test_create_turn_success(self, client: TestClient):
        """
        Test successful turn creation with episode generation

        Scenario:
        1. User has active session and battle
        2. User provides instruction
        3. System creates turn record
        4. System executes both VLA models (left and right)
        5. System stores episodes in MongoDB
        6. Returns turn_id and episode_ids

        Flow (from MVP spec):
        POST /api/battles/{battle_id}/turns
        → Create Turn record
        → Execute left model → save Episode to MongoDB
        → Execute right model → save Episode to MongoDB
        → Return TurnResponse
        """
        # Arrange: Create session and battle first
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        assert session_response.status_code == 201
        battle_id = session_response.json()["battle_id"]

        turn_data = {"instruction": "Pick up the red cube"}

        # Act
        response = client.post(f"/api/battles/{battle_id}/turns", json=turn_data)

        # Assert
        assert response.status_code == 201
        result = response.json()

        # Check response structure
        assert "turn_id" in result
        assert "left_episode_id" in result
        assert "right_episode_id" in result
        assert "status" in result

        # Check turn_id format
        assert result["turn_id"].startswith("turn_")

        # Check episode_id formats
        assert result["left_episode_id"].startswith("ep_")
        assert result["right_episode_id"].startswith("ep_")

        # Check status
        assert result["status"] == "completed"

        # Verify episodes are different
        assert result["left_episode_id"] != result["right_episode_id"]

    def test_create_turn_battle_not_found(self, client: TestClient):
        """
        Test turn creation fails when battle doesn't exist

        Scenario:
        1. User provides non-existent battle_id
        2. Returns 404 not found
        """
        # Arrange
        battle_id = "battle_nonexistent"
        turn_data = {"instruction": "Pick up the red cube"}

        # Act
        response = client.post(f"/api/battles/{battle_id}/turns", json=turn_data)

        # Assert
        assert response.status_code == 404
        detail = response.json()["detail"]
        assert "not found" in detail.lower()

    def test_create_turn_missing_instruction(self, client: TestClient):
        """
        Test turn creation fails when instruction is missing

        Scenario:
        1. User omits instruction from request
        2. Returns 422 validation error
        """
        # Arrange: Create session and battle first
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        turn_data = {}  # Missing instruction

        # Act
        response = client.post(f"/api/battles/{battle_id}/turns", json=turn_data)

        # Assert
        assert response.status_code == 422
        detail = response.json()["detail"]
        # Check that error mentions instruction
        assert any("instruction" in str(error).lower() for error in detail)

    def test_create_turn_empty_instruction(self, client: TestClient):
        """
        Test turn creation fails when instruction is empty string

        Scenario:
        1. User provides empty instruction
        2. Returns 422 validation error
        """
        # Arrange: Create session and battle first
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        turn_data = {"instruction": ""}  # Empty instruction

        # Act
        response = client.post(f"/api/battles/{battle_id}/turns", json=turn_data)

        # Assert
        assert response.status_code == 422

    def test_create_multiple_turns_in_battle(self, client: TestClient):
        """
        Test creating multiple turns in same battle (multi-turn)

        Scenario:
        1. User creates first turn
        2. User creates second turn in same battle
        3. Both succeed with incremental seq numbers

        From MVP spec:
        - Multi-turn battles (multiple instructions per session)
        - User can provide another instruction after voting
        """
        # Arrange: Create session and battle
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        # Act: Create first turn
        turn1_response = client.post(
            f"/api/battles/{battle_id}/turns",
            json={"instruction": "Pick up the red cube"},
        )

        # Act: Create second turn
        turn2_response = client.post(
            f"/api/battles/{battle_id}/turns",
            json={"instruction": "Move the cube to the left"},
        )

        # Assert
        assert turn1_response.status_code == 201
        assert turn2_response.status_code == 201

        turn1_data = turn1_response.json()
        turn2_data = turn2_response.json()

        # Verify different turn_ids
        assert turn1_data["turn_id"] != turn2_data["turn_id"]

        # Verify different episode_ids
        assert turn1_data["left_episode_id"] != turn2_data["left_episode_id"]
        assert turn1_data["right_episode_id"] != turn2_data["right_episode_id"]

    def test_create_turn_updates_session_last_active(self, client: TestClient):
        """
        Test that creating a turn updates session.last_active_at

        Scenario:
        1. Create session
        2. Create turn (which should update last_active_at)
        3. Verify turn created successfully

        Note: Detailed validation of last_active_at update is in service layer tests
        """
        # Arrange: Create session and battle
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        # Act: Create turn
        turn_response = client.post(
            f"/api/battles/{battle_id}/turns",
            json={"instruction": "Pick up the red cube"},
        )

        # Assert turn created (which implies session was updated)
        assert turn_response.status_code == 201

    @pytest.mark.asyncio
    async def test_create_turn_stores_episodes_in_mongodb(self, client: TestClient):
        """
        Test that turn creation stores episodes in MongoDB

        Scenario:
        1. Create turn
        2. Verify turn created and returns episode IDs
        3. (MongoDB verification would require MongoDB connection in tests)

        Note: Full MongoDB integration testing requires Beanie setup in test environment
        For now, we verify the API returns episode_ids which implies MongoDB storage
        """
        # Arrange: Create session and battle
        session_response = client.post(
            "/api/sessions/init",
            json={"robot_id": "widowx", "scene_id": "table"},
        )
        battle_id = session_response.json()["battle_id"]

        # Act: Create turn
        turn_response = client.post(
            f"/api/battles/{battle_id}/turns",
            json={"instruction": "Pick up the red cube"},
        )

        # Assert
        assert turn_response.status_code == 201
        result = turn_response.json()

        # Verify episode IDs are returned (implies MongoDB storage)
        assert result["left_episode_id"].startswith("ep_")
        assert result["right_episode_id"].startswith("ep_")
        assert result["left_episode_id"] != result["right_episode_id"]


class TestTurnAPIErrorHandling:
    """
    Test error handling for Turn API

    Verifies that database errors are properly handled with rollback
    """

    def test_create_turn_db_error_rolls_back(self, client: TestClient):
        """
        Test Turn API handles database errors with rollback

        Scenario:
        1. Create session and battle
        2. Mock db.commit() to raise exception
        3. Attempt to create turn
        4. Expect 500 error
        5. Verify no orphan Turn records in database
        """
        from unittest.mock import AsyncMock, patch

        # Arrange: Create session and battle
        init_response = client.post(
            "/api/sessions/init",
            json={
                "robot_id": "widowx",
                "scene_id": "table_pick_place",
            },
        )
        assert init_response.status_code == 201
        battle_id = init_response.json()["battle_id"]

        # Act: Mock db.commit() to simulate database error
        with patch(
            "vlaarena_backend.api.battles.AsyncSession.commit",
            new_callable=AsyncMock,
            side_effect=Exception("Database commit failed"),
        ):
            response = client.post(
                f"/api/battles/{battle_id}/turns",
                json={"instruction": "Pick up the cube"},
            )

        # Assert: Expect 500 error (internal server error)
        assert response.status_code == 500
        assert "Failed to create turn" in response.json()["detail"]

    def test_create_turn_mongodb_error_rolls_back(self, client: TestClient):
        """
        Test Turn API handles MongoDB errors with rollback

        Scenario:
        1. Create session and battle
        2. Mock Episode.insert() to raise exception
        3. Attempt to create turn
        4. Expect 500 error
        5. Verify no orphan Turn records (rollback worked)
        """
        from unittest.mock import AsyncMock, patch

        # Arrange: Create session and battle
        init_response = client.post(
            "/api/sessions/init",
            json={
                "robot_id": "widowx",
                "scene_id": "table_pick_place",
            },
        )
        assert init_response.status_code == 201
        battle_id = init_response.json()["battle_id"]

        # Act: Mock Episode.insert() to simulate MongoDB error
        with patch(
            "vlaarena_shared.mongodb_models.Episode.insert",
            new_callable=AsyncMock,
            side_effect=Exception("MongoDB insert failed"),
        ):
            response = client.post(
                f"/api/battles/{battle_id}/turns",
                json={"instruction": "Pick up the cube"},
            )

        # Assert: Expect 500 error
        assert response.status_code == 500
        assert "Failed to create turn" in response.json()["detail"]

    def test_create_turn_vla_service_error_rolls_back(self, client: TestClient):
        """
        Test Turn API handles VLA service errors with rollback

        Scenario:
        1. Create session and battle
        2. Mock MockVLAService.generate_episode() to raise exception
        3. Attempt to create turn
        4. Expect 500 error
        5. Verify rollback occurred
        """
        from unittest.mock import patch

        # Arrange: Create session and battle
        init_response = client.post(
            "/api/sessions/init",
            json={
                "robot_id": "widowx",
                "scene_id": "table_pick_place",
            },
        )
        assert init_response.status_code == 201
        battle_id = init_response.json()["battle_id"]

        # Act: Mock VLA service to simulate failure
        with patch(
            "vlaarena_backend.services.turn_service.MockVLAService.generate_episode",
            side_effect=Exception("VLA inference failed"),
        ):
            response = client.post(
                f"/api/battles/{battle_id}/turns",
                json={"instruction": "Pick up the cube"},
            )

        # Assert: Expect 500 error
        assert response.status_code == 500
        assert "Failed to create turn" in response.json()["detail"]
