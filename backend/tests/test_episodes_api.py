"""
Tests for VLA Arena Episode API endpoints
Following TDD workflow: Red → Green → Refactor

Episode API handles retrieving episode data from MongoDB for frontend replay.
"""

from unittest.mock import AsyncMock, patch

import pytest
from fastapi.testclient import TestClient
from pymongo.errors import ConnectionFailure, OperationFailure

from vlaarena_shared.mongodb_models import Episode, State


class TestEpisodeAPI:
    """Test GET /api/episodes/{episode_id} endpoint (VLA Arena MVP)"""

    @pytest.mark.asyncio
    async def test_get_episode_success(self, client: TestClient, mongodb):
        """
        Test successful episode retrieval

        Scenario:
        1. Episode exists in MongoDB
        2. User requests episode by ID
        3. Returns complete episode data with actions and states

        AAA Pattern:
        - Arrange: Create episode in MongoDB
        - Act: GET /api/episodes/{episode_id}
        - Assert: Verify response structure and data
        """
        # Arrange: Create episode in MongoDB
        episode = Episode(
            episode_id="ep_test123",
            session_id="sess_abc",
            battle_id="battle_def",
            turn_id="turn_ghi",
            battle_seq_in_session=1,
            turn_seq=1,
            seq_in_turn=0,
            side="left",
            model_id="openvla-7b",
            actions=[[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]],
            states=[
                State(
                    qpos=[1.0, 2.0, 3.0],
                    qvel=[0.1, 0.2, 0.3],
                    time=0.0,
                )
            ],
            duration_ms=1000,
        )
        await episode.insert()

        # Act
        response = client.get("/api/episodes/ep_test123")

        # Assert
        assert response.status_code == 200
        result = response.json()

        # Check response structure
        assert "episode_id" in result
        assert "actions" in result
        assert "states" in result

        # Check episode_id
        assert result["episode_id"] == "ep_test123"

        # Check actions
        assert len(result["actions"]) == 1
        assert len(result["actions"][0]) == 8
        assert result["actions"][0] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

        # Check states
        assert len(result["states"]) == 1
        state = result["states"][0]
        assert state["qpos"] == [1.0, 2.0, 3.0]
        assert state["qvel"] == [0.1, 0.2, 0.3]
        assert state["time"] == 0.0

    @pytest.mark.asyncio
    async def test_get_episode_not_found(self, client: TestClient, mongodb):
        """
        Test episode retrieval fails when episode doesn't exist

        Scenario:
        1. User requests non-existent episode_id
        2. Returns 404 not found

        AAA Pattern:
        - Arrange: No episode in database
        - Act: GET /api/episodes/{episode_id}
        - Assert: 404 status code
        """
        # Arrange: No episode created (empty database)

        # Act
        response = client.get("/api/episodes/ep_nonexistent")

        # Assert
        assert response.status_code == 404
        detail = response.json()["detail"]
        assert "not found" in detail.lower()

    @pytest.mark.asyncio
    async def test_get_episode_multiple_steps(self, client: TestClient, mongodb):
        """
        Test episode retrieval with multiple steps

        Scenario:
        1. Episode has multiple actions and states (e.g., 10 steps)
        2. User requests episode by ID
        3. Returns all actions and states in order

        AAA Pattern:
        - Arrange: Create episode with 10 steps
        - Act: GET /api/episodes/{episode_id}
        - Assert: Verify all steps returned
        """
        # Arrange: Create episode with 10 steps
        actions = [[float(i)] * 8 for i in range(10)]  # 10 actions
        states = [
            State(
                qpos=[float(i), float(i + 1), float(i + 2)],
                qvel=[float(i) * 0.1, float(i + 1) * 0.1, float(i + 2) * 0.1],
                time=float(i) * 0.1,
            )
            for i in range(10)
        ]

        episode = Episode(
            episode_id="ep_multi_steps",
            session_id="sess_abc",
            battle_id="battle_def",
            turn_id="turn_ghi",
            battle_seq_in_session=1,
            turn_seq=1,
            seq_in_turn=0,
            side="left",
            model_id="openvla-7b",
            actions=actions,
            states=states,
            duration_ms=1000,
        )
        await episode.insert()

        # Act
        response = client.get("/api/episodes/ep_multi_steps")

        # Assert
        assert response.status_code == 200
        result = response.json()

        # Check actions length
        assert len(result["actions"]) == 10

        # Check states length
        assert len(result["states"]) == 10

        # Verify first and last states
        assert result["states"][0]["time"] == 0.0
        assert result["states"][9]["time"] == 0.9

    @pytest.mark.asyncio
    async def test_get_episode_max_steps(self, client: TestClient, mongodb):
        """
        Test episode retrieval with maximum steps (50 steps)

        Scenario:
        1. Episode has 50 steps (max configured in MVP)
        2. User requests episode by ID
        3. Returns all 50 steps

        AAA Pattern:
        - Arrange: Create episode with 50 steps
        - Act: GET /api/episodes/{episode_id}
        - Assert: Verify 50 steps returned
        """
        # Arrange: Create episode with 50 steps
        max_steps = 50
        actions = [[float(i)] * 8 for i in range(max_steps)]
        states = [
            State(
                qpos=[float(i)] * 3,
                qvel=[float(i) * 0.1] * 3,
                time=float(i) * 0.1,
            )
            for i in range(max_steps)
        ]

        episode = Episode(
            episode_id="ep_max_steps",
            session_id="sess_abc",
            battle_id="battle_def",
            turn_id="turn_ghi",
            battle_seq_in_session=1,
            turn_seq=1,
            seq_in_turn=0,
            side="right",
            model_id="octo-base",
            actions=actions,
            states=states,
            duration_ms=5000,
        )
        await episode.insert()

        # Act
        response = client.get("/api/episodes/ep_max_steps")

        # Assert
        assert response.status_code == 200
        result = response.json()

        # Check max steps
        assert len(result["actions"]) == 50
        assert len(result["states"]) == 50

    @pytest.mark.asyncio
    async def test_get_episode_early_termination(self, client: TestClient, mongodb):
        """
        Test episode retrieval with early termination (less than max steps)

        Scenario:
        1. Episode terminated early (e.g., 15 steps out of 50 max)
        2. User requests episode by ID
        3. Returns only actual steps executed (15 steps)

        AAA Pattern:
        - Arrange: Create episode with 15 steps (terminated early)
        - Act: GET /api/episodes/{episode_id}
        - Assert: Verify 15 steps returned
        """
        # Arrange: Create episode with 15 steps (early termination)
        actual_steps = 15
        actions = [[float(i)] * 8 for i in range(actual_steps)]
        states = [
            State(
                qpos=[float(i)] * 3,
                qvel=[float(i) * 0.1] * 3,
                time=float(i) * 0.1,
            )
            for i in range(actual_steps)
        ]

        episode = Episode(
            episode_id="ep_early_term",
            session_id="sess_abc",
            battle_id="battle_def",
            turn_id="turn_ghi",
            battle_seq_in_session=1,
            turn_seq=1,
            seq_in_turn=0,
            side="left",
            model_id="openvla-7b",
            actions=actions,
            states=states,
            duration_ms=1500,
        )
        await episode.insert()

        # Act
        response = client.get("/api/episodes/ep_early_term")

        # Assert
        assert response.status_code == 200
        result = response.json()

        # Check actual steps (not max steps)
        assert len(result["actions"]) == 15
        assert len(result["states"]) == 15

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find_one")
    async def test_get_episode_database_connection_error(
        self, mock_find_one, client: TestClient, mongodb
    ):
        """
        Test episode retrieval fails when MongoDB connection fails

        Scenario:
        1. MongoDB connection is down
        2. User requests episode by ID
        3. Returns 503 Service Unavailable

        AAA Pattern:
        - Arrange: Mock MongoDB connection failure
        - Act: GET /api/episodes/{episode_id}
        - Assert: 503 status code with appropriate error message
        """
        # Arrange: Mock MongoDB connection failure
        mock_find_one.side_effect = ConnectionFailure("Connection timeout")

        # Act
        response = client.get("/api/episodes/ep_test123")

        # Assert
        assert response.status_code == 503
        detail = response.json()["detail"]
        assert "unavailable" in detail.lower()

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find_one")
    async def test_get_episode_database_operation_error(
        self, mock_find_one, client: TestClient, mongodb
    ):
        """
        Test episode retrieval fails when MongoDB operation fails

        Scenario:
        1. MongoDB operation fails (e.g., permission denied)
        2. User requests episode by ID
        3. Returns 503 Service Unavailable

        AAA Pattern:
        - Arrange: Mock MongoDB operation failure
        - Act: GET /api/episodes/{episode_id}
        - Assert: 503 status code
        """
        # Arrange: Mock MongoDB operation failure
        mock_find_one.side_effect = OperationFailure("Operation not permitted")

        # Act
        response = client.get("/api/episodes/ep_test123")

        # Assert
        assert response.status_code == 503
        detail = response.json()["detail"]
        assert "unavailable" in detail.lower()
