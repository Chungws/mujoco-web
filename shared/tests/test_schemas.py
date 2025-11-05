"""
Tests for Pydantic schemas (VLA Arena MVP)
Following TDD workflow: Red → Green → Refactor
"""

import pytest
from pydantic import ValidationError
from vlaarena_shared.schemas import (
    EpisodeMetrics,
    EpisodeResponse,
    EpisodeState,
    LeaderboardResponse,
    ModelRanking,
    RevealedModels,
    SessionInitRequest,
    SessionResponse,
    TurnRequest,
    TurnResponse,
    VoteRequest,
    VoteResponse,
)

# ==================== Session Schemas Tests ====================


class TestSessionSchemas:
    """Test Session-related schemas"""

    def test_session_init_request_valid(self):
        """Test SessionInitRequest with valid data"""
        # Arrange & Act
        data = {"robot_id": "widowx", "scene_id": "table"}
        request = SessionInitRequest(**data)

        # Assert
        assert request.robot_id == "widowx"
        assert request.scene_id == "table"

    def test_session_init_request_missing_robot_id(self):
        """Test SessionInitRequest raises error when robot_id missing"""
        # Arrange
        data = {"scene_id": "table"}

        # Act & Assert
        with pytest.raises(ValidationError) as exc_info:
            SessionInitRequest(**data)

        assert "robot_id" in str(exc_info.value)

    def test_session_init_request_missing_scene_id(self):
        """Test SessionInitRequest raises error when scene_id missing"""
        # Arrange
        data = {"robot_id": "widowx"}

        # Act & Assert
        with pytest.raises(ValidationError) as exc_info:
            SessionInitRequest(**data)

        assert "scene_id" in str(exc_info.value)

    def test_session_response_valid(self):
        """Test SessionResponse with valid data"""
        # Arrange & Act
        data = {
            "session_id": "sess_abc123",
            "battle_id": "battle_def456",
            "left_model": "???",
            "right_model": "???",
        }
        response = SessionResponse(**data)

        # Assert
        assert response.session_id == "sess_abc123"
        assert response.battle_id == "battle_def456"
        assert response.left_model == "???"
        assert response.right_model == "???"

    def test_session_response_models_hidden(self):
        """Test SessionResponse always uses hidden model names"""
        # Arrange & Act
        data = {
            "session_id": "sess_abc",
            "battle_id": "battle_def",
            "left_model": "???",
            "right_model": "???",
        }
        response = SessionResponse(**data)

        # Assert
        assert response.left_model == "???"
        assert response.right_model == "???"


# ==================== Turn Schemas Tests ====================


class TestTurnSchemas:
    """Test Turn-related schemas (Battle turns)"""

    def test_turn_request_valid(self):
        """Test TurnRequest with valid instruction"""
        # Arrange & Act
        data = {"instruction": "Pick up the red cube"}
        request = TurnRequest(**data)

        # Assert
        assert request.instruction == "Pick up the red cube"

    def test_turn_request_empty_instruction(self):
        """Test TurnRequest raises error when instruction is empty"""
        # Arrange
        data = {"instruction": ""}

        # Act & Assert
        with pytest.raises(ValidationError) as exc_info:
            TurnRequest(**data)

        assert "instruction" in str(exc_info.value)

    def test_turn_request_long_instruction(self):
        """Test TurnRequest accepts long instructions"""
        # Arrange
        long_instruction = "Pick up the red cube and place it on the blue plate. " * 20
        data = {"instruction": long_instruction}

        # Act
        request = TurnRequest(**data)

        # Assert
        assert request.instruction == long_instruction

    def test_turn_response_valid(self):
        """Test TurnResponse with valid data"""
        # Arrange & Act
        data = {
            "turn_id": "turn_ghi789",
            "left_episode_id": "ep_left_123",
            "right_episode_id": "ep_right_456",
            "status": "completed",
        }
        response = TurnResponse(**data)

        # Assert
        assert response.turn_id == "turn_ghi789"
        assert response.left_episode_id == "ep_left_123"
        assert response.right_episode_id == "ep_right_456"
        assert response.status == "completed"

    def test_turn_response_status_values(self):
        """Test TurnResponse accepts valid status values"""
        # Arrange & Act & Assert
        valid_statuses = ["completed", "failed", "running"]
        for status in valid_statuses:
            data = {
                "turn_id": "turn_123",
                "left_episode_id": "ep_left",
                "right_episode_id": "ep_right",
                "status": status,
            }
            response = TurnResponse(**data)
            assert response.status == status


# ==================== Episode Schemas Tests ====================


class TestEpisodeSchemas:
    """Test Episode-related schemas"""

    def test_episode_state_valid(self):
        """Test EpisodeState with valid data"""
        # Arrange & Act
        data = {
            "qpos": [0.1, 0.2, 0.3],
            "qvel": [0.01, 0.02, 0.03],
            "time": 0.0,
        }
        state = EpisodeState(**data)

        # Assert
        assert state.qpos == [0.1, 0.2, 0.3]
        assert state.qvel == [0.01, 0.02, 0.03]
        assert state.time == 0.0

    def test_episode_metrics_valid(self):
        """Test EpisodeMetrics with valid data"""
        # Arrange & Act
        data = {
            "success": True,
            "total_steps": 35,
            "max_steps": 50,
            "final_distance_to_goal": 0.05,
        }
        metrics = EpisodeMetrics(**data)

        # Assert
        assert metrics.success is True
        assert metrics.total_steps == 35
        assert metrics.max_steps == 50
        assert metrics.final_distance_to_goal == 0.05

    def test_episode_response_valid(self):
        """Test EpisodeResponse with valid data"""
        # Arrange
        data = {
            "episode_id": "ep_left_123",
            "actions": [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]],
            "states": [
                {"qpos": [0.1, 0.2], "qvel": [0.01, 0.02], "time": 0.0},
                {"qpos": [0.2, 0.3], "qvel": [0.02, 0.03], "time": 0.1},
            ],
            "metrics": {
                "success": True,
                "total_steps": 2,
                "max_steps": 50,
                "final_distance_to_goal": 0.05,
            },
        }

        # Act
        response = EpisodeResponse(**data)

        # Assert
        assert response.episode_id == "ep_left_123"
        assert len(response.actions) == 1
        assert len(response.states) == 2
        assert response.metrics.success is True
        assert response.metrics.total_steps == 2

    def test_episode_response_empty_actions(self):
        """Test EpisodeResponse with empty actions list"""
        # Arrange
        data = {
            "episode_id": "ep_123",
            "actions": [],
            "states": [],
            "metrics": {
                "success": False,
                "total_steps": 0,
                "max_steps": 50,
                "final_distance_to_goal": 1.0,
            },
        }

        # Act
        response = EpisodeResponse(**data)

        # Assert
        assert response.episode_id == "ep_123"
        assert len(response.actions) == 0
        assert len(response.states) == 0


# ==================== Vote Schemas Tests ====================


class TestVoteSchemas:
    """Test Vote-related schemas"""

    def test_vote_request_valid_left_better(self):
        """Test VoteRequest with 'left_better' vote"""
        # Arrange & Act
        data = {"battle_id": "battle_123", "vote": "left_better"}
        request = VoteRequest(**data)

        # Assert
        assert request.battle_id == "battle_123"
        assert request.vote == "left_better"

    def test_vote_request_valid_values(self):
        """Test VoteRequest accepts all valid vote values"""
        # Arrange
        valid_votes = ["left_better", "right_better", "tie", "both_bad"]

        # Act & Assert
        for vote in valid_votes:
            data = {"battle_id": "battle_123", "vote": vote}
            request = VoteRequest(**data)
            assert request.vote == vote

    def test_vote_request_invalid_vote(self):
        """Test VoteRequest raises error with invalid vote"""
        # Arrange
        data = {"battle_id": "battle_123", "vote": "invalid_vote"}

        # Act & Assert
        with pytest.raises(ValidationError) as exc_info:
            VoteRequest(**data)

        assert "vote" in str(exc_info.value)

    def test_vote_response_valid(self):
        """Test VoteResponse with valid data"""
        # Arrange & Act
        data = {
            "vote_id": "vote_jkl012",
            "revealed_models": {"left": "openvla-7b", "right": "octo-base"},
        }
        response = VoteResponse(**data)

        # Assert
        assert response.vote_id == "vote_jkl012"
        assert response.revealed_models.left == "openvla-7b"
        assert response.revealed_models.right == "octo-base"

    def test_revealed_models_valid(self):
        """Test RevealedModels schema"""
        # Arrange & Act
        data = {"left": "openvla-7b", "right": "octo-base"}
        models = RevealedModels(**data)

        # Assert
        assert models.left == "openvla-7b"
        assert models.right == "octo-base"


# ==================== Leaderboard Schemas Tests ====================


class TestLeaderboardSchemas:
    """Test Leaderboard-related schemas"""

    def test_model_ranking_valid(self):
        """Test ModelRanking with valid data"""
        # Arrange & Act
        data = {
            "model_id": "openvla-7b",
            "name": "OpenVLA 7B",
            "elo_score": 1650,
            "vote_count": 42,
            "win_rate": 0.62,
        }
        ranking = ModelRanking(**data)

        # Assert
        assert ranking.model_id == "openvla-7b"
        assert ranking.name == "OpenVLA 7B"
        assert ranking.elo_score == 1650
        assert ranking.vote_count == 42
        assert ranking.win_rate == 0.62

    def test_leaderboard_response_valid(self):
        """Test LeaderboardResponse with valid data"""
        # Arrange
        data = {
            "rankings": [
                {
                    "model_id": "openvla-7b",
                    "name": "OpenVLA 7B",
                    "elo_score": 1650,
                    "vote_count": 42,
                    "win_rate": 0.62,
                },
                {
                    "model_id": "octo-base",
                    "name": "Octo Base",
                    "elo_score": 1550,
                    "vote_count": 38,
                    "win_rate": 0.51,
                },
            ]
        }

        # Act
        response = LeaderboardResponse(**data)

        # Assert
        assert len(response.rankings) == 2
        assert response.rankings[0].model_id == "openvla-7b"
        assert response.rankings[1].model_id == "octo-base"

    def test_leaderboard_response_empty_rankings(self):
        """Test LeaderboardResponse with empty rankings"""
        # Arrange
        data = {"rankings": []}

        # Act
        response = LeaderboardResponse(**data)

        # Assert
        assert len(response.rankings) == 0

    def test_model_ranking_win_rate_validation(self):
        """Test ModelRanking win_rate is between 0 and 1"""
        # Arrange & Act
        data = {
            "model_id": "test-model",
            "name": "Test Model",
            "elo_score": 1500,
            "vote_count": 10,
            "win_rate": 0.5,
        }
        ranking = ModelRanking(**data)

        # Assert
        assert 0.0 <= ranking.win_rate <= 1.0
