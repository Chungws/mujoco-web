"""
Tests for Leaderboard API endpoints

Following TDD workflow:
- Red: Write failing tests
- Green: Verify implementation passes
- Refactor: Improve code quality
"""

import pytest
from fastapi.testclient import TestClient
from vlaarena_shared.models import ModelStatsByRobot, ModelStatsTotal


@pytest.mark.asyncio
class TestLeaderboardAPI:
    """Test GET /leaderboard endpoint"""

    async def test_get_global_leaderboard_success(self, client: TestClient, db):
        """Test global leaderboard endpoint returns 200 with rankings"""
        # Arrange - Create test data
        stats1 = ModelStatsTotal(
            model_id="model-a",
            elo_score=1600,
            elo_ci=50.0,
            vote_count=10,
            win_count=7,
            loss_count=3,
            tie_count=0,
            win_rate=0.7,
            organization="OrgA",
            license="MIT",
        )
        stats2 = ModelStatsTotal(
            model_id="model-b",
            elo_score=1500,
            elo_ci=60.0,
            vote_count=8,
            win_count=4,
            loss_count=4,
            tie_count=0,
            win_rate=0.5,
            organization="OrgB",
            license="Apache-2.0",
        )
        db.add_all([stats1, stats2])
        await db.commit()

        # Act
        response = client.get("/api/leaderboard")

        # Assert
        assert response.status_code == 200

        data = response.json()
        assert "rankings" in data
        assert len(data["rankings"]) == 2

        # Verify sorted by ELO
        assert data["rankings"][0]["model_id"] == "model-a"
        assert data["rankings"][0]["elo_score"] == 1600
        assert data["rankings"][0]["name"] == "model-a"
        assert data["rankings"][0]["vote_count"] == 10
        assert data["rankings"][0]["win_rate"] == 0.7

        assert data["rankings"][1]["model_id"] == "model-b"
        assert data["rankings"][1]["elo_score"] == 1500

    async def test_get_robot_specific_leaderboard_success(self, client: TestClient, db):
        """Test robot-specific leaderboard with robot_id param"""
        # Arrange
        franka_a = ModelStatsByRobot(
            model_id="model-a",
            robot_id="franka",
            elo_score=1600,
            elo_ci=50.0,
            vote_count=10,
            win_count=7,
            loss_count=3,
            tie_count=0,
            win_rate=0.7,
        )
        franka_b = ModelStatsByRobot(
            model_id="model-b",
            robot_id="franka",
            elo_score=1500,
            elo_ci=60.0,
            vote_count=8,
            win_count=4,
            loss_count=4,
            tie_count=0,
            win_rate=0.5,
        )
        # Different robot (should not appear)
        ur5_a = ModelStatsByRobot(
            model_id="model-a",
            robot_id="ur5",
            elo_score=1400,
            elo_ci=70.0,
            vote_count=10,
            win_count=3,
            loss_count=7,
            tie_count=0,
            win_rate=0.3,
        )
        db.add_all([franka_a, franka_b, ur5_a])
        await db.commit()

        # Act
        response = client.get("/api/leaderboard?robot_id=franka")

        # Assert
        assert response.status_code == 200

        data = response.json()
        assert len(data["rankings"]) == 2  # Only franka models

        assert data["rankings"][0]["model_id"] == "model-a"
        assert data["rankings"][0]["elo_score"] == 1600
        assert data["rankings"][1]["model_id"] == "model-b"
        assert data["rankings"][1]["elo_score"] == 1500

    async def test_get_leaderboard_empty_result(self, client: TestClient, db):
        """Test leaderboard with no data returns empty array"""
        # Arrange - No data

        # Act
        response = client.get("/api/leaderboard")

        # Assert
        assert response.status_code == 200

        data = response.json()
        assert data["rankings"] == []

    async def test_get_leaderboard_filters_low_vote_counts(self, client: TestClient, db):
        """Test models with few votes are filtered out"""
        # Arrange
        stats_high = ModelStatsTotal(
            model_id="model-a",
            elo_score=1600,
            vote_count=10,  # Above threshold
            win_count=7,
            loss_count=3,
            tie_count=0,
            win_rate=0.7,
            organization="OrgA",
            license="MIT",
        )
        stats_low = ModelStatsTotal(
            model_id="model-b",
            elo_score=1700,  # Higher ELO
            vote_count=3,  # Below threshold (5)
            win_count=2,
            loss_count=1,
            tie_count=0,
            win_rate=0.67,
            organization="OrgB",
            license="Apache-2.0",
        )
        db.add_all([stats_high, stats_low])
        await db.commit()

        # Act
        response = client.get("/api/leaderboard")

        # Assert
        assert response.status_code == 200

        data = response.json()
        assert len(data["rankings"]) == 1
        assert data["rankings"][0]["model_id"] == "model-a"  # Only model with >= 5 votes


@pytest.mark.asyncio
class TestLeaderboardAPIErrors:
    """Test error handling for leaderboard API"""

    async def test_get_leaderboard_handles_db_error(self, client: TestClient, monkeypatch):
        """Test API handles database errors gracefully"""
        # Arrange - Mock service to raise exception
        from vlaarena_backend.services.leaderboard_service import LeaderboardService

        async def mock_error(*args, **kwargs):
            raise Exception("Database connection failed")

        monkeypatch.setattr(LeaderboardService, "get_leaderboard", mock_error)

        # Act
        response = client.get("/api/leaderboard")

        # Assert
        assert response.status_code == 500

        data = response.json()
        assert "detail" in data
        assert "Failed to get leaderboard" in data["detail"]
