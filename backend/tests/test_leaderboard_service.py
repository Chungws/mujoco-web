"""
Tests for LeaderboardService

Following TDD workflow:
- Red: Write failing tests
- Green: Verify implementation passes
- Refactor: Improve code quality
"""

import pytest
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_backend.services.leaderboard_service import LeaderboardService
from vlaarena_shared.models import ModelStatsByRobot, ModelStatsTotal


@pytest.mark.asyncio
class TestGetLeaderboard:
    """Test get_leaderboard() service method"""

    async def test_get_global_leaderboard_success(self, db: AsyncSession):
        """Test global leaderboard returns ranked models"""
        # Arrange
        service = LeaderboardService(db)

        # Create test data
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
        result = await service.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert
        assert result.rankings is not None
        assert len(result.rankings) == 2

        # Verify sorted by ELO descending
        assert result.rankings[0].model_id == "model-a"
        assert result.rankings[0].elo_score == 1600
        assert result.rankings[0].name == "model-a"
        assert result.rankings[0].vote_count == 10
        assert result.rankings[0].win_rate == 0.7

        assert result.rankings[1].model_id == "model-b"
        assert result.rankings[1].elo_score == 1500

    async def test_get_robot_specific_leaderboard_success(self, db: AsyncSession):
        """Test robot-specific leaderboard"""
        # Arrange
        service = LeaderboardService(db)

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
        db.add_all([franka_a, franka_b])
        await db.commit()

        # Act
        result = await service.get_leaderboard(min_vote_count=5, robot_id="franka")

        # Assert
        assert len(result.rankings) == 2
        assert result.rankings[0].model_id == "model-a"
        assert result.rankings[0].elo_score == 1600
        assert result.rankings[1].model_id == "model-b"
        assert result.rankings[1].elo_score == 1500

    async def test_get_leaderboard_empty_result(self, db: AsyncSession):
        """Test leaderboard with no models returns empty list"""
        # Arrange
        service = LeaderboardService(db)

        # No data in database

        # Act
        result = await service.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert
        assert result.rankings == []

    async def test_get_leaderboard_filters_by_min_votes(self, db: AsyncSession):
        """Test min_vote_count filter"""
        # Arrange
        service = LeaderboardService(db)

        stats_high_votes = ModelStatsTotal(
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
        stats_low_votes = ModelStatsTotal(
            model_id="model-b",
            elo_score=1700,  # Higher ELO but low votes
            vote_count=3,  # Below threshold
            win_count=2,
            loss_count=1,
            tie_count=0,
            win_rate=0.67,
            organization="OrgB",
            license="Apache-2.0",
        )
        db.add_all([stats_high_votes, stats_low_votes])
        await db.commit()

        # Act
        result = await service.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert
        assert len(result.rankings) == 1
        assert result.rankings[0].model_id == "model-a"  # Only model with >= 5 votes

    async def test_get_leaderboard_sorted_by_elo_descending(self, db: AsyncSession):
        """Test rankings are sorted by ELO score descending"""
        # Arrange
        service = LeaderboardService(db)

        # Create 5 models with different ELO scores
        for i in range(5):
            stats = ModelStatsTotal(
                model_id=f"model-{i}",
                elo_score=1500 + (i * 50),  # 1500, 1550, 1600, 1650, 1700
                vote_count=10,
                win_count=5,
                loss_count=5,
                tie_count=0,
                win_rate=0.5,
                organization=f"Org{i}",
                license="MIT",
            )
            db.add(stats)
        await db.commit()

        # Act
        result = await service.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert - Sorted by ELO descending
        assert len(result.rankings) == 5
        assert result.rankings[0].elo_score == 1700  # Highest
        assert result.rankings[1].elo_score == 1650
        assert result.rankings[2].elo_score == 1600
        assert result.rankings[3].elo_score == 1550
        assert result.rankings[4].elo_score == 1500  # Lowest
