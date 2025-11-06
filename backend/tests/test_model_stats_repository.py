"""
Tests for ModelStatsRepository

Following TDD workflow:
- Red: Write failing tests
- Green: Verify implementation passes
- Refactor: Improve code quality
"""

import pytest
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_backend.repositories.model_stats_repository import ModelStatsRepository
from vlaarena_shared.models import ModelStatsByRobot, ModelStatsTotal


@pytest.mark.asyncio
class TestGetLeaderboard:
    """Test get_leaderboard() method"""

    async def test_get_global_leaderboard_success(self, db: AsyncSession):
        """Test global leaderboard (robot_id=None) returns ModelStatsTotal"""
        # Arrange
        repo = ModelStatsRepository(db)

        # Create test data
        stats1 = ModelStatsTotal(
            model_id="model-a",
            elo_score=1600,
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
            vote_count=10,
            win_count=5,
            loss_count=5,
            tie_count=0,
            win_rate=0.5,
            organization="OrgB",
            license="Apache-2.0",
        )
        stats3 = ModelStatsTotal(
            model_id="model-c",
            elo_score=1400,
            vote_count=10,
            win_count=3,
            loss_count=7,
            tie_count=0,
            win_rate=0.3,
            organization="OrgC",
            license="GPL-3.0",
        )
        db.add_all([stats1, stats2, stats3])
        await db.commit()

        # Act
        results = await repo.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert
        assert len(results) == 3
        assert all(isinstance(r, ModelStatsTotal) for r in results)

        # Verify sorted by ELO descending
        assert results[0].model_id == "model-a"
        assert results[0].elo_score == 1600
        assert results[1].model_id == "model-b"
        assert results[1].elo_score == 1500
        assert results[2].model_id == "model-c"
        assert results[2].elo_score == 1400

    async def test_get_robot_specific_leaderboard_success(self, db: AsyncSession):
        """Test robot-specific leaderboard returns ModelStatsByRobot"""
        # Arrange
        repo = ModelStatsRepository(db)

        # Create robot-specific stats
        franka_a = ModelStatsByRobot(
            model_id="model-a",
            robot_id="franka",
            elo_score=1600,
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
            vote_count=10,
            win_count=5,
            loss_count=5,
            tie_count=0,
            win_rate=0.5,
        )
        # Different robot (should not appear)
        ur5_a = ModelStatsByRobot(
            model_id="model-a",
            robot_id="ur5",
            elo_score=1400,
            vote_count=10,
            win_count=3,
            loss_count=7,
            tie_count=0,
            win_rate=0.3,
        )
        db.add_all([franka_a, franka_b, ur5_a])
        await db.commit()

        # Act
        results = await repo.get_leaderboard(min_vote_count=5, robot_id="franka")

        # Assert
        assert len(results) == 2  # Only franka models
        assert all(isinstance(r, ModelStatsByRobot) for r in results)
        assert all(r.robot_id == "franka" for r in results)

        # Verify sorted by ELO descending
        assert results[0].model_id == "model-a"
        assert results[0].elo_score == 1600
        assert results[1].model_id == "model-b"
        assert results[1].elo_score == 1500

    async def test_get_leaderboard_filters_by_min_votes(self, db: AsyncSession):
        """Test min_vote_count filter"""
        # Arrange
        repo = ModelStatsRepository(db)

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
        results = await repo.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert
        assert len(results) == 1
        assert results[0].model_id == "model-a"  # Only model with >= 5 votes

    async def test_get_leaderboard_empty_result(self, db: AsyncSession):
        """Test leaderboard with no qualifying models"""
        # Arrange
        repo = ModelStatsRepository(db)

        # No data in database

        # Act
        results = await repo.get_leaderboard(min_vote_count=5, robot_id=None)

        # Assert
        assert len(results) == 0
        assert results == []

    async def test_get_leaderboard_robot_not_found(self, db: AsyncSession):
        """Test leaderboard for non-existent robot"""
        # Arrange
        repo = ModelStatsRepository(db)

        # Create data for different robot
        stats = ModelStatsByRobot(
            model_id="model-a",
            robot_id="franka",
            elo_score=1600,
            vote_count=10,
            win_count=7,
            loss_count=3,
            tie_count=0,
            win_rate=0.7,
        )
        db.add(stats)
        await db.commit()

        # Act
        results = await repo.get_leaderboard(min_vote_count=5, robot_id="nonexistent")

        # Assert
        assert len(results) == 0


@pytest.mark.asyncio
class TestGetModelStats:
    """Test get_model_stats() methods"""

    async def test_get_model_stats_by_robot_success(self, db: AsyncSession):
        """Test getting robot-specific stats"""
        # Arrange
        repo = ModelStatsRepository(db)

        stats = ModelStatsByRobot(
            model_id="model-a",
            robot_id="franka",
            elo_score=1600,
            vote_count=10,
            win_count=7,
            loss_count=3,
            tie_count=0,
            win_rate=0.7,
        )
        db.add(stats)
        await db.commit()

        # Act
        result = await repo.get_model_stats_by_robot("model-a", "franka")

        # Assert
        assert result is not None
        assert result.model_id == "model-a"
        assert result.robot_id == "franka"
        assert result.elo_score == 1600

    async def test_get_model_stats_by_robot_not_found(self, db: AsyncSession):
        """Test getting non-existent robot-specific stats"""
        # Arrange
        repo = ModelStatsRepository(db)

        # Act
        result = await repo.get_model_stats_by_robot("nonexistent", "franka")

        # Assert
        assert result is None

    async def test_get_model_stats_total_success(self, db: AsyncSession):
        """Test getting global stats"""
        # Arrange
        repo = ModelStatsRepository(db)

        stats = ModelStatsTotal(
            model_id="model-a",
            elo_score=1600,
            vote_count=10,
            win_count=7,
            loss_count=3,
            tie_count=0,
            win_rate=0.7,
            organization="OrgA",
            license="MIT",
        )
        db.add(stats)
        await db.commit()

        # Act
        result = await repo.get_model_stats_total("model-a")

        # Assert
        assert result is not None
        assert result.model_id == "model-a"
        assert result.elo_score == 1600
        assert result.organization == "OrgA"

    async def test_get_model_stats_total_not_found(self, db: AsyncSession):
        """Test getting non-existent global stats"""
        # Arrange
        repo = ModelStatsRepository(db)

        # Act
        result = await repo.get_model_stats_total("nonexistent")

        # Assert
        assert result is None
