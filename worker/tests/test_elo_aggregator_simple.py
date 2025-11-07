"""
Simple test for robot-specific + global ELO aggregation

Tests that Worker correctly updates both ModelStatsByRobot and ModelStatsTotal tables
"""

import pytest
from sqlmodel import select
from vlaarena_shared.models import ModelStatsByRobot, ModelStatsTotal, Vote


@pytest.mark.asyncio
class TestELOAggregatorDualELO:
    """Test robot-specific + global ELO aggregation"""

    async def test_robot_specific_and_global_elo_update(self, test_db_session):
        """
        Test that a single vote updates both robot-specific and global ELO

        When a vote is processed:
        1. ModelStatsByRobot (model_id + robot_id) should be created/updated
        2. ModelStatsTotal (model_id) should be created/updated
        """
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create a pending vote
        vote = Vote(
            vote_id="vote-robot-1",
            battle_id="battle-robot-1",
            session_id="session-1",
            robot_id="franka",
            scene_id="table",
            vote="left_better",
            left_model_id="model-a",
            right_model_id="model-b",
            processing_status="pending",
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute: Run aggregator
        model_configs = {
            "model-a": {"organization": "OrgA", "license": "MIT"},
            "model-b": {"organization": "OrgB", "license": "Apache-2.0"},
        }
        aggregator = ELOAggregator(test_db_session, model_configs=model_configs)
        votes_processed = await aggregator.process_pending_votes()

        assert votes_processed == 1

        # Verify robot-specific stats exist
        result_robot_a = await test_db_session.execute(
            select(ModelStatsByRobot).where(
                ModelStatsByRobot.model_id == "model-a",
                ModelStatsByRobot.robot_id == "franka",
            )
        )
        stats_robot_a = result_robot_a.scalar_one()
        assert stats_robot_a.elo_score > 1500  # Winner
        assert stats_robot_a.vote_count == 1
        assert stats_robot_a.win_count == 1

        result_robot_b = await test_db_session.execute(
            select(ModelStatsByRobot).where(
                ModelStatsByRobot.model_id == "model-b",
                ModelStatsByRobot.robot_id == "franka",
            )
        )
        stats_robot_b = result_robot_b.scalar_one()
        assert stats_robot_b.elo_score < 1500  # Loser
        assert stats_robot_b.vote_count == 1
        assert stats_robot_b.loss_count == 1

        # Verify global stats exist
        result_total_a = await test_db_session.execute(
            select(ModelStatsTotal).where(ModelStatsTotal.model_id == "model-a")
        )
        stats_total_a = result_total_a.scalar_one()
        assert stats_total_a.elo_score > 1500
        assert stats_total_a.vote_count == 1
        assert stats_total_a.win_count == 1
        assert stats_total_a.organization == "OrgA"
        assert stats_total_a.license == "MIT"

        result_total_b = await test_db_session.execute(
            select(ModelStatsTotal).where(ModelStatsTotal.model_id == "model-b")
        )
        stats_total_b = result_total_b.scalar_one()
        assert stats_total_b.elo_score < 1500
        assert stats_total_b.vote_count == 1
        assert stats_total_b.loss_count == 1
        assert stats_total_b.organization == "OrgB"
        assert stats_total_b.license == "Apache-2.0"

    async def test_multiple_robots_independent_elo(self, test_db_session):
        """
        Test that different robots maintain independent ELO rankings

        Model A might be good on Franka but bad on UR5
        """
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Vote 1: Franka robot, model-a wins
        vote1 = Vote(
            vote_id="vote-1",
            battle_id="battle-1",
            session_id="session-1",
            robot_id="franka",
            scene_id="table",
            vote="left_better",
            left_model_id="model-a",
            right_model_id="model-b",
            processing_status="pending",
        )
        # Vote 2: UR5 robot, model-b wins (same models, different robot)
        vote2 = Vote(
            vote_id="vote-2",
            battle_id="battle-2",
            session_id="session-2",
            robot_id="ur5",
            scene_id="table",
            vote="right_better",
            left_model_id="model-a",
            right_model_id="model-b",
            processing_status="pending",
        )
        test_db_session.add(vote1)
        test_db_session.add(vote2)
        await test_db_session.commit()

        # Execute
        model_configs = {
            "model-a": {"organization": "OrgA", "license": "MIT"},
            "model-b": {"organization": "OrgB", "license": "Apache-2.0"},
        }
        aggregator = ELOAggregator(test_db_session, model_configs=model_configs)
        votes_processed = await aggregator.process_pending_votes()

        assert votes_processed == 2

        # Verify Franka-specific: model-a wins
        result = await test_db_session.execute(
            select(ModelStatsByRobot).where(
                ModelStatsByRobot.model_id == "model-a",
                ModelStatsByRobot.robot_id == "franka",
            )
        )
        franka_a = result.scalar_one()
        assert franka_a.elo_score > 1500
        assert franka_a.win_count == 1

        # Verify UR5-specific: model-a loses
        result = await test_db_session.execute(
            select(ModelStatsByRobot).where(
                ModelStatsByRobot.model_id == "model-a",
                ModelStatsByRobot.robot_id == "ur5",
            )
        )
        ur5_a = result.scalar_one()
        assert ur5_a.elo_score < 1500
        assert ur5_a.loss_count == 1

        # Verify global: model-a has 1 win, 1 loss
        result = await test_db_session.execute(
            select(ModelStatsTotal).where(ModelStatsTotal.model_id == "model-a")
        )
        total_a = result.scalar_one()
        assert total_a.vote_count == 2
        assert total_a.win_count == 1
        assert total_a.loss_count == 1
