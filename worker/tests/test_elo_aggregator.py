"""
Tests for ELO aggregation worker

Tests vote aggregation workflow:
- Read pending votes from PostgreSQL
- Calculate ELO ratings for models
- Update model_stats table
- Mark votes as processed
- Handle errors and update worker_status
"""

from datetime import UTC, datetime

import pytest
from sqlmodel import select

from vlaarena_shared.models import ModelStats, Vote


@pytest.mark.asyncio
class TestELOAggregator:
    """Test ELO aggregation workflow"""

    async def test_process_single_vote(self, test_db_session):
        """Test processing a single vote and updating model stats"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create model stats for both models (initial ELO = 1500)
        model_a_stats = ModelStats(
            model_id="gpt-4",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="OpenAI",
            license="proprietary",
        )
        model_b_stats = ModelStats(
            model_id="claude-3",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="Anthropic",
            license="proprietary",
        )
        test_db_session.add(model_a_stats)
        test_db_session.add(model_b_stats)
        await test_db_session.commit()

        # Setup: Create a pending vote (left model wins)
        vote = Vote(
            vote_id="vote-1",
            battle_id="battle-1",
            session_id="session-1",
            vote="left_better",
            left_model_id="gpt-4",
            right_model_id="claude-3",
            processing_status="pending",
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute: Run aggregator
        aggregator = ELOAggregator(test_db_session)
        votes_processed = await aggregator.process_pending_votes()

        # Verify: 1 vote processed
        assert votes_processed == 1

        # Verify: Vote marked as processed
        await test_db_session.refresh(vote)
        assert vote.processing_status == "processed"
        assert vote.processed_at is not None

        # Verify: Model A (gpt-4) ELO increased (won)
        await test_db_session.refresh(model_a_stats)
        assert model_a_stats.elo_score > 1500
        assert model_a_stats.vote_count == 1
        assert model_a_stats.win_count == 1
        assert model_a_stats.loss_count == 0
        assert model_a_stats.win_rate == 1.0

        # Verify: Model B (claude-3) ELO decreased (lost)
        await test_db_session.refresh(model_b_stats)
        assert model_b_stats.elo_score < 1500
        assert model_b_stats.vote_count == 1
        assert model_b_stats.win_count == 0
        assert model_b_stats.loss_count == 1
        assert model_b_stats.win_rate == 0.0

        # Verify: Confidence intervals updated (1 vote = 784.0)
        assert model_a_stats.elo_ci == 784.0
        assert model_b_stats.elo_ci == 784.0

    async def test_process_tie_vote(self, test_db_session):
        """Test processing a tie vote"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create model stats
        model_a_stats = ModelStats(
            model_id="gpt-4",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="OpenAI",
            license="proprietary",
        )
        model_b_stats = ModelStats(
            model_id="claude-3",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="Anthropic",
            license="proprietary",
        )
        test_db_session.add(model_a_stats)
        test_db_session.add(model_b_stats)
        await test_db_session.commit()

        # Setup: Create a tie vote
        vote = Vote(
            vote_id="vote-2",
            battle_id="battle-2",
            session_id="session-1",
            vote="tie",
            left_model_id="gpt-4",
            right_model_id="claude-3",
            processing_status="pending",
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute
        aggregator = ELOAggregator(test_db_session)
        votes_processed = await aggregator.process_pending_votes()

        # Verify
        assert votes_processed == 1

        # Verify: Both models ELO unchanged (equal ratings, tie)
        await test_db_session.refresh(model_a_stats)
        await test_db_session.refresh(model_b_stats)
        assert model_a_stats.elo_score == 1500
        assert model_b_stats.elo_score == 1500

        # Verify: Tie counts updated
        assert model_a_stats.tie_count == 1
        assert model_b_stats.tie_count == 1
        assert model_a_stats.vote_count == 1
        assert model_b_stats.vote_count == 1

    async def test_process_both_bad_vote(self, test_db_session):
        """Test processing a both_bad vote"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create model stats
        model_a_stats = ModelStats(
            model_id="gpt-4",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="OpenAI",
            license="proprietary",
        )
        model_b_stats = ModelStats(
            model_id="claude-3",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="Anthropic",
            license="proprietary",
        )
        test_db_session.add(model_a_stats)
        test_db_session.add(model_b_stats)
        await test_db_session.commit()

        # Setup: Create a both_bad vote
        vote = Vote(
            vote_id="vote-3",
            battle_id="battle-3",
            session_id="session-1",
            vote="both_bad",
            left_model_id="gpt-4",
            right_model_id="claude-3",
            processing_status="pending",
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute
        aggregator = ELOAggregator(test_db_session)
        votes_processed = await aggregator.process_pending_votes()

        # Verify
        assert votes_processed == 1

        # Verify: Both models ELO decreased (both_bad = 0.25 score)
        await test_db_session.refresh(model_a_stats)
        await test_db_session.refresh(model_b_stats)
        assert model_a_stats.elo_score < 1500
        assert model_b_stats.elo_score < 1500

    async def test_process_multiple_votes(self, test_db_session):
        """Test processing multiple votes in batch"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create model stats
        model_a_stats = ModelStats(
            model_id="gpt-4",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="OpenAI",
            license="proprietary",
        )
        model_b_stats = ModelStats(
            model_id="claude-3",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="Anthropic",
            license="proprietary",
        )
        test_db_session.add(model_a_stats)
        test_db_session.add(model_b_stats)
        await test_db_session.commit()

        # Setup: Create 3 pending votes
        votes = [
            Vote(
                vote_id=f"vote-{i}",
                battle_id=f"battle-{i}",
                session_id="session-1",
                vote="left_better" if i % 2 == 0 else "right_better",
                left_model_id="gpt-4",
                right_model_id="claude-3",
                processing_status="pending",
            )
            for i in range(3)
        ]
        for vote in votes:
            test_db_session.add(vote)
        await test_db_session.commit()

        # Execute
        aggregator = ELOAggregator(test_db_session)
        votes_processed = await aggregator.process_pending_votes()

        # Verify: 3 votes processed
        assert votes_processed == 3

        # Verify: All votes marked as processed
        for vote in votes:
            await test_db_session.refresh(vote)
            assert vote.processing_status == "processed"

        # Verify: Model stats updated (gpt-4 won 2 times, claude-3 won 1 time)
        await test_db_session.refresh(model_a_stats)
        await test_db_session.refresh(model_b_stats)
        assert model_a_stats.vote_count == 3
        assert model_b_stats.vote_count == 3
        assert model_a_stats.win_count == 2
        assert model_b_stats.win_count == 1

    async def test_skip_already_processed_votes(self, test_db_session):
        """Test that already processed votes are skipped"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create model stats
        model_stats = ModelStats(
            model_id="gpt-4",
            elo_score=1500,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization="OpenAI",
            license="proprietary",
        )
        test_db_session.add(model_stats)
        await test_db_session.commit()

        # Setup: Create an already processed vote
        vote = Vote(
            vote_id="vote-1",
            battle_id="battle-1",
            session_id="session-1",
            vote="left_better",
            left_model_id="gpt-4",
            right_model_id="claude-3",
            processing_status="processed",  # Already processed
            processed_at=datetime.now(UTC),
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute
        aggregator = ELOAggregator(test_db_session)
        votes_processed = await aggregator.process_pending_votes()

        # Verify: 0 votes processed (already processed)
        assert votes_processed == 0

    async def test_create_model_stats_if_not_exists(self, test_db_session):
        """Test that model stats are created if they don't exist"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create a vote without existing model stats
        vote = Vote(
            vote_id="vote-1",
            battle_id="battle-1",
            session_id="session-1",
            vote="left_better",
            left_model_id="new-model-1",
            right_model_id="new-model-2",
            processing_status="pending",
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute
        aggregator = ELOAggregator(test_db_session)
        votes_processed = await aggregator.process_pending_votes()

        # Verify: Vote processed
        assert votes_processed == 1

        # Verify: Model stats created for both models
        result = await test_db_session.execute(
            select(ModelStats).where(ModelStats.model_id == "new-model-1")
        )
        model_a_stats = result.scalar_one_or_none()
        assert model_a_stats is not None
        assert model_a_stats.elo_score > 1500  # Won

        result = await test_db_session.execute(
            select(ModelStats).where(ModelStats.model_id == "new-model-2")
        )
        model_b_stats = result.scalar_one_or_none()
        assert model_b_stats is not None
        assert model_b_stats.elo_score < 1500  # Lost

    async def test_error_handling(self, test_db_session):
        """Test error handling when vote processing fails"""
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        # Setup: Create a vote with invalid vote type
        vote = Vote(
            vote_id="vote-error",
            battle_id="battle-error",
            session_id="session-1",
            vote="invalid_vote_type",  # Invalid vote type
            left_model_id="gpt-4",
            right_model_id="claude-3",
            processing_status="pending",
        )
        test_db_session.add(vote)
        await test_db_session.commit()

        # Execute (should handle error gracefully)
        aggregator = ELOAggregator(test_db_session)
        await aggregator.process_pending_votes()

        # Verify: Vote marked as failed
        await test_db_session.refresh(vote)
        assert vote.processing_status == "failed"
        assert vote.error_message is not None
        assert "invalid" in vote.error_message.lower()
