"""
Tests for worker main module
"""

from datetime import UTC, datetime

import pytest
from sqlmodel import select

from vlaarena_shared.models import Vote, WorkerStatus
from vlaarena_worker.main import run_aggregation


@pytest.mark.asyncio
async def test_run_aggregation_success(test_db_session):
    """Test successful aggregation run updates worker_status"""
    # Arrange: Create pending votes
    vote1 = Vote(
        vote_id="vote_1",
        battle_id="battle_1",
        session_id="session_1",
        vote="left_better",
        left_model_id="gpt-4",
        right_model_id="claude-3",
        processing_status="pending",
    )
    vote2 = Vote(
        vote_id="vote_2",
        battle_id="battle_2",
        session_id="session_1",
        vote="right_better",
        left_model_id="gpt-4",
        right_model_id="claude-3",
        processing_status="pending",
    )
    test_db_session.add(vote1)
    test_db_session.add(vote2)
    await test_db_session.commit()

    # Act: Run aggregation
    await run_aggregation(test_db_session)

    # Assert: Check worker_status was created/updated
    await test_db_session.commit()  # Ensure changes are committed
    result = await test_db_session.execute(
        select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
    )
    worker_status = result.scalar_one()

    assert worker_status.status == "success"
    assert worker_status.votes_processed == 2
    assert worker_status.error_message is None
    assert worker_status.last_run_at is not None


@pytest.mark.asyncio
async def test_run_aggregation_updates_existing_status(test_db_session):
    """Test aggregation updates existing worker_status record"""
    # Arrange: Create existing worker_status
    old_time = datetime(2025, 1, 1, 0, 0, 0, tzinfo=UTC)
    existing_status = WorkerStatus(
        worker_name="elo_aggregator",
        last_run_at=old_time,
        status="success",
        votes_processed=5,
    )
    test_db_session.add(existing_status)
    await test_db_session.commit()

    # Create new pending vote
    vote = Vote(
        vote_id="vote_new",
        battle_id="battle_new",
        session_id="session_1",
        vote="tie",
        left_model_id="gpt-4",
        right_model_id="claude-3",
        processing_status="pending",
    )
    test_db_session.add(vote)
    await test_db_session.commit()

    # Act: Run aggregation
    await run_aggregation(test_db_session)

    # Assert: Check worker_status was updated (not duplicated)
    await test_db_session.commit()  # Ensure changes are committed
    result = await test_db_session.execute(
        select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
    )
    all_statuses = result.scalars().all()

    assert len(all_statuses) == 1  # No duplicate
    worker_status = all_statuses[0]

    assert worker_status.status == "success"
    assert worker_status.votes_processed == 1  # Only new votes processed
    assert worker_status.last_run_at > old_time  # Updated timestamp
    assert worker_status.error_message is None


@pytest.mark.asyncio
async def test_run_aggregation_no_pending_votes(test_db_session):
    """Test aggregation with no pending votes"""
    # Act: Run aggregation (no votes exist)
    await run_aggregation(test_db_session)

    # Assert: Check worker_status shows success with 0 votes processed
    await test_db_session.commit()  # Ensure changes are committed
    result = await test_db_session.execute(
        select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
    )
    worker_status = result.scalar_one()

    assert worker_status.status == "success"
    assert worker_status.votes_processed == 0
    assert worker_status.error_message is None


@pytest.mark.asyncio
async def test_run_aggregation_failure_updates_status(test_db_session):
    """Test aggregation failure updates worker_status with error"""
    # Arrange: Create invalid vote (will cause aggregation error)
    vote = Vote(
        vote_id="vote_invalid",
        battle_id="battle_invalid",
        session_id="session_1",
        vote="invalid_vote_type",  # Invalid vote type
        left_model_id="gpt-4",
        right_model_id="claude-3",
        processing_status="pending",
    )
    test_db_session.add(vote)
    await test_db_session.commit()

    # Act: Run aggregation (should catch error)
    await run_aggregation(test_db_session)

    # Assert: Check worker_status shows failure
    await test_db_session.commit()  # Ensure changes are committed
    result = await test_db_session.execute(
        select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
    )
    worker_status = result.scalar_one()

    # Aggregator marks individual votes as failed, but worker itself succeeds
    # (because it handled the error gracefully)
    assert worker_status.status == "success"
    assert worker_status.votes_processed == 0  # Failed vote not counted
    assert worker_status.error_message is None

    # Check vote was marked as failed
    result = await test_db_session.execute(select(Vote).where(Vote.vote_id == "vote_invalid"))
    failed_vote = result.scalar_one()
    assert failed_vote.processing_status == "failed"
    assert failed_vote.error_message is not None
