"""
Tests for Worker main.py error handling

Following TDD workflow:
- Red: Write failing tests for error scenarios
- Green: Add error handling to pass tests
- Refactor: Improve code quality
"""

from unittest.mock import AsyncMock, MagicMock

import pytest
from sqlmodel import select

from vlaarena_shared.models import WorkerStatus
from vlaarena_worker.main import (
    _update_worker_status,
    load_model_configs,
    run_aggregation,
)


@pytest.mark.asyncio
class TestLoadModelConfigs:
    """Test load_model_configs() error handling"""

    def test_load_config_file_not_found(self, monkeypatch):
        """Test graceful handling when config file doesn't exist"""
        # Arrange
        monkeypatch.setenv("MODELS_CONFIG_PATH", "/nonexistent/path/models.yaml")

        # Act
        result = load_model_configs()

        # Assert
        assert result == {}  # Should return empty dict, not crash

    def test_load_config_invalid_yaml(self, tmp_path):
        """Test handling of malformed YAML file"""
        # Arrange - Create invalid YAML file
        config_file = tmp_path / "invalid.yaml"
        config_file.write_text("invalid: yaml: content: {{")

        # Act
        result = load_model_configs()

        # Assert
        assert result == {}  # Should return empty dict on parse error

    def test_load_config_missing_models_key(self, tmp_path, monkeypatch):
        """Test handling when config is valid YAML but missing 'models' key"""
        # Arrange - Create valid YAML without 'models' key
        config_file = tmp_path / "config.yaml"
        config_file.write_text("other_key: value\n")
        monkeypatch.setenv("MODELS_CONFIG_PATH", str(config_file))

        # Act
        result = load_model_configs()

        # Assert
        assert result == {}


@pytest.mark.asyncio
class TestRunAggregationErrors:
    """Test run_aggregation() error handling"""

    async def test_db_connection_failure_handled(self, monkeypatch):
        """Test graceful handling of database connection failure"""
        # Arrange - Mock database to raise connection error
        mock_session_maker = AsyncMock()
        mock_session = MagicMock()
        mock_session.__aenter__.side_effect = Exception("Database connection failed")
        mock_session_maker.return_value = mock_session

        monkeypatch.setattr("vlaarena_worker.main.async_session_maker", mock_session_maker)

        # Act & Assert - Should not crash
        with pytest.raises(Exception) as exc_info:
            await run_aggregation()

        assert "Database connection failed" in str(exc_info.value)

    async def test_aggregation_failure_updates_worker_status(self, test_db_session, monkeypatch):
        """Test that aggregation failure is recorded in worker_status"""
        # Arrange - Mock aggregator to fail
        from vlaarena_worker.aggregators.elo_aggregator import ELOAggregator

        async def mock_process_error(*args, **kwargs):
            raise Exception("Aggregation processing failed")

        monkeypatch.setattr(ELOAggregator, "process_pending_votes", mock_process_error)

        # Act - Should catch error and update worker_status
        with pytest.raises(Exception):
            await run_aggregation(test_db_session)

        # Assert - Check worker_status was updated with error
        await test_db_session.commit()
        result = await test_db_session.execute(
            select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
        )
        worker_status = result.scalar_one_or_none()

        assert worker_status is not None
        assert worker_status.status == "failed"
        assert "Aggregation processing failed" in worker_status.error_message


@pytest.mark.asyncio
class TestUpdateWorkerStatusErrors:
    """Test _update_worker_status() error handling"""

    async def test_update_status_db_commit_failure(self, test_db_session, monkeypatch):
        """Test handling when database commit fails"""
        # Arrange - Mock commit to fail
        original_commit = test_db_session.commit

        async def mock_commit_error():
            raise Exception("Database commit failed")

        monkeypatch.setattr(test_db_session, "commit", mock_commit_error)

        # Act & Assert - Should propagate error
        with pytest.raises(Exception) as exc_info:
            await _update_worker_status(
                test_db_session,
                votes_processed=5,
                status="success",
                error_message=None,
            )

        assert "Database commit failed" in str(exc_info.value)

        # Restore original commit
        monkeypatch.setattr(test_db_session, "commit", original_commit)

    async def test_update_status_creates_new_record_on_first_run(self, test_db_session):
        """Test that worker_status is created on first run"""
        # Arrange - No existing worker_status

        # Act
        await _update_worker_status(
            test_db_session,
            votes_processed=10,
            status="success",
            error_message=None,
        )

        # Assert
        await test_db_session.commit()
        result = await test_db_session.execute(
            select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
        )
        worker_status = result.scalar_one()

        assert worker_status.votes_processed == 10
        assert worker_status.status == "success"
        assert worker_status.error_message is None

    async def test_update_status_updates_existing_record(self, test_db_session):
        """Test that existing worker_status is updated, not duplicated"""
        # Arrange - Create existing worker_status
        from datetime import UTC, datetime

        existing = WorkerStatus(
            worker_name="elo_aggregator",
            last_run_at=datetime(2025, 1, 1, tzinfo=UTC),
            status="success",
            votes_processed=5,
        )
        test_db_session.add(existing)
        await test_db_session.commit()

        # Act
        await _update_worker_status(
            test_db_session,
            votes_processed=10,
            status="failed",
            error_message="Test error",
        )

        # Assert
        await test_db_session.commit()
        result = await test_db_session.execute(
            select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
        )
        all_statuses = result.scalars().all()

        assert len(all_statuses) == 1  # No duplicate
        status = all_statuses[0]
        assert status.votes_processed == 10
        assert status.status == "failed"
        assert status.error_message == "Test error"
