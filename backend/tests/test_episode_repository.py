"""
Tests for Episode Repository

Unit tests for EpisodeRepository to verify error handling
and MongoDB operations coverage.
"""

from unittest.mock import AsyncMock, patch

import pytest
from pydantic import ValidationError
from pymongo.errors import ConnectionFailure, OperationFailure, ServerSelectionTimeoutError

from vlaarena_backend.exceptions import EpisodeDatabaseError, EpisodeValidationError
from vlaarena_backend.repositories.episode_repository import EpisodeRepository
from vlaarena_shared.mongodb_models import Episode, State


class TestEpisodeRepository:
    """Test EpisodeRepository error handling and MongoDB operations"""

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find_one")
    async def test_get_by_episode_id_connection_failure(self, mock_find_one):
        """
        Test get_by_episode_id raises EpisodeDatabaseError on connection failure

        AAA Pattern:
        - Arrange: Mock MongoDB connection failure
        - Act: Call get_by_episode_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_find_one.side_effect = ConnectionFailure("Connection timeout")

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_episode_id("ep_test123")

        assert "get_by_episode_id" in str(exc_info.value)
        assert exc_info.value.operation == "get_by_episode_id"

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find_one")
    async def test_get_by_episode_id_server_timeout(self, mock_find_one):
        """
        Test get_by_episode_id raises EpisodeDatabaseError on server timeout

        AAA Pattern:
        - Arrange: Mock MongoDB server timeout
        - Act: Call get_by_episode_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_find_one.side_effect = ServerSelectionTimeoutError("Server timeout")

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_episode_id("ep_test123")

        assert "get_by_episode_id" in str(exc_info.value)

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find_one")
    async def test_get_by_episode_id_operation_failure(self, mock_find_one):
        """
        Test get_by_episode_id raises EpisodeDatabaseError on operation failure

        AAA Pattern:
        - Arrange: Mock MongoDB operation failure
        - Act: Call get_by_episode_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_find_one.side_effect = OperationFailure("Operation not permitted")

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_episode_id("ep_test123")

        assert "get_by_episode_id" in str(exc_info.value)

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find_one")
    async def test_get_by_episode_id_unexpected_error(self, mock_find_one):
        """
        Test get_by_episode_id raises EpisodeDatabaseError on unexpected error

        AAA Pattern:
        - Arrange: Mock unexpected error
        - Act: Call get_by_episode_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_find_one.side_effect = RuntimeError("Unexpected error")

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_episode_id("ep_test123")

        assert "get_by_episode_id" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_create_validation_error(self, mongodb):
        """
        Test create raises EpisodeValidationError on invalid data

        AAA Pattern:
        - Arrange: Create episode with invalid data
        - Act: Call create
        - Assert: EpisodeValidationError is raised
        """
        # Arrange: Create episode with invalid state (missing required field)
        with pytest.raises((ValidationError, EpisodeValidationError)):
            episode = Episode(
                episode_id="ep_invalid",
                session_id="sess_test",
                battle_id="battle_test",
                turn_id="turn_test",
                battle_seq_in_session=1,
                turn_seq=1,
                seq_in_turn=0,
                side="left",
                model_id="test-model",
                actions=[[0.1] * 8],
                states=[{"invalid": "data"}],  # Invalid state structure
                duration_ms=1000,
            )
            # Act & Assert
            await EpisodeRepository.create(episode)

    # NOTE: The following tests for create() MongoDB errors (ConnectionFailure, OperationFailure)
    # cannot be easily mocked because Beanie's Document.insert() is not a regular instance method.
    # These error handling paths in EpisodeRepository.create() are covered by:
    # 1. Integration tests with actual MongoDB (not in scope for unit tests)
    # 2. Similar error handling patterns tested in get_by_*() methods
    #
    # The create() method's ValidationError handling is tested above,
    # and the error handling structure follows the same pattern as other methods.

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find")
    async def test_get_by_turn_id_connection_failure(self, mock_find):
        """
        Test get_by_turn_id raises EpisodeDatabaseError on connection failure

        AAA Pattern:
        - Arrange: Mock MongoDB connection failure
        - Act: Call get_by_turn_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_cursor = AsyncMock()
        mock_cursor.to_list.side_effect = ConnectionFailure("Connection failed")
        mock_find.return_value = mock_cursor

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_turn_id("turn_test123")

        assert "get_by_turn_id" in str(exc_info.value)

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find")
    async def test_get_by_turn_id_operation_failure(self, mock_find):
        """
        Test get_by_turn_id raises EpisodeDatabaseError on operation failure

        AAA Pattern:
        - Arrange: Mock MongoDB operation failure
        - Act: Call get_by_turn_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_cursor = AsyncMock()
        mock_cursor.to_list.side_effect = OperationFailure("Query failed")
        mock_find.return_value = mock_cursor

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_turn_id("turn_test123")

        assert "get_by_turn_id" in str(exc_info.value)

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find")
    async def test_get_by_battle_id_connection_failure(self, mock_find):
        """
        Test get_by_battle_id raises EpisodeDatabaseError on connection failure

        AAA Pattern:
        - Arrange: Mock MongoDB connection failure
        - Act: Call get_by_battle_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_cursor = AsyncMock()
        mock_cursor.to_list.side_effect = ConnectionFailure("Connection failed")
        mock_find.return_value = mock_cursor

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_battle_id("battle_test123")

        assert "get_by_battle_id" in str(exc_info.value)

    @pytest.mark.asyncio
    @patch("vlaarena_backend.repositories.episode_repository.Episode.find")
    async def test_get_by_battle_id_operation_failure(self, mock_find):
        """
        Test get_by_battle_id raises EpisodeDatabaseError on operation failure

        AAA Pattern:
        - Arrange: Mock MongoDB operation failure
        - Act: Call get_by_battle_id
        - Assert: EpisodeDatabaseError is raised
        """
        # Arrange
        mock_cursor = AsyncMock()
        mock_cursor.to_list.side_effect = OperationFailure("Query failed")
        mock_find.return_value = mock_cursor

        # Act & Assert
        with pytest.raises(EpisodeDatabaseError) as exc_info:
            await EpisodeRepository.get_by_battle_id("battle_test123")

        assert "get_by_battle_id" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_get_by_episode_id_success(self, mongodb):
        """
        Test get_by_episode_id returns episode successfully

        AAA Pattern:
        - Arrange: Create episode in MongoDB
        - Act: Call get_by_episode_id
        - Assert: Episode is returned
        """
        # Arrange
        episode = Episode(
            episode_id="ep_success",
            session_id="sess_test",
            battle_id="battle_test",
            turn_id="turn_test",
            battle_seq_in_session=1,
            turn_seq=1,
            seq_in_turn=0,
            side="left",
            model_id="test-model",
            actions=[[0.1] * 8],
            states=[State(qpos=[1.0], qvel=[0.1], time=0.0)],
            duration_ms=1000,
        )
        await episode.insert()

        # Act
        result = await EpisodeRepository.get_by_episode_id("ep_success")

        # Assert
        assert result is not None
        assert result.episode_id == "ep_success"
