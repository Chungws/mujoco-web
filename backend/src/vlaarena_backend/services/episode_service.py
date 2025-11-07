"""
Episode service for business logic
"""

import logging

from pydantic import ValidationError
from vlaarena_shared.mongodb_models import Episode
from vlaarena_shared.schemas import EpisodeResponse, EpisodeState

from vlaarena_backend.exceptions import EpisodeValidationError
from vlaarena_backend.repositories.episode_repository import EpisodeRepository

logger = logging.getLogger(__name__)


class EpisodeService:
    """
    Service layer for Episode operations

    Handles business logic for episode retrieval from MongoDB.
    Episodes contain actions, states, and metrics for frontend replay.
    """

    def __init__(self):
        """Initialize episode service"""
        self.repository = EpisodeRepository()

    async def get_episode(self, episode_id: str) -> Episode | None:
        """
        Get episode by ID

        Args:
            episode_id: Unique episode identifier

        Returns:
            Episode document or None if not found

        Raises:
            EpisodeDatabaseError: If MongoDB operation fails
        """
        return await self.repository.get_by_episode_id(episode_id)

    async def get_episode_response(self, episode_id: str) -> EpisodeResponse | None:
        """
        Get episode formatted as API response

        Args:
            episode_id: Unique episode identifier

        Returns:
            EpisodeResponse or None if not found

        Raises:
            EpisodeDatabaseError: If MongoDB operation fails
            EpisodeValidationError: If episode data is invalid

        Note:
            Converts MongoDB Episode document to Pydantic schema
            with only the fields needed for frontend replay (actions and states).
        """
        episode = await self.get_episode(episode_id)
        if not episode:
            return None

        try:
            # Convert MongoDB State objects to Pydantic EpisodeState schemas
            episode_states: list[EpisodeState] = [
                EpisodeState(
                    qpos=state.qpos,
                    qvel=state.qvel,
                    time=state.time,
                )
                for state in episode.states
            ]

            return EpisodeResponse(
                episode_id=episode.episode_id,
                actions=episode.actions,
                states=episode_states,
            )
        except ValidationError as e:
            logger.error(f"Validation error while converting episode {episode_id} to response: {e}")
            raise EpisodeValidationError(episode_id, e) from e
        except Exception as e:
            logger.error(f"Unexpected error while converting episode {episode_id} to response: {e}")
            raise EpisodeValidationError(episode_id, e) from e
