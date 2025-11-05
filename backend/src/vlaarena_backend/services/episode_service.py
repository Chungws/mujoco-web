"""
Episode service for business logic
"""

from typing import Optional

from vlaarena_backend.repositories.episode_repository import EpisodeRepository
from vlaarena_shared.mongodb_models import Episode
from vlaarena_shared.schemas import EpisodeResponse, EpisodeState


class EpisodeService:
    """
    Service layer for Episode operations

    Handles business logic for episode retrieval from MongoDB.
    Episodes contain actions, states, and metrics for frontend replay.
    """

    def __init__(self):
        """Initialize episode service"""
        self.repository = EpisodeRepository()

    async def get_episode(self, episode_id: str) -> Optional[Episode]:
        """
        Get episode by ID

        Args:
            episode_id: Unique episode identifier

        Returns:
            Episode document or None if not found
        """
        return await self.repository.get_by_episode_id(episode_id)

    async def get_episode_response(self, episode_id: str) -> Optional[EpisodeResponse]:
        """
        Get episode formatted as API response

        Args:
            episode_id: Unique episode identifier

        Returns:
            EpisodeResponse or None if not found

        Note:
            Converts MongoDB Episode document to Pydantic schema
            with only the fields needed for frontend replay.
            Handles Optional fields (e.g., final_distance_to_goal) gracefully.
        """
        episode = await self.get_episode(episode_id)
        if not episode:
            return None

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
            metrics={
                "success": episode.metrics.success,
                "total_steps": episode.metrics.total_steps,
                "max_steps": episode.metrics.max_steps,
                "final_distance_to_goal": episode.metrics.final_distance_to_goal or 0.0,
            },
        )
