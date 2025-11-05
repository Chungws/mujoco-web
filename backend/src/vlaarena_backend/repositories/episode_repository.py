"""
Episode repository for MongoDB operations using Beanie ODM
"""

from typing import Optional

from vlaarena_shared.mongodb_models import Episode


class EpisodeRepository:
    """
    Repository for Episode MongoDB operations

    Uses Beanie ODM for MongoDB interactions.
    Episodes are stored in MongoDB for efficient storage of
    variable-length actions and states arrays.
    """

    @staticmethod
    async def get_by_episode_id(episode_id: str) -> Optional[Episode]:
        """
        Get episode by episode_id

        Args:
            episode_id: Unique episode identifier (e.g., "ep_abc123")

        Returns:
            Episode document or None if not found
        """
        return await Episode.find_one(Episode.episode_id == episode_id)

    @staticmethod
    async def create(episode: Episode) -> Episode:
        """
        Create new episode in MongoDB

        Args:
            episode: Episode document to insert

        Returns:
            Inserted episode with MongoDB _id populated
        """
        return await episode.insert()

    @staticmethod
    async def get_by_turn_id(turn_id: str) -> list[Episode]:
        """
        Get all episodes for a turn (left and right)

        Args:
            turn_id: Turn identifier

        Returns:
            List of episodes (typically 2: left and right)
        """
        return await Episode.find(Episode.turn_id == turn_id).to_list()

    @staticmethod
    async def get_by_battle_id(battle_id: str) -> list[Episode]:
        """
        Get all episodes for a battle

        Args:
            battle_id: Battle identifier

        Returns:
            List of all episodes in the battle
        """
        return await Episode.find(Episode.battle_id == battle_id).to_list()
