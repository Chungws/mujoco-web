"""
Episode repository for MongoDB operations using Beanie ODM
"""

import logging

from pydantic import ValidationError
from pymongo.errors import ConnectionFailure, OperationFailure, ServerSelectionTimeoutError
from vlaarena_shared.mongodb_models import Episode

from vlaarena_backend.exceptions import EpisodeDatabaseError, EpisodeValidationError

logger = logging.getLogger(__name__)


class EpisodeRepository:
    """
    Repository for Episode MongoDB operations

    Uses Beanie ODM for MongoDB interactions.
    Episodes are stored in MongoDB for efficient storage of
    variable-length actions and states arrays.
    """

    @staticmethod
    async def get_by_episode_id(episode_id: str) -> Episode | None:
        """
        Get episode by episode_id

        Args:
            episode_id: Unique episode identifier (e.g., "ep_abc123")

        Returns:
            Episode document or None if not found

        Raises:
            EpisodeDatabaseError: If MongoDB operation fails
        """
        try:
            return await Episode.find_one(Episode.episode_id == episode_id)
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            logger.error(f"MongoDB connection error while fetching episode {episode_id}: {e}")
            raise EpisodeDatabaseError("get_by_episode_id", e) from e
        except OperationFailure as e:
            logger.error(f"MongoDB operation error while fetching episode {episode_id}: {e}")
            raise EpisodeDatabaseError("get_by_episode_id", e) from e
        except Exception as e:
            logger.error(f"Unexpected error while fetching episode {episode_id}: {e}")
            raise EpisodeDatabaseError("get_by_episode_id", e) from e

    @staticmethod
    async def create(episode: Episode) -> Episode:
        """
        Create new episode in MongoDB

        Args:
            episode: Episode document to insert

        Returns:
            Inserted episode with MongoDB _id populated

        Raises:
            EpisodeDatabaseError: If MongoDB operation fails
            EpisodeValidationError: If episode data is invalid
        """
        try:
            return await episode.insert()
        except ValidationError as e:
            logger.error(f"Validation error while creating episode {episode.episode_id}: {e}")
            raise EpisodeValidationError(episode.episode_id, e) from e
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            logger.error(
                f"MongoDB connection error while creating episode {episode.episode_id}: {e}"
            )
            raise EpisodeDatabaseError("create", e) from e
        except OperationFailure as e:
            logger.error(
                f"MongoDB operation error while creating episode {episode.episode_id}: {e}"
            )
            raise EpisodeDatabaseError("create", e) from e
        except Exception as e:
            logger.error(f"Unexpected error while creating episode {episode.episode_id}: {e}")
            raise EpisodeDatabaseError("create", e) from e

    @staticmethod
    async def get_by_turn_id(turn_id: str) -> list[Episode]:
        """
        Get all episodes for a turn (left and right)

        Args:
            turn_id: Turn identifier

        Returns:
            List of episodes (typically 2: left and right)

        Raises:
            EpisodeDatabaseError: If MongoDB operation fails
        """
        try:
            return await Episode.find(Episode.turn_id == turn_id).to_list()
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            logger.error(
                f"MongoDB connection error while fetching episodes for turn {turn_id}: {e}"
            )
            raise EpisodeDatabaseError("get_by_turn_id", e) from e
        except OperationFailure as e:
            logger.error(f"MongoDB operation error while fetching episodes for turn {turn_id}: {e}")
            raise EpisodeDatabaseError("get_by_turn_id", e) from e
        except Exception as e:
            logger.error(f"Unexpected error while fetching episodes for turn {turn_id}: {e}")
            raise EpisodeDatabaseError("get_by_turn_id", e) from e

    @staticmethod
    async def get_by_battle_id(battle_id: str) -> list[Episode]:
        """
        Get all episodes for a battle

        Args:
            battle_id: Battle identifier

        Returns:
            List of all episodes in the battle

        Raises:
            EpisodeDatabaseError: If MongoDB operation fails
        """
        try:
            return await Episode.find(Episode.battle_id == battle_id).to_list()
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            logger.error(
                f"MongoDB connection error while fetching episodes for battle {battle_id}: {e}"
            )
            raise EpisodeDatabaseError("get_by_battle_id", e) from e
        except OperationFailure as e:
            logger.error(
                f"MongoDB operation error while fetching episodes for battle {battle_id}: {e}"
            )
            raise EpisodeDatabaseError("get_by_battle_id", e) from e
        except Exception as e:
            logger.error(f"Unexpected error while fetching episodes for battle {battle_id}: {e}")
            raise EpisodeDatabaseError("get_by_battle_id", e) from e
