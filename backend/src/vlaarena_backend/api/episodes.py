"""
Episode API endpoints
"""

import logging

from fastapi import APIRouter, HTTPException, status

from vlaarena_backend.exceptions import (
    EpisodeDatabaseError,
    EpisodeRepositoryError,
    EpisodeValidationError,
)
from vlaarena_backend.services.episode_service import EpisodeService
from vlaarena_shared.schemas import EpisodeResponse


logger = logging.getLogger(__name__)

router = APIRouter()


@router.get(
    "/episodes/{episode_id}",
    response_model=EpisodeResponse,
    status_code=status.HTTP_200_OK,
)
async def get_episode(episode_id: str):
    """
    Get episode by ID from MongoDB

    Flow:
    1. User provides episode_id
    2. System retrieves episode from MongoDB
    3. Returns episode data (actions, states, metrics) for frontend replay

    Args:
        episode_id: Unique episode identifier (e.g., "ep_abc123")

    Returns:
        EpisodeResponse with actions, states, and metrics

    Raises:
        HTTPException 404: If episode not found
        HTTPException 500: If internal error occurs
    """
    try:
        episode_service = EpisodeService()
        episode = await episode_service.get_episode_response(episode_id)

        if not episode:
            logger.error(f"Episode not found: {episode_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Episode not found: {episode_id}",
            )

        return episode

    except HTTPException:
        # Re-raise HTTPException as-is
        raise

    except EpisodeValidationError as e:
        logger.error(f"Validation error for episode {episode_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Invalid episode data: {str(e)}",
        )

    except EpisodeDatabaseError as e:
        logger.error(f"Database error while fetching episode {episode_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Database service temporarily unavailable",
        )

    except EpisodeRepositoryError as e:
        logger.error(f"Repository error for episode {episode_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get episode: {str(e)}",
        )

    except Exception as e:
        logger.error(f"Unexpected error while fetching episode {episode_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get episode: {str(e)}",
        )
