"""
Models API endpoints
"""

import logging

from fastapi import APIRouter, Depends
from vlaarena_shared.schemas import ModelsListResponse

from ..services.model_service import ModelService, get_model_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/models", response_model=ModelsListResponse)
async def get_models(
    model_service: ModelService = Depends(get_model_service),
) -> ModelsListResponse:
    """
    Get list of available models

    Returns list of active models that can be used in battles.
    Model configurations are loaded from config/models.yaml.

    Returns:
        ModelsListResponse with list of active models

    Example:
        GET /api/models

        Response:
        {
            "models": [
                {
                    "model_id": "gpt-4o-mini",
                    "name": "GPT-4o Mini",
                    "provider": "OpenAI",
                    "status": "active"
                },
                ...
            ]
        }
    """
    logger.info("GET /api/models - Listing available models")

    models = model_service.list_models()

    logger.info(f"Returning {len(models)} active models")

    return ModelsListResponse(models=models)
