"""
Models API endpoints
"""

import logging

from fastapi import APIRouter, Depends
from fastapi.responses import PlainTextResponse
from vlaarena_shared.schemas import ModelsListResponse

from ..services.model_service import ModelService, get_model_service
from ..services.model_xml_service import ModelXMLService, get_model_xml_service

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


@router.get("/models/xml", response_class=PlainTextResponse)
async def get_model_xml(
    robot_id: str,
    scene_id: str,
    xml_service: ModelXMLService = Depends(get_model_xml_service),
) -> str:
    """
    Get composed MuJoCo XML for robot and scene

    Returns complete MuJoCo XML string by composing:
    - Template (compiler, worldbody structure, actuators)
    - Robot body (e.g., franka.xml)
    - Scene body (e.g., table.xml)

    This XML can be used by MuJoCo WASM in the frontend for visualization.

    Args:
        robot_id: Robot identifier (e.g., "franka")
        scene_id: Scene identifier (e.g., "table")
        xml_service: ModelXMLService dependency

    Returns:
        Complete MuJoCo XML string

    Raises:
        404: If template, robot, or scene XML not found
        500: If XML composition fails

    Example:
        GET /api/models/xml?robot_id=franka&scene_id=table

        Response: (text/xml)
        <mujoco model="franka_table">
          ...
        </mujoco>
    """
    logger.info(f"GET /api/models/xml - robot_id={robot_id}, scene_id={scene_id}")

    return xml_service.get_composed_xml(robot_id, scene_id)
