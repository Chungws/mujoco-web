"""
Models API endpoints
"""

import logging
from pathlib import Path

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import PlainTextResponse
from vlaarena_shared.schemas import ModelsListResponse

from ..services.model_service import ModelService, get_model_service

logger = logging.getLogger(__name__)

router = APIRouter()

# Root config directory for MuJoCo models
ROOT_CONFIG = Path(__file__).parent.parent.parent.parent.parent / "config" / "mujoco"


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

    Returns:
        Complete MuJoCo XML string

    Raises:
        404: If template, robot, or scene XML not found

    Example:
        GET /api/models/xml?robot_id=franka&scene_id=table

        Response: (text/xml)
        <mujoco model="franka_table">
          ...
        </mujoco>
    """
    logger.info(f"GET /api/models/xml - robot_id={robot_id}, scene_id={scene_id}")

    # Validate paths exist (before try-except to avoid HTTPException being caught)
    template_path = ROOT_CONFIG / "template.xml"
    if not template_path.exists():
        logger.error(f"Template not found: {template_path}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Template not found: {template_path}",
        )

    robot_path = ROOT_CONFIG / "robots" / f"{robot_id}.xml"
    if not robot_path.exists():
        logger.error(f"Robot not found: {robot_path}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot '{robot_id}' not found",
        )

    scene_path = ROOT_CONFIG / "scenes" / f"{scene_id}.xml"
    if not scene_path.exists():
        logger.error(f"Scene not found: {scene_path}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Scene '{scene_id}' not found",
        )

    try:
        # Load files
        template = template_path.read_text()
        robot_xml = robot_path.read_text()
        scene_xml = scene_path.read_text()

        # Compose final XML
        final_xml = template.format(
            model_name=f"{robot_id}_{scene_id}",
            robot_body=robot_xml,
            scene_body=scene_xml,
        )

        logger.info(f"Successfully composed XML for {robot_id}_{scene_id}")

        return final_xml

    except Exception as e:
        logger.error(f"Error reading or composing XML: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error composing XML: {e!s}",
        ) from e
