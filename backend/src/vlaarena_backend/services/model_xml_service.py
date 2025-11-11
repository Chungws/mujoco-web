"""
MuJoCo model XML composition service
"""

import logging
from pathlib import Path

from fastapi import HTTPException, status
from vlaarena_shared.mujoco_xml import get_model_xml

logger = logging.getLogger(__name__)


class ModelXMLService:
    """
    MuJoCo model XML composition service

    Handles XML composition for robot and scene combinations
    """

    def __init__(self, config_root: Path | None = None):
        """
        Initialize model XML service

        Args:
            config_root: Root config directory (defaults to project root/config/mujoco)
        """
        if config_root is None:
            # Default: backend/src/vlaarena_backend/services -> project root/config/mujoco
            config_root = Path(__file__).parent.parent.parent.parent.parent / "config" / "mujoco"
        self.config_root = config_root
        logger.info(f"ModelXMLService initialized with config_root: {config_root}")

    def get_composed_xml(self, robot_id: str, scene_id: str) -> str:
        """
        Get composed MuJoCo XML for robot and scene

        Args:
            robot_id: Robot identifier (e.g., "franka")
            scene_id: Scene identifier (e.g., "table")

        Returns:
            Complete MuJoCo XML string

        Raises:
            HTTPException: 404 if template, robot, or scene not found
            HTTPException: 500 if composition fails
        """
        logger.info(f"Composing XML for robot={robot_id}, scene={scene_id}")

        try:
            xml = get_model_xml(robot_id, scene_id, config_root=self.config_root)
            logger.info(f"Successfully composed XML for {robot_id}_{scene_id}")
            return xml

        except FileNotFoundError as e:
            logger.error(f"File not found: {e}")
            # Extract meaningful error message
            error_msg = str(e)
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=error_msg,
            ) from e

        except Exception as e:
            logger.error(f"Error composing XML: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Error composing XML: {e!s}",
            ) from e


# Singleton instance
_model_xml_service: ModelXMLService | None = None


def get_model_xml_service() -> ModelXMLService:
    """
    Get singleton ModelXMLService instance

    Returns:
        ModelXMLService instance
    """
    global _model_xml_service
    if _model_xml_service is None:
        _model_xml_service = ModelXMLService()
    return _model_xml_service
