"""
MuJoCo model XML composition utilities
Dynamically combines robot and scene XMLs
"""

from pathlib import Path


def get_model_xml(robot_id: str, scene_id: str, config_root: Path | None = None) -> str:
    """
    Dynamically compose MuJoCo XML from robot and scene

    Args:
        robot_id: Robot identifier (e.g., "franka")
        scene_id: Scene identifier (e.g., "table")
        config_root: Root config directory (defaults to project root/config/mujoco)

    Returns:
        Complete MuJoCo XML string

    Raises:
        FileNotFoundError: If template, robot, or scene XML not found
    """
    # Default config root (project root / config / mujoco)
    if config_root is None:
        config_root = Path(__file__).parent.parent.parent.parent / "config" / "mujoco"

    # Load template
    template_path = config_root / "template.xml"
    if not template_path.exists():
        raise FileNotFoundError(f"Template not found: {template_path}")
    template = template_path.read_text()

    # Load robot body
    robot_path = config_root / "robots" / f"{robot_id}.xml"
    if not robot_path.exists():
        raise FileNotFoundError(f"Robot '{robot_id}' not found at: {robot_path}")
    robot_xml = robot_path.read_text()

    # Load scene body
    scene_path = config_root / "scenes" / f"{scene_id}.xml"
    if not scene_path.exists():
        raise FileNotFoundError(f"Scene '{scene_id}' not found at: {scene_path}")
    scene_xml = scene_path.read_text()

    # Compose final XML
    final_xml = template.format(
        model_name=f"{robot_id}_{scene_id}",
        robot_body=robot_xml,
        scene_body=scene_xml,
    )

    return final_xml
