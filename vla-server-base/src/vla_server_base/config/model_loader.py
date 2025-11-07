"""
MuJoCo model XML composition
Dynamically combines robot and scene XMLs
"""

from pathlib import Path

# Root config directory
ROOT_CONFIG = Path(__file__).parent.parent.parent.parent.parent / "config" / "mujoco"


def get_model_xml(robot_id: str, scene_id: str) -> str:
    """
    Dynamically compose MuJoCo XML from robot and scene

    Args:
        robot_id: Robot identifier (e.g., "franka")
        scene_id: Scene identifier (e.g., "table")

    Returns:
        Complete MuJoCo XML string

    Raises:
        FileNotFoundError: If robot or scene XML not found
    """
    # Load template
    template_path = ROOT_CONFIG / "template.xml"
    if not template_path.exists():
        raise FileNotFoundError(f"Template not found: {template_path}")
    template = template_path.read_text()

    # Load robot body
    robot_path = ROOT_CONFIG / "robots" / f"{robot_id}.xml"
    if not robot_path.exists():
        raise FileNotFoundError(f"Robot not found: {robot_path}")
    robot_xml = robot_path.read_text()

    # Load scene body
    scene_path = ROOT_CONFIG / "scenes" / f"{scene_id}.xml"
    if not scene_path.exists():
        raise FileNotFoundError(f"Scene not found: {scene_path}")
    scene_xml = scene_path.read_text()

    # Compose final XML
    final_xml = template.format(
        model_name=f"{robot_id}_{scene_id}", robot_body=robot_xml, scene_body=scene_xml
    )

    return final_xml
