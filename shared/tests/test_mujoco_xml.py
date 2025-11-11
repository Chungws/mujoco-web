"""
Tests for MuJoCo XML composition utilities
"""

from pathlib import Path

import pytest
from vlaarena_shared.mujoco_xml import get_model_xml


class TestGetModelXML:
    """Test get_model_xml function"""

    def test_get_model_xml_success(self):
        """Test successful XML composition with valid robot and scene"""
        # Arrange
        robot_id = "franka"
        scene_id = "table"

        # Act
        xml = get_model_xml(robot_id, scene_id)

        # Assert
        assert xml is not None
        assert isinstance(xml, str)
        assert len(xml) > 0

        # Verify XML structure
        assert "<mujoco" in xml
        assert 'model="franka_table"' in xml
        assert "<compiler" in xml
        assert "<worldbody>" in xml
        assert "<actuator>" in xml
        assert "</mujoco>" in xml

        # Verify robot body is included
        assert "joint1" in xml
        assert "joint2" in xml
        assert "gripper" in xml

        # Verify scene body is included
        assert "table" in xml
        assert "red_cube" in xml or "blue_cube" in xml

    def test_get_model_xml_robot_not_found(self):
        """Test FileNotFoundError when robot XML does not exist"""
        # Arrange
        robot_id = "nonexistent_robot"
        scene_id = "table"

        # Act & Assert
        with pytest.raises(FileNotFoundError) as exc_info:
            get_model_xml(robot_id, scene_id)

        # Verify error message contains robot info
        assert "Robot" in str(exc_info.value)
        assert robot_id in str(exc_info.value)

    def test_get_model_xml_scene_not_found(self):
        """Test FileNotFoundError when scene XML does not exist"""
        # Arrange
        robot_id = "franka"
        scene_id = "nonexistent_scene"

        # Act & Assert
        with pytest.raises(FileNotFoundError) as exc_info:
            get_model_xml(robot_id, scene_id)

        # Verify error message contains scene info
        assert "Scene" in str(exc_info.value)
        assert scene_id in str(exc_info.value)

    def test_get_model_xml_template_structure(self):
        """Test that composed XML includes template elements"""
        # Arrange
        robot_id = "franka"
        scene_id = "table"

        # Act
        xml = get_model_xml(robot_id, scene_id)

        # Assert - Verify template elements
        assert '<compiler angle="radian"/>' in xml
        assert '<option timestep="0.002"/>' in xml
        assert '<light pos="0 0 3"' in xml
        assert '<geom name="floor"' in xml

        # Verify actuators for 7 joints + gripper
        assert "motor1" in xml
        assert "motor7" in xml
        assert "gripper_motor" in xml

    def test_get_model_xml_custom_config_root(self):
        """Test with custom config root path"""
        # Arrange
        robot_id = "franka"
        scene_id = "table"
        # Correct path: shared/tests -> shared -> mujoco-web -> config/mujoco
        config_root = Path(__file__).parent.parent.parent / "config" / "mujoco"

        # Act
        xml = get_model_xml(robot_id, scene_id, config_root=config_root)

        # Assert
        assert xml is not None
        assert "<mujoco" in xml
        assert 'model="franka_table"' in xml
