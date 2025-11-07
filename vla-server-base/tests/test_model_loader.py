"""
Tests for MuJoCo XML composition (model_loader)
Target: 10 tests
"""

import pytest
from vla_server_base.config.model_loader import get_model_xml


class TestModelLoader:
    """Test XML composition logic"""

    def test_compose_valid_model(self, robot_id, scene_id):
        """Test composing valid robot + scene"""
        xml = get_model_xml(robot_id, scene_id)

        assert xml is not None
        assert isinstance(xml, str)
        assert len(xml) > 0

    def test_xml_contains_model_name(self, robot_id, scene_id):
        """Test XML contains correct model name"""
        xml = get_model_xml(robot_id, scene_id)

        expected_name = f"{robot_id}_{scene_id}"
        assert f'model="{expected_name}"' in xml

    def test_xml_contains_robot_elements(self, robot_id, scene_id):
        """Test XML contains robot-specific elements"""
        xml = get_model_xml(robot_id, scene_id)

        # Franka-specific elements
        assert "franka_base" in xml
        assert "joint1" in xml
        assert "gripper" in xml

    def test_xml_contains_scene_elements(self, robot_id, scene_id):
        """Test XML contains scene-specific elements"""
        xml = get_model_xml(robot_id, scene_id)

        # Table scene elements
        assert "table" in xml
        assert "table_top" in xml
        assert "red_cube" in xml

    def test_xml_contains_template_elements(self, robot_id, scene_id):
        """Test XML contains template elements"""
        xml = get_model_xml(robot_id, scene_id)

        # Template elements
        assert "<mujoco" in xml
        assert "<worldbody>" in xml
        assert "<light" in xml
        assert "floor" in xml

    def test_xml_is_valid_structure(self, robot_id, scene_id):
        """Test XML has valid MuJoCo structure"""
        xml = get_model_xml(robot_id, scene_id)

        # Check basic structure
        assert xml.startswith("<mujoco")
        assert "</mujoco>" in xml
        assert "<worldbody>" in xml
        assert "</worldbody>" in xml

    def test_invalid_robot_raises_error(self, scene_id):
        """Test invalid robot ID raises FileNotFoundError"""
        with pytest.raises(FileNotFoundError, match="Robot not found"):
            get_model_xml("invalid_robot", scene_id)

    def test_invalid_scene_raises_error(self, robot_id):
        """Test invalid scene ID raises FileNotFoundError"""
        with pytest.raises(FileNotFoundError, match="Scene not found"):
            get_model_xml(robot_id, "invalid_scene")

    def test_xml_has_actuators(self, robot_id, scene_id):
        """Test XML includes robot actuators"""
        xml = get_model_xml(robot_id, scene_id)

        assert "<actuator>" in xml or "<actuator" in xml
        assert "motor" in xml.lower()

    def test_xml_composition_deterministic(self, robot_id, scene_id):
        """Test XML composition is deterministic"""
        xml1 = get_model_xml(robot_id, scene_id)
        xml2 = get_model_xml(robot_id, scene_id)

        assert xml1 == xml2
