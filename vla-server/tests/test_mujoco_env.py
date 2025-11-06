"""
Tests for MuJoCo Environment
Target: 15 tests
"""

import numpy as np
import pytest
from vla_server.config.model_loader import get_model_xml
from vla_server.services.mujoco_env import MuJoCoEnvironment


@pytest.fixture
def xml_string(robot_id, scene_id):
    """Fixture to provide MuJoCo XML string"""
    return get_model_xml(robot_id, scene_id)


@pytest.fixture
def env(xml_string):
    """Fixture to provide MuJoCo environment"""
    return MuJoCoEnvironment(xml_string)


class TestMuJoCoEnvironmentInit:
    """Test environment initialization"""

    def test_init_from_xml_string(self, xml_string):
        """Test initialization from XML string"""
        env = MuJoCoEnvironment(xml_string)

        assert env.model is not None
        assert env.data is not None
        assert env.renderer is not None

    def test_init_stores_initial_state(self, env):
        """Test initial state is stored"""
        assert env.initial_qpos is not None
        assert env.initial_qvel is not None
        assert isinstance(env.initial_qpos, np.ndarray)
        assert isinstance(env.initial_qvel, np.ndarray)

    def test_init_with_invalid_xml_raises_error(self):
        """Test invalid XML raises error"""
        invalid_xml = "<invalid>xml</invalid>"

        with pytest.raises(ValueError):  # MuJoCo raises ValueError for invalid XML
            MuJoCoEnvironment(invalid_xml)


class TestMuJoCoEnvironmentReset:
    """Test environment reset"""

    def test_reset_returns_observation(self, env):
        """Test reset returns observation"""
        obs = env.reset()

        assert isinstance(obs, dict)
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs

    def test_reset_restores_initial_state(self, env):
        """Test reset restores initial state"""
        # Modify state
        env.data.qpos[:] = env.data.qpos + 0.1
        env.data.qvel[:] = env.data.qvel + 0.1

        # Reset
        env.reset()

        # Check state restored
        np.testing.assert_array_almost_equal(env.data.qpos, env.initial_qpos)
        np.testing.assert_array_almost_equal(env.data.qvel, env.initial_qvel)

    def test_reset_clears_time(self, env):
        """Test reset clears simulation time"""
        # Advance time
        env.step([0.0] * 8)
        assert env.data.time > 0

        # Reset
        env.reset()

        assert env.data.time == 0.0


class TestMuJoCoEnvironmentStep:
    """Test environment step"""

    def test_step_accepts_action(self, env):
        """Test step accepts action and returns observation"""
        action = [0.0] * 8  # 8-dim action

        obs = env.step(action)

        assert isinstance(obs, dict)
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs

    def test_step_applies_action(self, env):
        """Test step applies action to actuators"""
        env.reset()

        action = [0.1] * 8

        env.step(action)

        # State should change (not necessarily qpos, but ctrl should be set)
        assert not np.array_equal(env.data.ctrl, np.zeros_like(env.data.ctrl))

    def test_step_advances_time(self, env):
        """Test step advances simulation time"""
        env.reset()
        initial_time = env.data.time

        env.step([0.0] * 8)

        assert env.data.time > initial_time

    def test_step_with_short_action(self, env):
        """Test step with action shorter than num_actuators"""
        env.reset()
        action = [0.1] * 5  # Shorter than 8

        obs = env.step(action)

        assert isinstance(obs, dict)

    def test_step_with_long_action(self, env):
        """Test step with action longer than num_actuators"""
        env.reset()
        action = [0.1] * 10  # Longer than 8

        obs = env.step(action)

        assert isinstance(obs, dict)


class TestMuJoCoEnvironmentObservation:
    """Test observation retrieval"""

    def test_get_observation_returns_correct_format(self, env):
        """Test get_observation returns correct format"""
        env.reset()
        obs = env.get_observation()

        assert isinstance(obs, dict)
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs

    def test_observation_image_shape(self, env):
        """Test observation image has correct shape"""
        env.reset()
        obs = env.get_observation()

        image = obs["image"]
        assert image.shape == (224, 224, 3)  # RGB image

    def test_observation_qpos_qvel_are_arrays(self, env):
        """Test qpos and qvel are numpy arrays"""
        env.reset()
        obs = env.get_observation()

        assert isinstance(obs["qpos"], np.ndarray)
        assert isinstance(obs["qvel"], np.ndarray)


class TestMuJoCoEnvironmentState:
    """Test state retrieval"""

    def test_get_state_returns_correct_format(self, env):
        """Test get_state returns correct format"""
        env.reset()
        state = env.get_state()

        assert isinstance(state, dict)
        assert "qpos" in state
        assert "qvel" in state
        assert "time" in state

    def test_get_state_returns_lists(self, env):
        """Test get_state returns lists for JSON serialization"""
        env.reset()
        state = env.get_state()

        assert isinstance(state["qpos"], list)
        assert isinstance(state["qvel"], list)
        assert isinstance(state["time"], float)
