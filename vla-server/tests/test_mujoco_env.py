"""
Tests for MuJoCo environment management
Following TDD: Red → Green → Refactor

Test Coverage:
1. Environment initialization
2. Robot/scene loading
3. State management (reset, get_state)
4. Simulation stepping
5. Observation extraction
6. Error handling
"""

import numpy as np
import pytest

from vla_server.services.mujoco_env import MuJoCoEnvironment


class TestMuJoCoEnvironmentInitialization:
    """Test suite for MuJoCo environment initialization"""

    def test_load_franka_robot(self):
        """
        Test loading Franka Emika Panda robot

        Arrange: Create environment with franka robot
        Act: Load environment
        Assert: Model loaded successfully, correct DOF
        """
        # Arrange & Act
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Assert
        assert env.model is not None
        assert env.data is not None
        assert env.model.nq >= 7  # At least 7 joints for Franka

    def test_invalid_robot_id_raises_error(self):
        """
        Test that invalid robot ID raises FileNotFoundError

        Arrange: Prepare invalid robot ID
        Act: Attempt to create environment
        Assert: Raises FileNotFoundError
        """
        # Arrange & Act & Assert
        with pytest.raises(FileNotFoundError):
            MuJoCoEnvironment(robot_id="invalid_robot", scene_id="table")

    def test_invalid_scene_id_raises_error(self):
        """
        Test that invalid scene ID raises FileNotFoundError

        Arrange: Prepare invalid scene ID
        Act: Attempt to create environment
        Assert: Raises FileNotFoundError
        """
        # Arrange & Act & Assert
        with pytest.raises(FileNotFoundError):
            MuJoCoEnvironment(robot_id="franka", scene_id="invalid_scene")

    def test_initial_state_stored(self):
        """
        Test that initial state is stored correctly

        Arrange: Create environment
        Act: Access initial state
        Assert: Initial qpos and qvel are stored
        """
        # Arrange & Act
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Assert
        assert hasattr(env, "initial_qpos")
        assert hasattr(env, "initial_qvel")
        assert len(env.initial_qpos) == env.model.nq
        assert len(env.initial_qvel) == env.model.nv


class TestMuJoCoEnvironmentState:
    """Test suite for state management"""

    def test_reset_environment(self):
        """
        Test environment reset to initial state

        Arrange: Create environment and modify state
        Act: Reset environment
        Assert: State returns to initial configuration
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        initial_qpos = env.data.qpos.copy()

        # Modify state
        env.data.qpos[:] = 999.0

        # Act
        env.reset()

        # Assert
        assert not (env.data.qpos == 999.0).all()
        np.testing.assert_array_almost_equal(env.data.qpos, initial_qpos)

    def test_reset_returns_observation(self):
        """
        Test that reset returns observation

        Arrange: Create environment
        Act: Reset environment
        Assert: Returns observation dict
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        obs = env.reset()

        # Assert
        assert isinstance(obs, dict)
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs

    def test_get_state(self):
        """
        Test getting current state

        Arrange: Create environment
        Act: Get state
        Assert: Returns qpos, qvel, time
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        state = env.get_state()

        # Assert
        assert "qpos" in state
        assert "qvel" in state
        assert "time" in state
        assert len(state["qpos"]) == env.model.nq
        assert len(state["qvel"]) == env.model.nv
        assert isinstance(state["time"], float)

    def test_get_state_returns_lists(self):
        """
        Test that get_state returns lists not arrays

        Arrange: Create environment
        Act: Get state
        Assert: Values are lists
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        state = env.get_state()

        # Assert
        assert isinstance(state["qpos"], list)
        assert isinstance(state["qvel"], list)


class TestMuJoCoEnvironmentSimulation:
    """Test suite for simulation stepping"""

    def test_step_simulation(self):
        """
        Test stepping simulation with action

        Arrange: Create environment
        Act: Apply action and step
        Assert: Simulation time advances
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        initial_time = env.data.time
        action = [0.0] * 8  # 8-dim action

        # Act
        env.step(action)

        # Assert
        assert env.data.time > initial_time

    def test_step_returns_observation(self):
        """
        Test that step returns observation

        Arrange: Create environment
        Act: Step with action
        Assert: Returns observation dict
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        action = [0.0] * 8

        # Act
        obs = env.step(action)

        # Assert
        assert isinstance(obs, dict)
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs

    def test_multiple_steps(self):
        """
        Test multiple simulation steps

        Arrange: Create environment
        Act: Step multiple times
        Assert: Time advances correctly
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        action = [0.0] * 8
        times = []

        # Act
        for _ in range(5):
            env.step(action)
            times.append(env.data.time)

        # Assert
        assert all(times[i] < times[i + 1] for i in range(len(times) - 1))

    def test_action_applied_to_actuators(self):
        """
        Test that actions are applied to actuators

        Arrange: Create environment
        Act: Apply non-zero action
        Assert: Control values are set
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        action = [0.5] * env.model.nu

        # Act
        env.step(action)

        # Assert
        assert (env.data.ctrl == 0.5).all()


class TestMuJoCoEnvironmentObservation:
    """Test suite for observation extraction"""

    def test_get_observation(self):
        """
        Test getting observation from environment

        Arrange: Create environment
        Act: Get observation
        Assert: Returns dict with required keys
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        obs = env.get_observation()

        # Assert
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs

    def test_observation_image_format(self):
        """
        Test that observation image has correct format

        Arrange: Create environment
        Act: Get observation
        Assert: Image is RGB with correct dimensions
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        obs = env.get_observation()

        # Assert
        assert obs["image"].shape[-1] == 3  # RGB
        assert obs["image"].shape[0] == 224  # Height
        assert obs["image"].shape[1] == 224  # Width

    def test_observation_proprioception(self):
        """
        Test that observation includes proprioception

        Arrange: Create environment
        Act: Get observation
        Assert: qpos and qvel are numpy arrays
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        obs = env.get_observation()

        # Assert
        assert isinstance(obs["qpos"], np.ndarray)
        assert isinstance(obs["qvel"], np.ndarray)
        assert len(obs["qpos"]) == env.model.nq
        assert len(obs["qvel"]) == env.model.nv


class TestMuJoCoEnvironmentErrorHandling:
    """Test suite for error handling"""

    def test_step_with_wrong_action_dimension(self):
        """
        Test stepping with wrong action dimension

        Arrange: Create environment
        Act: Step with wrong action size
        Assert: Handles gracefully (truncates or pads)
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act - action too long (should truncate)
        action_long = [0.0] * 20
        obs = env.step(action_long)

        # Assert - should not raise error
        assert obs is not None

    def test_step_with_short_action(self):
        """
        Test stepping with action shorter than actuators

        Arrange: Create environment
        Act: Step with short action
        Assert: Handles gracefully
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act - action too short
        action_short = [0.0] * 3
        obs = env.step(action_short)

        # Assert - should not raise error
        assert obs is not None
