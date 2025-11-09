"""
Unit tests for EpisodeExecutor

Tests the episode execution service with mock VLA adapter.
"""

import numpy as np
import pytest
from vla_server_base.adapters.base import VLAModelAdapter
from vla_server_base.services.episode_executor import EpisodeExecutor


class MockVLAAdapter(VLAModelAdapter):
    """Mock VLA adapter for testing"""

    def __init__(self):
        super().__init__()
        self.model_id = "mock-test"
        self.device = "cpu"
        self.model_loaded = True

    def load_model(self, model_id: str, device: str = "cpu", cache_dir: str = "/tmp"):
        """Mock load - already loaded"""
        pass

    def preprocess_observation(self, obs: dict) -> dict:
        """Pass through observation"""
        return obs

    def preprocess_instruction(self, instruction: str) -> str:
        """Pass through instruction"""
        return instruction

    def predict(self, obs: dict, instruction: str) -> np.ndarray:
        """Return random 8-dim action"""
        return np.random.randn(8).astype(np.float32)

    def postprocess_action(self, action: np.ndarray) -> list[float]:
        """Convert to list"""
        return action.tolist()


# Simple 8-DOF robot XML for testing
TEST_XML = """
<mujoco model="test_robot">
  <compiler angle="radian"/>
  <option timestep="0.002"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" pos="0 0 0" size="0 0 0.05" type="plane"/>

    <body name="base" pos="0 0 0">
      <inertial pos="0 0 0" mass="4" diaginertia="0.4 0.4 0.4"/>
      <geom type="box" size="0.1 0.1 0.1"/>

      <body name="link1" pos="0 0 0.2">
        <joint name="joint1" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <geom type="cylinder" size="0.05 0.1"/>

        <body name="link2" pos="0 0 0.2">
          <joint name="joint2" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="cylinder" size="0.04 0.1"/>

          <body name="link3" pos="0 0 0.2">
            <joint name="joint3" type="hinge" axis="0 0 1"/>
            <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
            <geom type="box" size="0.02 0.02 0.02"/>

            <body name="finger1" pos="0.015 0 0">
              <joint name="joint4" type="slide" axis="1 0 0"/>
              <inertial pos="0 0 0" mass="0.1" diaginertia="0.01 0.01 0.01"/>
              <geom type="box" size="0.005 0.01 0.03"/>
            </body>

            <body name="finger2" pos="-0.015 0 0">
              <joint name="joint5" type="slide" axis="-1 0 0"/>
              <inertial pos="0 0 0" mass="0.1" diaginertia="0.01 0.01 0.01"/>
              <geom type="box" size="0.005 0.01 0.03"/>
            </body>

            <body name="dummy6">
              <joint name="joint6" type="slide" axis="0 0 1" range="-1 1"/>
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>
              <geom type="sphere" size="0.001"/>
            </body>

            <body name="dummy7">
              <joint name="joint7" type="slide" axis="0 0 1" range="-1 1"/>
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>
              <geom type="sphere" size="0.001"/>
            </body>

            <body name="dummy8">
              <joint name="joint8" type="slide" axis="0 0 1" range="-1 1"/>
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>
              <geom type="sphere" size="0.001"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="motor1" joint="joint1"/>
    <motor name="motor2" joint="joint2"/>
    <motor name="motor3" joint="joint3"/>
    <motor name="motor4" joint="joint4"/>
    <motor name="motor5" joint="joint5"/>
    <motor name="motor6" joint="joint6"/>
    <motor name="motor7" joint="joint7"/>
    <motor name="motor8" joint="joint8"/>
  </actuator>
</mujoco>
"""


class TestEpisodeExecutor:
    """Test EpisodeExecutor class"""

    def test_init(self):
        """Test executor initialization"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=10)

        assert executor.adapter == adapter
        assert executor.max_steps == 10

    def test_execute_episode_returns_correct_format(self):
        """Test execute_episode returns actions, states, duration_ms"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=5)

        result = executor.execute_episode(instruction="Pick up the cube", xml_string=TEST_XML)

        # Check return format
        assert "actions" in result
        assert "states" in result
        assert "duration_ms" in result

        # Check types
        assert isinstance(result["actions"], list)
        assert isinstance(result["states"], list)
        assert isinstance(result["duration_ms"], int)

    def test_execute_episode_correct_number_of_steps(self):
        """Test executor runs for max_steps"""
        adapter = MockVLAAdapter()
        max_steps = 10
        executor = EpisodeExecutor(adapter, max_steps=max_steps)

        result = executor.execute_episode(instruction="Pick up the cube", xml_string=TEST_XML)

        assert len(result["actions"]) == max_steps
        assert len(result["states"]) == max_steps

    def test_execute_episode_action_dimension(self):
        """Test actions have correct format and dimension"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=5)

        result = executor.execute_episode(instruction="Pick up the cube", xml_string=TEST_XML)

        for action in result["actions"]:
            assert isinstance(action, list)
            assert len(action) > 0  # At least one dimension
            assert all(isinstance(a, float) for a in action)

    def test_execute_episode_state_format(self):
        """Test state contains qpos, qvel, time"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=5)

        result = executor.execute_episode(instruction="Pick up the cube", xml_string=TEST_XML)

        for state in result["states"]:
            assert "qpos" in state
            assert "qvel" in state
            assert "time" in state
            assert isinstance(state["qpos"], list)
            assert isinstance(state["qvel"], list)
            assert isinstance(state["time"], float)

    def test_execute_episode_time_advances(self):
        """Test simulation time advances with each step"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=5)

        result = executor.execute_episode(instruction="Pick up the cube", xml_string=TEST_XML)

        times = [state["time"] for state in result["states"]]

        # Time should increase monotonically
        for i in range(1, len(times)):
            assert times[i] > times[i - 1]

    def test_execute_episode_duration_is_positive(self):
        """Test execution duration is positive"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=5)

        result = executor.execute_episode(instruction="Pick up the cube", xml_string=TEST_XML)

        assert result["duration_ms"] > 0

    def test_execute_episode_with_invalid_xml_raises_error(self):
        """Test invalid XML raises error"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=5)

        invalid_xml = "<mujoco><invalid></mujoco>"

        with pytest.raises((ValueError, RuntimeError)):
            executor.execute_episode(instruction="Pick up the cube", xml_string=invalid_xml)

    def test_execute_episode_with_different_instructions(self):
        """Test executor works with different instructions"""
        adapter = MockVLAAdapter()
        executor = EpisodeExecutor(adapter, max_steps=3)

        instructions = [
            "Pick up the cube",
            "Move to the left",
            "Place the object down",
        ]

        for instruction in instructions:
            result = executor.execute_episode(instruction=instruction, xml_string=TEST_XML)
            assert len(result["actions"]) == 3
            assert len(result["states"]) == 3
