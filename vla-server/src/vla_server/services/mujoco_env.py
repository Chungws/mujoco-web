"""
MuJoCo environment management for VLA execution
Handles robot/scene loading, simulation stepping, observation extraction
"""

from pathlib import Path

import mujoco
import numpy as np

from ..config import settings


class MuJoCoEnvironment:
    """MuJoCo simulation environment"""

    def __init__(self, robot_id: str, scene_id: str):
        """
        Initialize MuJoCo environment

        Args:
            robot_id: Robot identifier (e.g., "franka")
            scene_id: Scene identifier (e.g., "table")

        Raises:
            FileNotFoundError: If robot or scene model not found
        """
        self.robot_id = robot_id
        self.scene_id = scene_id

        # Load model
        model_path = self._get_model_path(robot_id, scene_id)
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        # Renderer for observations
        self.renderer = mujoco.Renderer(self.model, height=224, width=224)

        # Store initial state
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()

    def _get_model_path(self, robot_id: str, scene_id: str) -> Path:
        """
        Get path to MuJoCo XML model

        Args:
            robot_id: Robot identifier
            scene_id: Scene identifier

        Returns:
            Path to MuJoCo XML file

        Raises:
            FileNotFoundError: If model file not found
        """
        base_path = Path(settings.mujoco_model_path)

        # For MVP, we use scene.xml from robot directory
        # In future, we can support scene-specific configurations
        model_path = base_path / "robots" / robot_id / "scene.xml"

        if not model_path.exists():
            raise FileNotFoundError(f"Robot model not found: {model_path}")

        # Validate scene_id exists (for MVP, just check if directory exists)
        scene_path = base_path / "scenes" / scene_id
        if not scene_path.exists() and scene_id != "table":
            # For MVP, "table" scene is implicit in robot's scene.xml
            raise FileNotFoundError(f"Scene not found: {scene_path}")

        return model_path

    def reset(self) -> dict:
        """
        Reset environment to initial state

        Returns:
            Observation after reset
        """
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = self.initial_qvel
        self.data.time = 0.0
        mujoco.mj_forward(self.model, self.data)

        return self.get_observation()

    def step(self, action: list[float]) -> dict:
        """
        Step simulation with action

        Args:
            action: 8-dim action vector (or any length, will be truncated/padded)

        Returns:
            Observation after step
        """
        # Apply action to actuators (truncate if too long, pad if too short)
        action_array = np.array(action)
        num_actuators = self.model.nu

        if len(action_array) >= num_actuators:
            # Truncate if action is too long
            self.data.ctrl[:] = action_array[:num_actuators]
        else:
            # Pad with zeros if action is too short
            self.data.ctrl[: len(action_array)] = action_array
            self.data.ctrl[len(action_array) :] = 0.0

        # Step simulation
        mujoco.mj_step(self.model, self.data)

        return self.get_observation()

    def get_observation(self) -> dict:
        """
        Get current observation

        Returns:
            Dictionary with image, qpos, qvel
        """
        # Render image
        self.renderer.update_scene(self.data)
        image = self.renderer.render()

        return {
            "image": image,
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
        }

    def get_state(self) -> dict:
        """
        Get current state for episode recording

        Returns:
            Dictionary with qpos, qvel, time (all as lists/float for JSON serialization)
        """
        return {
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "time": float(self.data.time),
        }
