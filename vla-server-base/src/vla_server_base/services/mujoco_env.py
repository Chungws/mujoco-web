"""
MuJoCo environment management for VLA execution
Stateless design - accepts XML string directly
"""

import mujoco
import numpy as np


class MuJoCoEnvironment:
    """Stateless MuJoCo simulation environment"""

    def __init__(self, xml_string: str):
        """
        Initialize MuJoCo environment from XML string

        Args:
            xml_string: Complete MuJoCo XML model
        """
        # Load model from XML string (stateless!)
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)

        # Renderer for observations (224x224 for VLA models)
        self.renderer = mujoco.Renderer(self.model, height=224, width=224)

        # Store initial state
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()

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
            action: 8-dim action vector (7 joints + gripper)

        Returns:
            Observation after step
        """
        # Convert to numpy array
        action_array = np.array(action, dtype=np.float32)

        # Apply action to actuators (truncate or pad as needed)
        num_actuators = self.model.nu
        if len(action_array) >= num_actuators:
            self.data.ctrl[:] = action_array[:num_actuators]
        else:
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
            Dictionary with qpos, qvel, time (as lists for JSON serialization)
        """
        return {
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "time": float(self.data.time),
        }
