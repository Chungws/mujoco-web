"""
VLA (Vision-Language-Action) execution service

Mock implementation for MVP development.
Simulates VLA model inference and episode generation.
"""

import logging
import random
import time
from typing import Any


logger = logging.getLogger(__name__)


class MockVLAService:
    """
    Mock VLA execution service for MVP development

    Generates fake episodes without actual VLA inference or MuJoCo simulation.
    Used for:
    - Rapid API development and testing
    - Frontend integration before real VLA models are ready
    - CI/CD pipeline testing

    In production, this will be replaced with real VLA inference service.
    """

    # Episode configuration
    MAX_STEPS = 50
    ACTION_DIM = 8
    MIN_STEPS = 20  # Variable length episodes

    def __init__(self):
        """Initialize mock VLA service"""
        # Model-specific seeds for deterministic but different behavior
        self.model_seeds = {
            "openvla-7b": 42,
            "octo-base": 123,
        }
        logger.info("Initialized MockVLAService")

    def generate_episode(
        self,
        model_id: str,
        instruction: str,
        robot_id: str,
        scene_id: str,
    ) -> dict[str, Any]:
        """
        Generate mock episode data

        Args:
            model_id: VLA model identifier (e.g., "openvla-7b")
            instruction: Natural language instruction
            robot_id: Robot identifier (e.g., "widowx")
            scene_id: Scene identifier (e.g., "table")

        Returns:
            Dictionary containing:
            - actions: List of 8-dim action vectors
            - states: List of state dicts (qpos, qvel, time)
            - duration_ms: Execution duration in milliseconds
        """
        logger.info(
            f"Generating episode: model={model_id}, robot={robot_id}, "
            f"scene={scene_id}, instruction={instruction[:50]}..."
        )
        start_time = time.time()

        # Use model-specific seed for deterministic but different behavior
        seed = self.model_seeds.get(model_id, 0)
        # Add instruction hash for variation
        seed += hash(instruction) % 1000
        random.seed(seed)

        # Generate variable length episode (MIN_STEPS to MAX_STEPS)
        num_steps = random.randint(self.MIN_STEPS, self.MAX_STEPS)

        # Generate actions
        actions = self._generate_actions(num_steps)

        # Generate states
        states = self._generate_states(num_steps)

        # Calculate duration (ensure at least 1ms)
        duration_ms = max(1, int((time.time() - start_time) * 1000))

        logger.info(f"Episode generated: {num_steps} steps, duration={duration_ms}ms")

        return {
            "actions": actions,
            "states": states,
            "duration_ms": duration_ms,
        }

    def _generate_actions(self, num_steps: int) -> list[list[float]]:
        """
        Generate mock 8-dim actions

        Action space for robot manipulation:
        - 3D position delta (x, y, z)
        - 3D rotation delta (roll, pitch, yaw)
        - Gripper command (open/close)
        - Reserved/padding

        Args:
            num_steps: Number of actions to generate

        Returns:
            List of 8-dimensional action vectors
        """
        actions = []
        for _ in range(num_steps):
            action = [
                random.uniform(-0.1, 0.1)  # x delta
                for _ in range(self.ACTION_DIM)
            ]
            actions.append(action)
        return actions

    def _generate_states(self, num_steps: int) -> list[dict[str, Any]]:
        """
        Generate mock MuJoCo states

        Args:
            num_steps: Number of states to generate

        Returns:
            List of state dictionaries with qpos, qvel, time
        """
        states = []
        for step in range(num_steps):
            # Mock joint positions and velocities
            # WidowX has ~7 joints
            qpos = [random.uniform(-1.0, 1.0) for _ in range(7)]
            qvel = [random.uniform(-0.5, 0.5) for _ in range(7)]

            state = {
                "qpos": qpos,
                "qvel": qvel,
                "time": step * 0.1,  # 10Hz control rate
            }
            states.append(state)
        return states
