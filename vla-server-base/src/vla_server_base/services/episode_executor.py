"""
Episode execution service for VLA models

Handles full episode generation: environment setup, VLA inference loop, and state recording.
"""

import time
from typing import Any

from vla_server_base.adapters.base import VLAModelAdapter
from vla_server_base.services.mujoco_env import MuJoCoEnvironment


class EpisodeExecutor:
    """Execute full episodes with VLA models in MuJoCo simulation"""

    def __init__(self, adapter: VLAModelAdapter, max_steps: int = 75):
        """
        Initialize episode executor

        Args:
            adapter: VLA model adapter (must be loaded)
            max_steps: Maximum steps per episode (default: 75 = 15s @ 5Hz)
        """
        self.adapter = adapter
        self.max_steps = max_steps

    def execute_episode(
        self,
        instruction: str,
        xml_string: str,
    ) -> dict[str, Any]:
        """
        Execute full episode with VLA model

        Flow:
        1. Create MuJoCo environment from XML
        2. Reset and get initial observation
        3. Loop: predict action → step → record state (max_steps times)
        4. Return episode data (actions, states, duration_ms)

        Args:
            instruction: Natural language instruction
            xml_string: Complete MuJoCo XML model string

        Returns:
            Dictionary containing:
            - actions: List of 8-dim action vectors
            - states: List of state dicts (qpos, qvel, time)
            - duration_ms: Execution duration in milliseconds
        """
        start_time = time.time()

        # 1. Create environment from XML
        env = MuJoCoEnvironment(xml_string)

        # 2. Reset environment
        obs = env.reset()

        # 3. Execute episode loop
        actions = []
        states = []

        for _ in range(self.max_steps):
            # Preprocess observation and instruction
            processed_obs = self.adapter.preprocess_observation(obs)
            processed_instruction = self.adapter.preprocess_instruction(instruction)

            # Predict action
            raw_action = self.adapter.predict(processed_obs, processed_instruction)

            # Postprocess action
            action = self.adapter.postprocess_action(raw_action)

            # Step environment
            obs = env.step(action)

            # Record action and state
            actions.append(action)
            states.append(env.get_state())

            # Optional: Add termination condition (e.g., task completion)
            # For MVP, we just run for max_steps

        # 5. Calculate duration
        duration_ms = int((time.time() - start_time) * 1000)

        return {
            "actions": actions,
            "states": states,
            "duration_ms": duration_ms,
        }
