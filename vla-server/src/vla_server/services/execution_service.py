"""
Execution service - orchestrates MuJoCo + VLA
Manages episode generation with caching for environments and models
"""

import time

from ..config import settings
from ..schemas.execute import ExecuteRequest, ExecuteResponse, State
from .mujoco_env import MuJoCoEnvironment
from .vla_model import VLAModelManager


class ExecutionService:
    """Service for executing VLA episodes"""

    def __init__(self):
        """Initialize execution service with empty caches"""
        # Lazy loading - models cached here
        self.envs = {}  # robot_id+scene_id → MuJoCoEnvironment
        self.models = {}  # model_id → VLAModelManager

    async def execute(self, request: ExecuteRequest) -> ExecuteResponse:
        """
        Execute VLA episode

        Args:
            request: Execution request with model_id, robot_id, scene_id, instruction

        Returns:
            Episode data with actions and states

        Raises:
            FileNotFoundError: If robot or scene not found
            ValueError: If model not found
        """
        start_time = time.time()

        # 1. Get or create environment
        env = self._get_environment(request.robot_id, request.scene_id)

        # 2. Get or load model
        model = self._get_model(request.model_id)

        # 3. Reset environment
        env.reset()

        # 4. Run episode
        actions = []
        states = []

        # Calculate max steps based on duration and control frequency
        # Example: 15s × 5Hz = 75 steps (more realistic than fixed 50)
        max_steps = int(settings.max_episode_seconds * settings.control_frequency)

        for step in range(max_steps):
            # Get observation
            obs = env.get_observation()

            # VLA inference
            action = model.predict(obs, request.instruction)

            # Step simulation
            env.step(action)

            # Record
            actions.append(action)
            states.append(State(**env.get_state()))

            # TODO: Check termination signal from VLA model
            # Most VLA models use last dimension as terminate signal
            # if len(action) >= 8 and action[7] > 0.5:
            #     break

        # 5. Calculate duration
        duration_ms = int((time.time() - start_time) * 1000)

        # 6. Return response
        return ExecuteResponse(
            actions=actions,
            states=states,
            duration_ms=duration_ms,
            metadata={
                "num_steps": len(actions),
                "max_steps": max_steps,
                "early_termination": len(actions) < max_steps,
            },
        )

    def _get_environment(self, robot_id: str, scene_id: str) -> MuJoCoEnvironment:
        """
        Get or create MuJoCo environment (cached)

        Args:
            robot_id: Robot identifier
            scene_id: Scene identifier

        Returns:
            MuJoCo environment instance

        Raises:
            FileNotFoundError: If robot or scene not found
        """
        key = f"{robot_id}_{scene_id}"
        if key not in self.envs:
            self.envs[key] = MuJoCoEnvironment(robot_id, scene_id)
        return self.envs[key]

    def _get_model(self, model_id: str) -> VLAModelManager:
        """
        Get or load VLA model (cached)

        Args:
            model_id: Model identifier

        Returns:
            VLA model manager instance

        Raises:
            ValueError: If model not found
        """
        if model_id not in self.models:
            self.models[model_id] = VLAModelManager(model_id)
        return self.models[model_id]
