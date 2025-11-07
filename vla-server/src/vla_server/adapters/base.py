"""
Abstract base class for VLA model adapters

Defines interface for model-specific preprocessing/postprocessing
Each VLA model (Octo-Small, SmolVLA) implements this interface
"""

from abc import ABC, abstractmethod
from typing import Any


class VLAModelAdapter(ABC):
    """
    Abstract base class for VLA model adapters

    Each adapter handles:
    - Model loading and initialization
    - Observation preprocessing (image resizing, normalization)
    - Instruction preprocessing (tokenization, encoding)
    - Model inference
    - Action postprocessing (standardize to 8-dim format)
    """

    @abstractmethod
    def load_model(self, model_id: str, device: str, cache_dir: str) -> None:
        """
        Load VLA model from HuggingFace or local cache

        Args:
            model_id: Model identifier (e.g., "octo-small", "smolvla")
            device: PyTorch device (cuda, mps, cpu)
            cache_dir: Directory for model cache

        Raises:
            ValueError: If model_id is not supported
            RuntimeError: If model loading fails
        """
        pass

    @abstractmethod
    def preprocess_observation(self, obs: dict[str, Any]) -> Any:
        """
        Preprocess observation to model-specific format

        Args:
            obs: Standard observation dictionary:
                - image: (H, W, 3) RGB numpy array, uint8
                - qpos: (n,) joint positions, float32
                - qvel: (n,) joint velocities, float32

        Returns:
            Model-specific observation format

        Note:
            Different models expect different image sizes:
            - Octo: 224x224
            - SmolVLA: 256x256
        """
        pass

    @abstractmethod
    def preprocess_instruction(self, instruction: str) -> Any:
        """
        Preprocess text instruction to model-specific format

        Args:
            instruction: Natural language instruction (e.g., "Pick up the red cube")

        Returns:
            Model-specific instruction format (tokenized, encoded, etc.)
        """
        pass

    @abstractmethod
    def predict(self, obs: Any, instruction: Any) -> Any:
        """
        Run model inference

        Args:
            obs: Preprocessed observation (from preprocess_observation)
            instruction: Preprocessed instruction (from preprocess_instruction)

        Returns:
            Raw model output (model-specific format)

        Note:
            This method should handle:
            - Moving inputs to correct device
            - Running model in eval mode
            - Disabling gradients (torch.no_grad)
        """
        pass

    @abstractmethod
    def postprocess_action(self, raw_action: Any) -> list[float]:
        """
        Postprocess model output to standard 8-dim action

        Args:
            raw_action: Raw model output (from predict)

        Returns:
            8-dim action list: [j1, j2, j3, j4, j5, j6, j7, gripper]
            - j1-j7: Joint positions/velocities (robot-specific)
            - gripper: Gripper open/close (0.0 = closed, 1.0 = open)

        Note:
            - Some models output 7-dim (no gripper) - pad with 0.0
            - Some models output different action spaces - normalize to [-1, 1]
        """
        pass
