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

    def __init__(self):
        """Initialize adapter with common state"""
        self.model_id: str | None = None
        self.device: str | None = None
        self.model_loaded: bool = False

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
        Postprocess model output to standardized action format

        Args:
            raw_action: Raw model output (from predict)

        Returns:
            Action list with variable dimensions (model-specific)
            - Robot arm models: typically 7-8 dim [j1, ..., j7, gripper]
            - Humanoid models: typically 20-30 dim [j1, ..., j_n]
            - Dimension determined by model architecture and target robot

        Note:
            - MuJoCo environment automatically handles dimension mismatch:
              * If len(action) > num_actuators: truncates to model.nu
              * If len(action) < num_actuators: pads remaining with zeros
            - Models should normalize actions to appropriate range (typically [-1, 1])
        """
        pass
