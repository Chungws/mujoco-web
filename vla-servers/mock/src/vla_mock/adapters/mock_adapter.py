"""
Mock VLA adapter for testing

Provides deterministic, lightweight adapter without real model loading
Useful for testing adapter interface and integration tests
"""

from typing import Any

import numpy as np
from vla_server_base import VLAModelAdapter


class MockVLAAdapter(VLAModelAdapter):
    """
    Mock VLA adapter that returns deterministic actions

    Useful for:
    - Testing adapter interface
    - Integration tests without GPU
    - CI/CD pipeline tests
    - MacBook development without model downloads
    """

    def __init__(self):
        """Initialize mock adapter"""
        self.model_loaded = False
        self.model_id = None
        self.device = None

    def load_model(self, model_id: str, device: str, cache_dir: str) -> None:
        """
        Mock model loading (no actual download)

        Args:
            model_id: Model identifier
            device: PyTorch device
            cache_dir: Cache directory (ignored)
        """
        if not model_id:
            raise ValueError("model_id cannot be empty")

        self.model_id = model_id
        self.device = device
        self.model_loaded = True

    def preprocess_observation(self, obs: dict[str, Any]) -> dict[str, Any]:
        """
        Mock observation preprocessing

        Args:
            obs: Standard observation dict

        Returns:
            Same observation (no transformation)

        Raises:
            ValueError: If required keys missing
        """
        required_keys = {"image", "qpos", "qvel"}
        missing_keys = required_keys - set(obs.keys())
        if missing_keys:
            raise ValueError(f"Missing required keys: {missing_keys}")

        # Validate image shape
        image = obs["image"]
        if not isinstance(image, np.ndarray):
            raise ValueError("image must be numpy array")
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image must be (H, W, 3), got {image.shape}")

        # Return as-is (mock doesn't transform)
        return {
            "image": obs["image"],
            "qpos": obs["qpos"],
            "qvel": obs["qvel"],
        }

    def preprocess_instruction(self, instruction: str) -> str:
        """
        Mock instruction preprocessing

        Args:
            instruction: Natural language instruction

        Returns:
            Same instruction (no transformation)

        Raises:
            ValueError: If instruction is empty
        """
        if not instruction or not instruction.strip():
            raise ValueError("instruction cannot be empty")

        return instruction

    def predict(self, obs: Any, instruction: Any) -> np.ndarray:
        """
        Mock prediction - returns deterministic action

        Args:
            obs: Preprocessed observation
            instruction: Preprocessed instruction

        Returns:
            Deterministic 8-dim action based on instruction hash

        Note:
            Action is deterministic for same instruction (useful for testing)
        """
        if not self.model_loaded:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        # Generate deterministic action based on instruction
        # Use hash for reproducibility
        seed = hash(instruction) % (2**32)
        rng = np.random.RandomState(seed)

        # Generate 8-dim action in [-1, 1] range
        action = rng.uniform(-1.0, 1.0, size=8).astype(np.float32)

        return action

    def postprocess_action(self, raw_action: Any) -> list[float]:
        """
        Mock action postprocessing

        Args:
            raw_action: Raw action (numpy array)

        Returns:
            8-dim action list

        Raises:
            ValueError: If action is not 8-dim
        """
        if not isinstance(raw_action, np.ndarray):
            raise ValueError("raw_action must be numpy array")

        if raw_action.shape != (8,):
            raise ValueError(f"raw_action must be (8,), got {raw_action.shape}")

        # Convert to list
        return raw_action.tolist()
