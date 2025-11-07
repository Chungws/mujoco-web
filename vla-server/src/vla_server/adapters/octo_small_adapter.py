"""
Octo-Small VLA adapter

Model-specific preprocessing/postprocessing for Octo-Small 27M (JAX-based)
Octo model: https://octo-models.github.io/
HuggingFace: rail-berkeley/octo-small
"""

from typing import Any

import jax
import numpy as np

from .base import VLAModelAdapter


class OctoSmallAdapter(VLAModelAdapter):
    """
    Adapter for Octo-Small 27M model (JAX-based)

    Model specifications:
    - Input image size: 256x256 RGB
    - Action space: 7-dim (no gripper control)
    - Control frequency: 5 Hz (RT-1/Octo standard)
    - Model size: 27M parameters
    - Framework: JAX

    Note:
        Octo is a JAX-based model, not PyTorch. This adapter uses the
        official octo-models/octo library for model loading and inference.
    """

    def __init__(self):
        """Initialize Octo-Small adapter"""
        self.model = None
        self.device = None
        self.model_loaded = False

    def load_model(self, model_id: str, device: str, cache_dir: str) -> None:
        """
        Load Octo-Small model from HuggingFace

        Args:
            model_id: Model identifier (should be "octo-small")
            device: Device specification (JAX uses "cpu", "cuda", or "tpu")
            cache_dir: Directory for model cache

        Raises:
            ValueError: If model_id is not "octo-small"
            RuntimeError: If model loading fails

        Note:
            JAX device management is different from PyTorch.
            - "cuda" → uses GPU if available
            - "cpu" → forces CPU
            - "auto" → lets JAX decide (default)

            We use octo-small-1.5 (latest version) from HuggingFace.
        """
        if model_id != "octo-small":
            raise ValueError(f"Expected model_id='octo-small', got '{model_id}'")

        try:
            from octo.model.octo_model import OctoModel

            # Load from HuggingFace Hub
            # Use octo-small-1.5 (latest version)
            # See: https://github.com/octo-models/octo/blob/main/examples/01_inference_pretrained.ipynb
            hub_path = "hf://rail-berkeley/octo-small-1.5"

            self.model = OctoModel.load_pretrained(hub_path)
            self.device = device
            self.model_loaded = True

        except Exception as e:
            raise RuntimeError(f"Failed to load Octo-Small model: {e!s}") from e

    def preprocess_observation(self, obs: dict[str, Any]) -> dict[str, Any]:
        """
        Preprocess observation to Octo-Small format

        Args:
            obs: Standard observation dict:
                - image: (H, W, 3) RGB numpy array, uint8
                - qpos: (n,) joint positions
                - qvel: (n,) joint velocities

        Returns:
            Preprocessed observation:
                - image_primary: (1, 1, 256, 256, 3) RGB array (batch, time, H, W, C)
                - timestep_pad_mask: (1, 1) boolean mask

        Raises:
            ValueError: If required keys are missing or image format is invalid

        Note:
            Octo expects:
            - 256x256 images (not 224x224)
            - Batch and time dimensions added
            - Image as "image_primary" key
            - Timestep padding mask
        """
        # Validate required keys
        required_keys = {"image", "qpos", "qvel"}
        missing_keys = required_keys - set(obs.keys())
        if missing_keys:
            raise ValueError(f"Missing required keys: {missing_keys}")

        # Validate image
        image = obs["image"]
        if not isinstance(image, np.ndarray):
            raise ValueError("image must be numpy array")
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image must be (H, W, 3), got {image.shape}")

        # Resize image to 256x256 if needed
        if image.shape[:2] != (256, 256):
            from PIL import Image

            pil_image = Image.fromarray(image)
            pil_image_resized = pil_image.resize((256, 256), Image.BILINEAR)
            image_resized = np.array(pil_image_resized)
        else:
            image_resized = image

        # Add batch and time dimensions: (256, 256, 3) → (1, 1, 256, 256, 3)
        image_with_dims = image_resized[np.newaxis, np.newaxis, ...]

        return {
            "image_primary": image_with_dims,
            "timestep_pad_mask": np.array([[True]]),
        }

    def preprocess_instruction(self, instruction: str) -> str:
        """
        Preprocess text instruction to Octo-Small format

        Args:
            instruction: Natural language instruction (e.g., "Pick up the red cube")

        Returns:
            Same instruction (Octo uses standard text encoding)

        Raises:
            ValueError: If instruction is empty

        Note:
            Octo-Small processes text instructions internally,
            so no preprocessing is needed here.
        """
        if not instruction or not instruction.strip():
            raise ValueError("instruction cannot be empty")

        return instruction

    def predict(self, obs: Any, instruction: Any) -> Any:
        """
        Run Octo-Small inference

        Args:
            obs: Preprocessed observation (from preprocess_observation)
            instruction: Preprocessed instruction (from preprocess_instruction)

        Returns:
            Raw model output (numpy array with action)

        Raises:
            RuntimeError: If model is not loaded

        Note:
            - Octo uses JAX for inference
            - Returns action directly as numpy array
            - Model outputs 7-dim action (no gripper)
            - Actions are normalized to [-1, 1] range
        """
        if not self.model_loaded:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        # Create task (language-conditioned)
        # See: https://github.com/octo-models/octo/blob/main/examples/01_inference_pretrained.ipynb
        task = self.model.create_tasks(texts=[instruction])

        # Run inference
        # Note: We don't use unnormalization_statistics to keep actions in normalized range [-1, 1]
        # This is consistent with other VLA adapters
        action = self.model.sample_actions(
            obs,
            task,
            rng=jax.random.PRNGKey(0),
        )

        return action

    def postprocess_action(self, raw_action: Any) -> list[float]:
        """
        Postprocess Octo-Small output to standard 8-dim action

        Args:
            raw_action: Raw model output (numpy array or JAX array)

        Returns:
            8-dim action list: [j1, j2, j3, j4, j5, j6, j7, gripper]

        Raises:
            ValueError: If raw_action is not a numpy/JAX array

        Note:
            - Octo-Small outputs 7-dim action (no gripper control)
            - We pad with 0.0 for gripper to get standard 8-dim format
            - Action values are in normalized range [-1, 1]
            - Output shape from model: (batch, action_chunk, action_dim)
        """
        # Convert JAX array to numpy if needed
        if hasattr(raw_action, "__array__"):
            action_np = np.array(raw_action)
        elif isinstance(raw_action, np.ndarray):
            action_np = raw_action
        else:
            raise ValueError("raw_action must be numpy array or JAX array")

        # Remove batch and action_chunk dimensions if present
        # Expected shape: (1, action_chunk, 7) → (7,)
        while action_np.ndim > 1:
            action_np = action_np[0]

        # Octo outputs 7-dim (no gripper), pad with 0.0
        if len(action_np) == 7:
            action_np = np.append(action_np, 0.0)
        elif len(action_np) != 8:
            raise ValueError(f"Expected 7-dim or 8-dim action, got {len(action_np)}-dim")

        return action_np.tolist()
