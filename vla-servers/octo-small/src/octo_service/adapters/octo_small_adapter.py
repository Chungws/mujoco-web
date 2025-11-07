"""
Octo-Small VLA adapter

Implements VLAModelAdapter for Octo-Small (27M parameters) model.
Handles JAX-based inference with action chunking and observation windowing.
"""

from typing import Any

import jax
import numpy as np
from PIL import Image
from vla_server_base import VLAModelAdapter


class OctoSmallAdapter(VLAModelAdapter):
    """
    Adapter for Octo-Small (27M) VLA model

    Key features:
    - Image input: 256x256 RGB (resized from MuJoCo 224x224)
    - Action output: 7-dim (end-effector control)
    - Action chunking: Predicts 4 actions, uses first (receding horizon)
    - Observation window: 2 timesteps (pad mask for first step)
    - Framework: JAX 0.4.20

    Reference:
    - Model: hf://rail-berkeley/octo-small-1.5
    - GitHub: https://github.com/octo-models/octo
    """

    def __init__(self):
        """Initialize Octo-Small adapter"""
        self.model = None
        self.model_id = None
        self.device = None
        self.rng_key = jax.random.PRNGKey(0)

    def load_model(self, model_id: str, device: str, cache_dir: str) -> None:
        """
        Load Octo-Small model from HuggingFace

        Args:
            model_id: Model identifier (e.g., "octo-small")
            device: Device type ("cpu", "cuda", or "auto")
            cache_dir: Directory for model cache

        Raises:
            ValueError: If model_id is not supported
            RuntimeError: If model loading fails

        Note:
            JAX automatically detects and uses GPUs when available.
            Device configuration happens via JAX backend selection.
        """
        if not model_id:
            raise ValueError("model_id cannot be empty")

        # Import here to avoid loading at module level
        from octo.model.octo_model import OctoModel

        try:
            # Detect available JAX devices
            detected_device = self._detect_jax_device()

            # Load from HuggingFace
            # Octo uses "octo-small-1.5" for the latest version
            hf_path = "hf://rail-berkeley/octo-small-1.5"

            self.model = OctoModel.load_pretrained(hf_path)
            self.model_id = model_id
            self.device = detected_device

        except Exception as e:
            raise RuntimeError(f"Failed to load Octo-Small model: {e}") from e

    def _detect_jax_device(self) -> str:
        """
        Detect available JAX device

        Returns:
            Device type: "cuda" if GPU available, "cpu" otherwise

        Note:
            JAX automatically uses GPU when available.
            This method just reports what JAX is using.
        """
        try:
            devices = jax.devices()
            if devices:
                platform = devices[0].platform
                if platform == "gpu":
                    return "cuda"
                return platform
        except Exception:
            pass
        return "cpu"

    def preprocess_observation(self, obs: dict[str, Any]) -> dict[str, Any]:
        """
        Preprocess observation for Octo-Small

        Args:
            obs: Standard observation dict
                - image: (H, W, 3) RGB numpy array, uint8
                - qpos: (n,) joint positions
                - qvel: (n,) joint velocities

        Returns:
            Octo-formatted observation:
                - image_primary: (1, 1, 256, 256, 3) float32 [0, 1]
                - timestep_pad_mask: (1, 1) bool

        Raises:
            ValueError: If observation format is invalid

        Note:
            - Octo expects 256x256 images (not 224x224)
            - Adds batch and time dimensions: [batch=1, time=1, H, W, C]
            - Normalizes to [0, 1] range
            - timestep_pad_mask: True for valid observations
        """
        # Validate input
        required_keys = {"image", "qpos", "qvel"}
        missing_keys = required_keys - set(obs.keys())
        if missing_keys:
            raise ValueError(f"Missing required keys: {missing_keys}")

        image = obs["image"]
        if not isinstance(image, np.ndarray):
            raise ValueError("image must be numpy array")
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image must be (H, W, 3), got {image.shape}")

        # Resize to 256x256 (Octo requirement)
        pil_image = Image.fromarray(image)
        resized_image = pil_image.resize((256, 256), Image.Resampling.BILINEAR)
        resized_array = np.array(resized_image, dtype=np.float32)

        # Normalize to [0, 1]
        if resized_array.dtype == np.uint8 or resized_array.max() > 1.0:
            resized_array = resized_array / 255.0

        # Add batch and time dimensions: [1, 1, 256, 256, 3]
        image_batched = resized_array[np.newaxis, np.newaxis, ...]

        # timestep_pad_mask: True for valid observations
        # For single timestep, always True
        timestep_pad_mask = np.array([[True]], dtype=bool)

        return {
            "image_primary": image_batched,
            "timestep_pad_mask": timestep_pad_mask,
        }

    def preprocess_instruction(self, instruction: str) -> dict[str, Any]:
        """
        Preprocess instruction for Octo-Small

        Args:
            instruction: Natural language instruction

        Returns:
            Task dictionary for model.create_tasks()

        Raises:
            ValueError: If instruction is empty

        Note:
            Returns dict format suitable for model.create_tasks(texts=[...])
        """
        if not instruction or not instruction.strip():
            raise ValueError("instruction cannot be empty")

        # Return as dict for create_tasks
        return {"text": instruction}

    def predict(self, obs: Any, instruction: Any) -> Any:
        """
        Run Octo-Small inference

        Args:
            obs: Preprocessed observation (from preprocess_observation)
            instruction: Preprocessed instruction (from preprocess_instruction)

        Returns:
            Raw action tensor: (1, pred_horizon=4, action_dim=7)

        Raises:
            RuntimeError: If model not loaded or inference fails

        Note:
            - Octo predicts 4 actions (action chunking)
            - We use receding horizon: only first action
            - Action shape: [batch=1, pred_horizon=4, action_dim=7]
        """
        if self.model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        try:
            # Create task from instruction
            task = self.model.create_tasks(texts=[instruction["text"]])

            # Sample actions
            # Returns: (batch, pred_horizon, action_dim)
            actions = self.model.sample_actions(
                obs,
                task,
                rng=self.rng_key,
            )

            # Update RNG key for next prediction
            self.rng_key, _ = jax.random.split(self.rng_key)

            return actions

        except Exception as e:
            raise RuntimeError(f"Inference failed: {e}") from e

    def postprocess_action(self, raw_action: Any) -> list[float]:
        """
        Postprocess Octo action to standard 8-dim format

        Args:
            raw_action: Raw action from predict()
                Shape: (1, pred_horizon=4, action_dim=7)

        Returns:
            8-dim action list: [j1, j2, j3, j4, j5, j6, j7, gripper]

        Raises:
            ValueError: If action format is invalid

        Note:
            - Extracts first action from chunked predictions (receding horizon)
            - Removes batch dimension: (1, 4, 7) → (7,)
            - Pads gripper dimension: (7,) → (8,)
            - Gripper set to 0.0 (Octo doesn't predict gripper)
        """
        # Validate shape
        if not hasattr(raw_action, "shape"):
            raise ValueError("raw_action must have shape attribute (JAX array)")

        # Expected: (batch=1, pred_horizon=4, action_dim=7)
        if len(raw_action.shape) != 3:
            raise ValueError(f"Expected 3D action, got shape {raw_action.shape}")

        batch_size, _pred_horizon, _action_dim = raw_action.shape

        if batch_size != 1:
            raise ValueError(f"Expected batch_size=1, got {batch_size}")

        # Extract first action (receding horizon control)
        # Shape: (1, 4, 7) → (7,)
        first_action = raw_action[0, 0, :]

        # Convert JAX array to numpy
        action_np = np.array(first_action, dtype=np.float32)

        # Validate action dimension
        if len(action_np) != 7:
            raise ValueError(f"Expected 7-dim action, got {len(action_np)}")

        # Pad gripper dimension (Octo outputs 7-dim, we need 8-dim)
        # Standard format: [j1, j2, j3, j4, j5, j6, j7, gripper]
        action_8dim = np.append(action_np, 0.0)

        return action_8dim.tolist()
