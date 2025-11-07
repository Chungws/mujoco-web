"""
Octo-Small VLA adapter

Model-specific preprocessing/postprocessing for Octo-Small 27M
Octo model: https://octo-models.github.io/
HuggingFace: openvla/octo-small
"""

from typing import Any

import numpy as np

from .base import VLAModelAdapter


class OctoSmallAdapter(VLAModelAdapter):
    """
    Adapter for Octo-Small 27M model

    Model specifications:
    - Input image size: 224x224 RGB
    - Action space: 7-dim (no gripper control)
    - Control frequency: 5 Hz (RT-1/Octo standard)
    - Model size: 27M parameters
    """

    def __init__(self):
        """Initialize Octo-Small adapter"""
        self.model = None
        self.processor = None
        self.device = None
        self.model_loaded = False

    def load_model(self, model_id: str, device: str, cache_dir: str) -> None:
        """
        Load Octo-Small model from HuggingFace

        Args:
            model_id: Model identifier (should be "octo-small")
            device: PyTorch device (cuda, mps, cpu)
            cache_dir: Directory for model cache

        Raises:
            ValueError: If model_id is not "octo-small"
            RuntimeError: If model loading fails
        """
        if model_id != "octo-small":
            raise ValueError(f"Expected model_id='octo-small', got '{model_id}'")

        try:
            import torch
            from transformers import AutoModel, AutoProcessor

            hub_id = "openvla/octo-small"

            self.device = device
            self.processor = AutoProcessor.from_pretrained(
                hub_id, cache_dir=cache_dir, trust_remote_code=True
            )

            # Load model with appropriate dtype based on device
            dtype = torch.float16 if device == "cuda" else torch.float32
            self.model = AutoModel.from_pretrained(
                hub_id,
                cache_dir=cache_dir,
                torch_dtype=dtype,
                trust_remote_code=True,
            )

            self.model = self.model.to(device)
            self.model.eval()
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
                - image: (224, 224, 3) RGB numpy array
                - qpos: (n,) joint positions
                - qvel: (n,) joint velocities (not used by Octo but kept for consistency)

        Raises:
            ValueError: If required keys are missing or image format is invalid

        Note:
            Octo expects 224x224 images. If input image is different size,
            it will be resized using PIL.Image.resize with BILINEAR interpolation.
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

        # Resize image to 224x224 if needed
        if image.shape[:2] != (224, 224):
            from PIL import Image

            pil_image = Image.fromarray(image)
            pil_image_resized = pil_image.resize((224, 224), Image.BILINEAR)
            image_resized = np.array(pil_image_resized)
        else:
            image_resized = image

        return {
            "image": image_resized,
            "qpos": obs["qpos"],
            "qvel": obs["qvel"],
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
            Octo-Small uses standard text tokenization via AutoProcessor,
            so no preprocessing is needed here. The processor will handle
            tokenization during inference.
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
            Raw model output (torch.Tensor with action)

        Raises:
            RuntimeError: If model is not loaded

        Note:
            - Runs model in eval mode with torch.no_grad()
            - Uses AutoProcessor to process inputs
            - Returns raw model output (will be postprocessed separately)
        """
        if not self.model_loaded:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        import torch

        # Process inputs using AutoProcessor
        inputs = self.processor(images=obs["image"], text=instruction, return_tensors="pt")

        # Move inputs to correct device
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Run inference
        with torch.no_grad():
            outputs = self.model(**inputs)
            action = outputs.action  # Model-specific output format

        return action

    def postprocess_action(self, raw_action: Any) -> list[float]:
        """
        Postprocess Octo-Small output to standard 8-dim action

        Args:
            raw_action: Raw model output (torch.Tensor)

        Returns:
            8-dim action list: [j1, j2, j3, j4, j5, j6, j7, gripper]

        Raises:
            ValueError: If raw_action is not a torch.Tensor

        Note:
            - Octo-Small outputs 7-dim action (no gripper control)
            - We pad with 0.0 for gripper to get standard 8-dim format
            - Action values are already in normalized range [-1, 1]
        """
        import torch

        if not isinstance(raw_action, torch.Tensor):
            raise ValueError("raw_action must be torch.Tensor")

        # Convert to numpy
        action_np = raw_action.cpu().numpy()

        # Remove batch dimension if present
        if action_np.ndim > 1:
            action_np = action_np[0]

        # Octo outputs 7-dim (no gripper), pad with 0.0
        if len(action_np) == 7:
            action_np = np.append(action_np, 0.0)
        elif len(action_np) != 8:
            raise ValueError(f"Expected 7-dim or 8-dim action, got {len(action_np)}-dim")

        return action_np.tolist()
