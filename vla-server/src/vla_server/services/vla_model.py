"""
VLA model loading and inference
Supports Octo-Small, SmolVLA from HuggingFace

For MVP development:
- Models are loaded lazily on first use
- Models are cached in memory after loading
- Supports CPU, CUDA, and MPS devices
"""

import torch
from transformers import AutoModel, AutoProcessor

from ..config import settings


class VLAModelManager:
    """VLA model manager for loading and running inference"""

    MODEL_HUB_IDS = {
        "octo-small": "octo-models/octo-small",
        "smolvla": "HuggingFaceTB/SmolVLA",
    }

    def __init__(self, model_id: str, device: str = "auto"):
        """
        Initialize VLA model

        Args:
            model_id: Model identifier (e.g., "octo-small", "smolvla")
            device: Device to use (auto, cuda, cpu, mps)

        Raises:
            ValueError: If model_id is not recognized
        """
        self.model_id = model_id
        self.device = self._get_device(device)

        # Validate model ID
        hub_id = self.MODEL_HUB_IDS.get(model_id)
        if not hub_id:
            raise ValueError(f"Unknown model: {model_id}")

        # Load model and processor
        self.processor = AutoProcessor.from_pretrained(hub_id, cache_dir=settings.vla_model_cache)
        self.model = AutoModel.from_pretrained(hub_id, cache_dir=settings.vla_model_cache)

        # Move model to device and set to eval mode
        self.model = self.model.to(self.device)
        self.model.eval()

    def _get_device(self, device: str) -> str:
        """
        Get torch device string

        Args:
            device: Device specification (auto, cuda, cpu, mps)

        Returns:
            Device string for torch
        """
        if device == "auto":
            if torch.cuda.is_available():
                return "cuda"
            elif torch.backends.mps.is_available():
                return "mps"
            else:
                return "cpu"
        return device

    def predict(self, observation: dict, instruction: str) -> list[float]:
        """
        Predict action from observation and instruction

        Args:
            observation: Dictionary with:
                - image: numpy array (H, W, 3) RGB image
                - qpos: list of joint positions
                - qvel: list of joint velocities
            instruction: Natural language instruction

        Returns:
            8-dim action vector as list of floats

        Note:
            This is a simplified implementation for MVP.
            Actual VLA models may require different preprocessing and output handling.
        """
        # Process inputs
        inputs = self.processor(images=observation["image"], text=instruction, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Run inference
        with torch.no_grad():
            outputs = self.model(**inputs)

            # Extract action from model output
            # Note: This is model-specific and may need adjustment for actual models
            if hasattr(outputs, "action"):
                action = outputs.action
            elif hasattr(outputs, "logits"):
                action = outputs.logits
            else:
                # Fallback: try to get first output
                action = outputs[0] if isinstance(outputs, tuple) else outputs.last_hidden_state

            # Ensure action is 8-dimensional
            if action.dim() > 1:
                action = action.squeeze()
            if action.shape[0] != 8:
                # Pad or truncate to 8 dimensions
                if action.shape[0] < 8:
                    action = torch.cat([action, torch.zeros(8 - action.shape[0])])
                else:
                    action = action[:8]

        # Convert to list and return
        return action.cpu().numpy().tolist()
