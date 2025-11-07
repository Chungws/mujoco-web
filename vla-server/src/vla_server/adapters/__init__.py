"""
VLA adapter factory

Provides factory function to get appropriate adapter based on model_id
"""

from .base import VLAModelAdapter
from .mock_adapter import MockVLAAdapter
from .octo_small_adapter import OctoSmallAdapter


def get_adapter(model_id: str) -> VLAModelAdapter:
    """
    Get VLA adapter for given model

    Args:
        model_id: Model identifier (e.g., "mock", "octo-small", "smolvla")

    Returns:
        Appropriate VLAModelAdapter instance

    Raises:
        ValueError: If model_id is not recognized

    Example:
        >>> adapter = get_adapter("mock")
        >>> adapter.load_model("mock", "cpu", "./cache")
    """
    # Adapter registry
    adapters = {
        "mock": MockVLAAdapter,
        "octo-small": OctoSmallAdapter,
        # Future: "smolvla": SmolVLAAdapter,
    }

    adapter_class = adapters.get(model_id)
    if not adapter_class:
        available = list(adapters.keys())
        raise ValueError(f"Unknown model_id: '{model_id}'. Available models: {available}")

    return adapter_class()


__all__ = [
    "MockVLAAdapter",
    "OctoSmallAdapter",
    "VLAModelAdapter",
    "get_adapter",
]
