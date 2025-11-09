"""
Backend configuration

Loads VLA model endpoints from config/models.yaml and provides
them to services that need to communicate with VLA servers.
"""

import logging
from pathlib import Path
from typing import Any

import yaml

logger = logging.getLogger(__name__)

# Path to models.yaml (project root/config/models.yaml)
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
MODELS_CONFIG_PATH = PROJECT_ROOT / "config" / "models.yaml"


def load_models_config() -> dict[str, Any]:
    """
    Load models configuration from YAML file

    Returns:
        Dictionary containing models configuration

    Raises:
        FileNotFoundError: If config/models.yaml not found
        yaml.YAMLError: If YAML parsing fails
    """
    if not MODELS_CONFIG_PATH.exists():
        raise FileNotFoundError(f"Models config not found: {MODELS_CONFIG_PATH}")

    with open(MODELS_CONFIG_PATH) as f:
        config = yaml.safe_load(f)

    logger.info(f"Loaded models config from {MODELS_CONFIG_PATH}")
    return config


def get_model_endpoints() -> dict[str, str]:
    """
    Get mapping of model_id -> base_url from models.yaml

    Only includes models with status="active"

    Returns:
        Dictionary mapping model_id to base_url
        Example: {"openvla-7b": "http://localhost:8001",
                  "octo-base": "http://localhost:8001"}
    """
    config = load_models_config()
    models = config.get("models", [])

    endpoints = {}
    for model in models:
        if model.get("status") == "active":
            model_id = model["id"]
            base_url = model["base_url"]
            endpoints[model_id] = base_url

    logger.info(f"Loaded {len(endpoints)} active model endpoints")
    return endpoints
