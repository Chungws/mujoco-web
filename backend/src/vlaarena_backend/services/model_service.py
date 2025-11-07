"""
Model configuration management service
"""

import logging
import os
import random
from pathlib import Path

import yaml
from vlaarena_shared.config import settings
from vlaarena_shared.schemas import ModelInfo

logger = logging.getLogger(__name__)


class ModelConfig:
    """
    Single model configuration

    Loaded from config/models.yaml
    """

    def __init__(self, config_dict: dict):
        self.id: str = config_dict["id"]
        self.name: str = config_dict["name"]
        self.model: str = config_dict["model"]
        self.base_url: str = config_dict["base_url"]
        self.api_key_env: str | None = config_dict.get("api_key_env")
        self.organization: str = config_dict["organization"]
        self.license: str = config_dict["license"]
        self.status: str = config_dict.get("status", "active")

    @property
    def api_key(self) -> str | None:
        """
        Resolve API key from environment variable

        Returns:
            API key string or None (for local models)
        """
        if self.api_key_env is None:
            return None
        return os.getenv(self.api_key_env)

    def to_model_info(self) -> ModelInfo:
        """
        Convert to API response schema

        Returns:
            ModelInfo schema for GET /api/models
        """
        return ModelInfo(
            model_id=self.id,
            name=self.name,
            provider=self.organization,
            status=self.status,  # type: ignore
        )


class ModelService:
    """
    Model configuration service

    Loads and manages model configurations from YAML file
    """

    def __init__(self, config_path: str | None = None):
        """
        Initialize model service

        Args:
            config_path: Path to models.yaml (defaults to settings.models_config_path)
        """
        self.config_path = Path(config_path or settings.models_config_path)
        self.models: dict[str, ModelConfig] = {}
        self._load_models()

    def _load_models(self) -> None:
        """
        Load model configurations from YAML file

        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If YAML parsing fails
        """
        if not self.config_path.exists():
            raise FileNotFoundError(f"Model config not found: {self.config_path}")

        logger.info(f"Loading model config from {self.config_path}")

        with open(self.config_path, encoding="utf-8") as f:
            config = yaml.safe_load(f)

        if not config or "models" not in config:
            raise ValueError("Invalid model config: missing 'models' key")

        for model_dict in config["models"]:
            model_config = ModelConfig(model_dict)
            self.models[model_config.id] = model_config

        logger.info(f"Loaded {len(self.models)} models: {list(self.models.keys())}")

    def get_model(self, model_id: str) -> ModelConfig | None:
        """
        Get model configuration by ID

        Args:
            model_id: Model identifier

        Returns:
            ModelConfig or None if not found
        """
        return self.models.get(model_id)

    def get_active_models(self) -> list[ModelConfig]:
        """
        Get all active models

        Returns:
            List of active ModelConfig objects
        """
        return [m for m in self.models.values() if m.status == "active"]

    def list_models(self) -> list[ModelInfo]:
        """
        List all active models (for GET /api/models)

        Returns:
            List of ModelInfo schemas
        """
        active_models = self.get_active_models()
        return [m.to_model_info() for m in active_models]

    def select_models_for_battle(self) -> tuple[ModelConfig, ModelConfig]:
        """
        Select 2 random models for battle

        Strategy: Uniform random selection (all models equal probability)
        Future: Can be upgraded to ELO-based matching, category-based, etc.

        Returns:
            Tuple of (model_a, model_b) where model_a != model_b

        Raises:
            ValueError: If less than 2 active models available
        """
        active_models = self.get_active_models()

        if len(active_models) < 2:
            raise ValueError(
                f"Need at least 2 active models for battle, found {len(active_models)}"
            )

        # Random selection without replacement
        model_a, model_b = random.sample(active_models, 2)

        logger.info(f"Selected models for battle: {model_a.id} vs {model_b.id}")

        return model_a, model_b


# Singleton instance
_model_service: ModelService | None = None


def get_model_service() -> ModelService:
    """
    Get singleton ModelService instance

    Returns:
        ModelService instance
    """
    global _model_service
    if _model_service is None:
        _model_service = ModelService()
    return _model_service
