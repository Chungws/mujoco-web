"""
Configuration for VLA Server
Uses Pydantic Settings for environment variable management
"""

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """VLA Server settings"""

    model_config = SettingsConfigDict(env_prefix="VLA_", env_file=".env")

    # Server
    host: str = "0.0.0.0"
    port: int = 8001
    reload: bool = True

    # Models
    mujoco_model_path: str = "./models"
    vla_model_cache: str = "./model_cache"
    default_vla_model: str = "octo-small"
    device: str = "auto"  # auto (cuda → mps → cpu), cuda, cpu, mps

    # Logging
    log_level: str = "INFO"


settings = Settings()
