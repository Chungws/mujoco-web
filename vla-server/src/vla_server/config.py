"""
Configuration for VLA Server
Uses Pydantic Settings for environment variable management
"""

from pathlib import Path

from pydantic_settings import BaseSettings, SettingsConfigDict

# Root .env file location (mujoco-web-vla/.env)
ROOT_ENV_FILE = Path(__file__).parent.parent.parent.parent.parent / ".env"


class Settings(BaseSettings):
    """VLA Server settings"""

    model_config = SettingsConfigDict(env_prefix="VLA_", env_file=str(ROOT_ENV_FILE))

    # Server
    host: str = "0.0.0.0"
    port: int = 8001
    reload: bool = True

    # Models
    mujoco_model_path: str = "./models"
    vla_model_cache: str = "./model_cache"
    default_vla_model: str = "octo-small"
    device: str = "auto"  # auto (cuda → mps → cpu), cuda, cpu, mps

    # Execution - based on RT-1/Octo standards
    control_frequency: float = 5.0  # Hz (RT-1/Octo standard: 3-5 Hz)
    max_episode_seconds: float = 15.0  # seconds (typical task duration)

    # Logging
    log_level: str = "INFO"


settings = Settings()
