"""
Shared configuration settings (shared between backend and worker)
"""

from typing import List

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables

    Usage:
        from vlaarena_shared.config import settings

        db_url = settings.postgres_uri
    """

    model_config = SettingsConfigDict(
        env_file=[
            "../.env",
            ".env",
        ],  # Parent dir first, then current (current overrides)
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # Database URLs
    postgres_uri: str = "postgresql+asyncpg://postgres:postgres@localhost:5432/vlaarena"

    # CORS settings (backend only)
    cors_origins: str = "http://localhost:3000,http://127.0.0.1:3000"

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string"""
        return [
            origin.strip() for origin in self.cors_origins.split(",") if origin.strip()
        ]

    # Model configuration
    # Local dev: ../config/models.yaml (relative to backend/ or worker/)
    # Production: /app/config/models.yaml (absolute path in Docker)
    models_config_path: str = "../config/models.yaml"

    # Worker settings
    worker_interval_minutes: int = 60  # Run worker every N minutes
    worker_timezone: str = "UTC"

    # LLM API timeouts (seconds)
    # Note: CPU inference can take 30-60s per request, so read timeout should be higher
    llm_connect_timeout: int = 5
    llm_read_timeout: int = 90
    llm_write_timeout: int = 5
    llm_pool_timeout: int = 5

    # LLM API retry settings
    llm_retry_attempts: int = 3
    llm_retry_backoff_base: float = 1.0  # Base delay in seconds (1s, 2s, 4s)

    # LLM Mock Mode (for development and testing)
    use_mock_llm: bool = (
        False  # Set to True to use mock LLM client instead of real API calls
    )

    # Battle settings
    max_follow_ups: int = 5  # Maximum 5 follow-ups (6 total messages)

    # Leaderboard settings
    min_votes_for_leaderboard: int = 5  # Minimum votes to appear on leaderboard

    # ELO settings
    initial_elo: int = 1500
    k_factor: int = 32

    # PostgreSQL connection pool settings (Backend API)
    postgres_pool_size: int = 10
    postgres_max_overflow: int = 20  # Total max: 10 + 20 = 30 connections

    # PostgreSQL connection pool settings (Worker)
    worker_pool_size: int = 2
    worker_max_overflow: int = 3  # Total max: 2 + 3 = 5 connections
    worker_pool_timeout: int = 10  # Connection timeout in seconds


# Singleton instance
settings = Settings()


# Multi-assistant system prompt
# Used when assembling session-wide conversation history for LLM requests
MULTI_ASSISTANT_SYSTEM_PROMPT = (
    "You are participating in a multi-model comparison system. "
    "Multiple AI assistants are responding to the same conversation. "
    "Each 'assistant' message may be from a different AI model. "
    "Continue the conversation naturally without mentioning this setup to the user."
)
