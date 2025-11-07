"""
Custom exceptions for VLA Arena backend

Following FastAPI patterns: Service layer raises domain exceptions,
Router layer catches and converts to HTTPException.
"""


class EpisodeRepositoryError(Exception):
    """Base exception for episode repository operations"""

    pass


class EpisodeNotFoundError(EpisodeRepositoryError):
    """Raised when episode is not found in MongoDB"""

    def __init__(self, episode_id: str):
        self.episode_id = episode_id
        super().__init__(f"Episode not found: {episode_id}")


class EpisodeDatabaseError(EpisodeRepositoryError):
    """Raised when MongoDB operation fails"""

    def __init__(self, operation: str, error: Exception):
        self.operation = operation
        self.original_error = error
        super().__init__(f"Database error during {operation}: {error!s}")


class EpisodeValidationError(EpisodeRepositoryError):
    """Raised when episode data is invalid"""

    def __init__(self, episode_id: str, error: Exception):
        self.episode_id = episode_id
        self.original_error = error
        super().__init__(f"Invalid episode data for {episode_id}: {error!s}")
