"""
Logging configuration for worker

Re-exports shared logging functions for worker use.

Note: Package-level logger should be configured in main.py using:
    logger = setup_logging("vlaarena_worker")

Child modules can use standard logging:
    import logging
    logger = logging.getLogger(__name__)  # Inherits parent configuration
"""

from vlaarena_shared.logging_config import setup_logging

__all__ = ["setup_logging"]
