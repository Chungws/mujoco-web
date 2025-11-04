"""
Shared logging configuration for backend and worker

Logs to stdout for Docker container compatibility.
Use INFO for normal operations, ERROR for failures.
"""

import logging
import os
import sys


def setup_logging(logger_name: str = "vlaarena") -> logging.Logger:
    """
    Setup logging configuration

    Args:
        logger_name: Name of the logger (e.g., "vlaarena_backend", "vlaarena_worker")

    Returns:
        logging.Logger: Configured logger instance

    Log Levels:
        - INFO: Normal operations (requests, processing, updates)
        - ERROR: Failures (database errors, API failures, processing errors)

    Environment Variables:
        LOG_LEVEL: Logging level (default: INFO)
                  Options: DEBUG, INFO, WARNING, ERROR, CRITICAL

    Format:
        [YYYY-MM-DD HH:MM:SS] [LEVEL] message
    """
    # Get log level from environment or default to INFO
    log_level_name = os.getenv("LOG_LEVEL", "INFO").upper()
    log_level = getattr(logging, log_level_name, logging.INFO)

    # Create logger
    logger = logging.getLogger(logger_name)
    logger.setLevel(log_level)

    # Remove existing handlers to avoid duplicates
    logger.handlers = []

    # Create StreamHandler (stdout)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(log_level)

    # Create formatter
    # Format: [2025-01-21 10:30:45] [INFO] Worker started
    formatter = logging.Formatter(
        fmt="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    )
    stream_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(stream_handler)

    # Allow propagation for hierarchical logging
    # Package logger (e.g., "vlaarena_backend") propagates to child loggers
    # Child loggers (e.g., "vlaarena_backend.api.sessions") inherit settings
    logger.propagate = True

    return logger
