"""
Dependency injection for FastAPI

Provides shared instances of services and clients.
"""

import logging

from vlaarena_backend.config import get_model_endpoints
from vlaarena_backend.services.vla_client import VLAServiceClient

logger = logging.getLogger(__name__)

# Global VLA client instance (initialized at startup)
_vla_client: VLAServiceClient | None = None


def initialize_vla_client():
    """
    Initialize VLA service client at application startup

    Loads model endpoints from config/models.yaml and creates
    HTTP client for communicating with VLA servers.
    """
    global _vla_client

    try:
        endpoints = get_model_endpoints()
        _vla_client = VLAServiceClient(model_endpoints=endpoints)
        logger.info(f"VLA client initialized with {len(endpoints)} endpoints")
    except Exception as e:
        logger.error(f"Failed to initialize VLA client: {e}")
        raise


def get_vla_client() -> VLAServiceClient:
    """
    Get VLA service client instance (for dependency injection)

    Returns:
        VLAServiceClient instance

    Raises:
        RuntimeError: If client not initialized (call initialize_vla_client first)
    """
    if _vla_client is None:
        raise RuntimeError("VLA client not initialized. Call initialize_vla_client() at startup.")
    return _vla_client
