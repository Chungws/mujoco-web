"""
Database connection and session management for worker

Re-exports shared database functions for worker use.
"""

from vlaarena_shared.database import get_worker_db as get_db
from vlaarena_shared.database import worker_engine as engine
from vlaarena_shared.database import worker_session_maker as async_session_maker

__all__ = [
    "async_session_maker",
    "engine",
    "get_db",
]
