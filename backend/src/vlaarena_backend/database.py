"""
Database connection and session management

Re-exports shared database functions for backend use.
"""

from vlaarena_shared.database import backend_engine as engine
from vlaarena_shared.database import backend_session_maker as async_session_maker
from vlaarena_shared.database import (
    create_db_and_tables,
    drop_db_and_tables,
)
from vlaarena_shared.database import get_backend_db as get_db

__all__ = [
    "async_session_maker",
    "create_db_and_tables",
    "drop_db_and_tables",
    "engine",
    "get_db",
]
