"""
SQLModel models for PostgreSQL (shared between backend and worker)
"""

from datetime import UTC, datetime
from typing import Any, Dict, Optional

from sqlalchemy import JSON, DateTime
from sqlmodel import Column, Field, SQLModel


class Session(SQLModel, table=True):
    """
    Session container for multiple battles (PostgreSQL)

    One session can have multiple battles with different model pairs.
    """

    __tablename__ = "sessions"

    id: Optional[int] = Field(default=None, primary_key=True)
    session_id: str = Field(unique=True, index=True, max_length=50)
    title: str = Field(max_length=200)  # First prompt for display
    user_id: Optional[str] = Field(
        default=None, index=True, max_length=50
    )  # Anonymous user UUID string
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )
    last_active_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class Battle(SQLModel, table=True):
    """
    Single battle between two models (PostgreSQL)

    Conversation history stored in Turn and Message tables.
    """

    __tablename__ = "battles"

    id: Optional[int] = Field(default=None, primary_key=True)
    battle_id: str = Field(unique=True, index=True, max_length=50)
    session_id: str = Field(
        index=True, max_length=50
    )  # FK to sessions (application-level)
    left_model_id: str = Field(max_length=255)
    right_model_id: str = Field(max_length=255)

    seq_in_session: int = Field(index=True)  # Order of battle in session

    status: str = Field(
        default="ongoing", max_length=20, index=True
    )  # ongoing, voted, abandoned
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )


class Turn(SQLModel, table=True):
    __tablename__ = "turns"
    id: Optional[int] = Field(default=None, primary_key=True)
    turn_id: str = Field(unique=True, index=True, max_length=50)

    session_id: str = Field(index=True, max_length=50)
    battle_id: str = Field(index=True, max_length=50)
    battle_seq_in_session: int = Field(index=True)

    seq: int = Field(index=True)

    user_input: Optional[str] = None
    user_input_json: Optional[Dict[str, Any]] = Field(
        default=None, sa_column=Column(JSON)
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class Message(SQLModel, table=True):
    __tablename__ = "messages"
    id: Optional[int] = Field(default=None, primary_key=True)
    message_id: str = Field(unique=True, index=True, max_length=50)

    session_id: str = Field(index=True, max_length=50)
    battle_id: str = Field(index=True, max_length=50)
    turn_id: str = Field(index=True, max_length=50)

    battle_seq_in_session: int = Field(index=True)
    turn_seq: int = Field(index=True)
    seq_in_turn: int = Field(index=True)

    session_seq: Optional[int] = Field(default=None, index=True)

    side: str = Field(max_length=10)
    content: Optional[str] = None
    content_json: Optional[Dict[str, Any]] = Field(default=None, sa_column=Column(JSON))
    token_count: Optional[int] = None
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class Vote(SQLModel, table=True):
    """
    User vote on battle outcome with denormalized model IDs (PostgreSQL)

    Denormalized to avoid JOIN queries in worker aggregation.
    """

    __tablename__ = "votes"

    id: Optional[int] = Field(default=None, primary_key=True)
    vote_id: str = Field(unique=True, index=True, max_length=50)
    battle_id: str = Field(unique=True, index=True, max_length=50)  # 1:1 relationship
    session_id: str = Field(index=True, max_length=50)  # For analytics
    vote: str = Field(max_length=20)  # left_better, right_better, tie, both_bad
    left_model_id: str = Field(max_length=255)  # Denormalized from battle
    right_model_id: str = Field(max_length=255)  # Denormalized from battle
    processing_status: str = Field(
        default="pending", max_length=20, index=True
    )  # pending, processed, failed
    processed_at: Optional[datetime] = Field(
        default=None, sa_column=Column(DateTime(timezone=True), nullable=True)
    )
    error_message: Optional[str] = Field(default=None)
    voted_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class ModelStats(SQLModel, table=True):
    """
    Leaderboard model statistics (PostgreSQL)

    Updated by worker hourly based on votes table.
    """

    __tablename__ = "model_stats"

    id: Optional[int] = Field(default=None, primary_key=True)
    model_id: str = Field(unique=True, index=True, max_length=255)
    elo_score: int = Field(default=1500, index=True)
    elo_ci: float = Field(default=200.0)  # 95% confidence interval
    vote_count: int = Field(default=0, index=True)
    win_count: int = Field(default=0)
    loss_count: int = Field(default=0)
    tie_count: int = Field(default=0)
    win_rate: float = Field(default=0.0)
    organization: str = Field(max_length=255)
    license: str = Field(max_length=50)  # 'proprietary', 'open-source', etc.
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )


class WorkerStatus(SQLModel, table=True):
    """
    Worker execution status tracking (PostgreSQL)

    Stores last successful worker run timestamp
    """

    __tablename__ = "worker_status"

    id: Optional[int] = Field(default=None, primary_key=True)
    worker_name: str = Field(unique=True, max_length=100)  # e.g., "elo_aggregator"
    last_run_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )
    status: str = Field(max_length=50)  # 'success', 'failed', 'running'
    votes_processed: int = Field(default=0)
    error_message: Optional[str] = Field(default=None, max_length=1000)
