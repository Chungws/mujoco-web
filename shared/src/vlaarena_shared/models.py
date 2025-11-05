"""
SQLModel models for PostgreSQL (VLA Arena MVP)
Based on ADR-002: Database Schema Design
"""

from datetime import UTC, datetime

from sqlalchemy import DateTime
from sqlmodel import Column, Field, SQLModel


class Session(SQLModel, table=True):
    __tablename__ = "sessions"

    id: int | None = Field(default=None, primary_key=True)
    session_id: str = Field(unique=True, index=True, max_length=50)
    robot_id: str = Field(max_length=50)
    scene_id: str = Field(max_length=50)
    user_id: str | None = Field(default=None, index=True, max_length=50)
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )
    last_active_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class Battle(SQLModel, table=True):
    __tablename__ = "battles"

    id: int | None = Field(default=None, primary_key=True)
    battle_id: str = Field(unique=True, index=True, max_length=50)
    session_id: str = Field(index=True, max_length=50)
    seq_in_session: int = Field(index=True)

    left_model_id: str = Field(max_length=255)
    right_model_id: str = Field(max_length=255)

    status: str = Field(default="ongoing", max_length=20, index=True)
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

    id: int | None = Field(default=None, primary_key=True)
    turn_id: str = Field(unique=True, index=True, max_length=50)

    session_id: str = Field(index=True, max_length=50)
    battle_id: str = Field(index=True, max_length=50)
    battle_seq_in_session: int = Field(index=True)

    seq: int = Field(index=True)
    instruction: str

    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class Vote(SQLModel, table=True):
    __tablename__ = "votes"

    id: int | None = Field(default=None, primary_key=True)
    vote_id: str = Field(unique=True, index=True, max_length=50)
    battle_id: str = Field(unique=True, index=True, max_length=50)
    session_id: str = Field(index=True, max_length=50)

    robot_id: str = Field(index=True, max_length=50)
    scene_id: str = Field(index=True, max_length=50)

    left_model_id: str = Field(max_length=255)
    right_model_id: str = Field(max_length=255)

    vote: str = Field(max_length=20)

    processing_status: str = Field(default="pending", max_length=20, index=True)
    processed_at: datetime | None = Field(
        default=None, sa_column=Column(DateTime(timezone=True), nullable=True)
    )
    error_message: str | None = Field(default=None)
    voted_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False, index=True),
    )


class ModelStatsByRobot(SQLModel, table=True):
    __tablename__ = "model_stats_by_robot"

    id: int | None = Field(default=None, primary_key=True)
    model_id: str = Field(index=True, max_length=255)
    robot_id: str = Field(index=True, max_length=50)

    elo_score: int = Field(default=1500, index=True)
    elo_ci: float = Field(default=200.0)
    vote_count: int = Field(default=0, index=True)
    win_count: int = Field(default=0)
    loss_count: int = Field(default=0)
    tie_count: int = Field(default=0)
    win_rate: float = Field(default=0.0)
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )


class ModelStatsTotal(SQLModel, table=True):
    __tablename__ = "model_stats_total"

    id: int | None = Field(default=None, primary_key=True)
    model_id: str = Field(unique=True, index=True, max_length=255)

    elo_score: int = Field(default=1500, index=True)
    elo_ci: float = Field(default=200.0)
    vote_count: int = Field(default=0, index=True)
    win_count: int = Field(default=0)
    loss_count: int = Field(default=0)
    tie_count: int = Field(default=0)
    win_rate: float = Field(default=0.0)

    organization: str = Field(max_length=255)
    license: str = Field(max_length=50)

    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )


class WorkerStatus(SQLModel, table=True):
    __tablename__ = "worker_status"

    id: int | None = Field(default=None, primary_key=True)
    worker_name: str = Field(unique=True, max_length=100)
    last_run_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        sa_column=Column(DateTime(timezone=True), nullable=False),
    )
    status: str = Field(max_length=50)
    votes_processed: int = Field(default=0)
    error_message: str | None = Field(default=None, max_length=1000)
