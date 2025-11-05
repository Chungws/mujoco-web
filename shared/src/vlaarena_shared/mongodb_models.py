"""
MongoDB models for VLA Arena (using Beanie ODM)
Based on ADR-002: Database Schema Design
"""

from datetime import UTC, datetime
from typing import ClassVar

from beanie import Document, Indexed
from pydantic import BaseModel, Field


class State(BaseModel):
    """
    MuJoCo state at a single timestep

    Contains joint positions, velocities, and timestamp for replay
    """

    qpos: list[float] = Field(description="Joint positions")
    qvel: list[float] = Field(description="Joint velocities")
    time: float = Field(description="Simulation time in seconds")


class Episode(Document):
    """
    VLA model execution episode stored in MongoDB

    Contains actions and states for browser-based replay.
    Size: ~13 KB max (50 steps), average ~8 KB
    """

    # Identifiers
    episode_id: Indexed(str, unique=True)
    session_id: Indexed(str)
    battle_id: Indexed(str)
    turn_id: Indexed(str)

    # Sequence tracking
    battle_seq_in_session: Indexed(int)
    turn_seq: Indexed(int)
    seq_in_turn: Indexed(int) = Field(description="0=left, 1=right")

    # Model info
    side: str = Field(description="left or right")
    model_id: Indexed(str)

    # Execution data (variable length, up to EPISODE_MAX_STEPS)
    actions: list[list[float]] = Field(description="8-dim actions for each step")
    states: list[State] = Field(description="MuJoCo states (qpos, qvel, time) for replay")

    # Metadata
    duration_ms: int = Field(description="Execution time in milliseconds")
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        description="Episode creation timestamp",
    )

    class Settings:
        name = "episodes"
        use_state_management = True
        indexes: ClassVar = [
            "episode_id",
            [("battle_id", 1), ("side", 1)],
            "turn_id",
            "model_id",
        ]
