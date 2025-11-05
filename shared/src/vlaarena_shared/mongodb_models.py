"""
MongoDB models for VLA Arena (using Beanie ODM)
Based on ADR-002: Database Schema Design
"""

from datetime import UTC, datetime
from typing import Optional

from beanie import Document
from pydantic import BaseModel, Field


class State(BaseModel):
    """
    MuJoCo state at a single timestep

    Contains joint positions, velocities, and timestamp for replay
    """

    qpos: list[float] = Field(description="Joint positions")
    qvel: list[float] = Field(description="Joint velocities")
    time: float = Field(description="Simulation time in seconds")


class Metrics(BaseModel):
    """
    Episode execution metrics

    Summary statistics for episode performance evaluation
    """

    success: bool = Field(description="Task completion status")
    total_steps: int = Field(description="Actual steps executed")
    max_steps: int = Field(description="Configured maximum steps")
    terminated_early: bool = Field(default=False)
    final_distance_to_goal: Optional[float] = Field(default=None)
    collision_count: int = Field(default=0)
    gripper_opened_at_step: Optional[int] = Field(default=None)


class Episode(Document):
    """
    VLA model execution episode stored in MongoDB

    Contains actions, states, and metrics for browser-based replay.
    Size: ~13 KB max (50 steps), average ~8 KB
    """

    # Identifiers
    episode_id: str = Field(unique=True, index=True)
    session_id: str = Field(index=True)
    battle_id: str = Field(index=True)
    turn_id: str = Field(index=True)

    # Sequence tracking
    battle_seq_in_session: int = Field(index=True)
    turn_seq: int = Field(index=True)
    seq_in_turn: int = Field(
        index=True, description="0=left, 1=right"
    )

    # Model info
    side: str = Field(description="left or right")
    model_id: str = Field(index=True)

    # Execution data (variable length, up to EPISODE_MAX_STEPS)
    actions: list[list[float]] = Field(
        description="8-dim actions for each step"
    )
    states: list[State] = Field(
        description="MuJoCo states (qpos, qvel, time) for replay"
    )

    # Metrics (small summary)
    metrics: Metrics = Field(description="Episode performance metrics")

    # Metadata
    duration_ms: int = Field(description="Execution time in milliseconds")
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC),
        description="Episode creation timestamp",
    )

    class Settings:
        name = "episodes"
        use_state_management = True
        indexes = [
            "episode_id",
            [("battle_id", 1), ("side", 1)],
            "turn_id",
            "model_id",
        ]
