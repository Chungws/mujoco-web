"""
Pydantic schemas for API requests/responses (VLA Arena MVP)
Based on WORKSPACE/FEATURES/001_MVP.md specification
"""

from typing import Literal

from pydantic import BaseModel, Field

# ==================== Session Schemas ====================


class SessionInitRequest(BaseModel):
    """Request schema for POST /api/sessions/init"""

    robot_id: str = Field(..., min_length=1, max_length=50)
    scene_id: str = Field(..., min_length=1, max_length=50)


class SessionResponse(BaseModel):
    """Response schema for POST /api/sessions/init"""

    session_id: str
    battle_id: str
    left_model: str = "???"  # Hidden until vote
    right_model: str = "???"  # Hidden until vote


# ==================== Turn Schemas (Battle) ====================


class TurnRequest(BaseModel):
    """Request schema for POST /api/battles/{battle_id}/turns"""

    instruction: str = Field(..., min_length=1)


class TurnResponse(BaseModel):
    """Response schema for POST /api/battles/{battle_id}/turns"""

    turn_id: str
    left_episode_id: str
    right_episode_id: str
    status: str  # "completed", "failed", "running"


# ==================== Episode Schemas ====================


class EpisodeState(BaseModel):
    """Single state in an episode (qpos, qvel, time)"""

    qpos: list[float]
    qvel: list[float]
    time: float


class EpisodeMetrics(BaseModel):
    """Episode execution metrics"""

    success: bool
    total_steps: int
    max_steps: int
    final_distance_to_goal: float


class EpisodeResponse(BaseModel):
    """Response schema for GET /api/episodes/{episode_id}"""

    episode_id: str
    actions: list[list[float]]  # Variable length, up to 50
    states: list[EpisodeState]  # Variable length, up to 50
    metrics: EpisodeMetrics


# ==================== Vote Schemas ====================


class VoteRequest(BaseModel):
    """Request schema for POST /api/votes"""

    battle_id: str
    vote: Literal["left_better", "right_better", "tie", "both_bad"]


class RevealedModels(BaseModel):
    """Model identities revealed after voting"""

    left: str  # model_id
    right: str  # model_id


class VoteResponse(BaseModel):
    """Response schema for POST /api/votes"""

    vote_id: str
    revealed_models: RevealedModels


# ==================== Models Schemas ====================


class ModelInfo(BaseModel):
    """Single model info for GET /api/models"""

    model_id: str
    name: str
    provider: str
    status: Literal["active", "inactive"]


class ModelsListResponse(BaseModel):
    """Response schema for GET /api/models"""

    models: list[ModelInfo]


# ==================== Leaderboard Schemas ====================


class ModelRanking(BaseModel):
    """Single model ranking in leaderboard"""

    model_id: str
    name: str
    elo_score: int
    vote_count: int
    win_rate: float


class LeaderboardResponse(BaseModel):
    """Response schema for GET /api/leaderboard"""

    rankings: list[ModelRanking]


# ==================== Error Schemas ====================


class ErrorResponse(BaseModel):
    """Standard error response"""

    error: str
    detail: str | None = None
    status_code: int
