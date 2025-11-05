"""
Pydantic schemas for API requests/responses (shared between backend and frontend)
"""

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field

# ==================== Common Schemas ====================


class Response(BaseModel):
    """Single model response in a battle"""

    position: Literal["left", "right"]
    text: str
    latency_ms: int


# ==================== Session Schemas ====================


class SessionCreate(BaseModel):
    """Request schema for creating a new session"""

    prompt: str = Field(..., min_length=1, max_length=10000)
    user_id: str | None = None  # Optional anonymous user ID (UUID string)


class SessionResponse(BaseModel):
    """Response schema for session creation (includes first battle)"""

    session_id: str
    battle_id: str
    message_id: str  # Always "msg_1" for first message
    responses: list[Response]


class SessionItem(BaseModel):
    """Single session item in session list"""

    session_id: str
    title: str
    created_at: datetime
    last_active_at: datetime


class SessionListResponse(BaseModel):
    """Response schema for GET /api/sessions"""

    sessions: list[SessionItem]
    total: int


# ==================== Battle Schemas ====================


class BattleCreate(BaseModel):
    """Request schema for creating a new battle"""

    prompt: str = Field(..., min_length=1, max_length=10000)


class BattleResponse(BaseModel):
    """Response schema for battle creation"""

    battle_id: str
    message_id: str
    responses: list[Response]


class FollowUpCreate(BaseModel):
    """Request schema for follow-up message"""

    prompt: str = Field(..., min_length=1, max_length=10000)


class FollowUpResponse(BaseModel):
    """Response schema for follow-up message"""

    battle_id: str
    message_id: str
    responses: list[Response]
    message_count: int  # Total messages in conversation (1-6)
    max_messages: int = 6  # Maximum allowed messages


class BattleItem(BaseModel):
    """Single battle item in battle list"""

    battle_id: str
    left_model_id: str
    right_model_id: str
    conversation: list[dict]
    status: str
    vote: str | None = None  # Only present if status is 'voted'
    created_at: datetime


class BattleListResponse(BaseModel):
    """Response schema for GET /api/sessions/{session_id}/battles"""

    session_id: str
    battles: list[BattleItem]


# ==================== Vote Schemas ====================


class VoteCreate(BaseModel):
    """Request schema for submitting a vote"""

    vote: Literal["left_better", "right_better", "tie", "both_bad"]


class RevealedModels(BaseModel):
    """Model identities revealed after voting"""

    left: str  # model_id
    right: str  # model_id


class VoteResponse(BaseModel):
    """Response schema for vote submission"""

    battle_id: str
    vote: str
    revealed_models: RevealedModels


# ==================== Model Schemas ====================


class ModelInfo(BaseModel):
    """Single model information"""

    model_id: str
    name: str
    provider: str
    status: Literal["active", "inactive"]


class ModelsListResponse(BaseModel):
    """Response schema for GET /api/models"""

    models: list[ModelInfo]


# ==================== Leaderboard Schemas ====================


class ModelStatsResponse(BaseModel):
    """Single model statistics in leaderboard"""

    rank: int
    model_id: str
    model_name: str
    elo_score: int
    elo_ci: float
    vote_count: int
    win_rate: float
    organization: str
    license: str


class LeaderboardMetadata(BaseModel):
    """Leaderboard metadata"""

    total_models: int
    total_votes: int
    last_updated: datetime


class LeaderboardResponse(BaseModel):
    """Response schema for GET /api/leaderboard"""

    leaderboard: list[ModelStatsResponse]
    metadata: LeaderboardMetadata


# ==================== Error Schemas ====================


class ErrorResponse(BaseModel):
    """Standard error response"""

    error: str
    detail: str | None = None
    status_code: int
