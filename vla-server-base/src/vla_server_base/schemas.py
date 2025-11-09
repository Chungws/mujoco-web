"""
Common Pydantic schemas for VLA services
"""

from pydantic import BaseModel


class ExecuteRequest(BaseModel):
    """Request for episode execution"""

    instruction: str
    xml_string: str


class State(BaseModel):
    """MuJoCo state snapshot"""

    qpos: list[float]
    qvel: list[float]
    time: float


class ExecuteResponse(BaseModel):
    """Response from episode execution"""

    actions: list[list[float]]
    states: list[State]
    duration_ms: int
