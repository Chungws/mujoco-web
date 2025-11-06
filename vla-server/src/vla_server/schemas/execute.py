"""
Request/Response schemas for /execute endpoint
"""

from pydantic import BaseModel, ConfigDict, Field


class ExecuteRequest(BaseModel):
    """Request to execute VLA model in MuJoCo environment"""

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "model_id": "octo-small",
                "robot_id": "franka",
                "scene_id": "table",
                "instruction": "Pick up the red cube",
            }
        }
    )

    model_id: str = Field(..., description="VLA model identifier")
    robot_id: str = Field(..., description="Robot type (e.g., 'franka')")
    scene_id: str = Field(..., description="Scene type (e.g., 'table')")
    instruction: str = Field(..., min_length=1, description="Natural language instruction")


class State(BaseModel):
    """MuJoCo state at a single timestep"""

    qpos: list[float] = Field(..., description="Joint positions")
    qvel: list[float] = Field(..., description="Joint velocities")
    time: float = Field(..., description="Simulation time")


class ExecuteResponse(BaseModel):
    """Response containing episode data"""

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "actions": [[0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]],
                "states": [{"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}],
                "duration_ms": 5120,
                "metadata": {"num_steps": 35, "max_steps": 50, "early_termination": True},
            }
        }
    )

    actions: list[list[float]] = Field(..., description="Action sequence (variable length)")
    states: list[State] = Field(..., description="State sequence (same length as actions)")
    duration_ms: int = Field(..., description="Execution duration in milliseconds")
    metadata: dict = Field(
        default_factory=dict,
        description="Additional metadata (num_steps, early_termination, etc.)",
    )
