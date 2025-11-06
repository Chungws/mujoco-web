# Feature: VLA Execution Server

**Status:** Week 3 - Implementation
**Priority:** HIGH
**Timeline:** Week 3-5 (3 weeks)
**Target Date:** Week 5 completion
**Related ADR:** [ADR-003: VLA Server Separation](../ARCHITECTURE/ADR_003-VLA_Server_Separation.md)

---

## 📋 Overview

Independent microservice for executing VLA (Vision-Language-Action) models in MuJoCo simulated environments. Receives execution requests via HTTP, runs episodes, and returns action/state sequences.

**Key Features:**
- MuJoCo simulation (Franka Emika Panda robot)
- VLA model inference (Octo-Small, SmolVLA)
- Episode generation (up to 50 steps)
- State-based recording (qpos, qvel, time)
- GPU/CPU/MPS support

---

## 🎯 Objectives

### MVP Scope

**What's Included:**
- ✅ 1 robot (Franka Emika Panda from MuJoCo Menagerie)
- ✅ 1 scene (Table pick-and-place)
- ✅ 2 VLA models (Octo-Small 27M, SmolVLA 450M)
- ✅ Episode execution (max 50 steps)
- ✅ Action/state recording
- ✅ HTTP API (POST /execute)
- ✅ MacBook compatible (CPU/MPS mode)

**What's Deferred (Post-MVP):**
- ❌ Multiple robots (WidowX, UR5)
- ❌ Multiple scenes (Kitchen, Warehouse)
- ❌ Model fine-tuning endpoints
- ❌ Real-time streaming
- ❌ Episode caching

---

## 🏗️ Architecture

### High-Level Flow

```
Backend → HTTP POST /execute
  ↓
VLA Server:
  1. Parse request (model_id, robot_id, scene_id, instruction)
  2. Load MuJoCo environment (lazy loading, cached)
  3. Load VLA model (lazy loading, cached)
  4. Run episode loop (max 50 steps):
     a. Get observation (camera + proprioception)
     b. VLA inference (action from observation + instruction)
     c. MuJoCo step (apply action, update state)
     d. Record (action, qpos, qvel, time)
     e. Check termination (goal reached or max steps)
  5. Return episode data
  ↓
Response: {actions, states, duration_ms, metadata}
```

### Component Diagram

```
┌────────────────────────────────────────────────┐
│         VLA Server (FastAPI)                    │
│                                                 │
│  ┌──────────────────────────────────────────┐ │
│  │  API Layer (api/execute.py)               │ │
│  │  - POST /execute                          │ │
│  │  - POST /health                           │ │
│  │  - GET /models (list available models)   │ │
│  └────────────┬─────────────────────────────┘ │
│               │                                 │
│  ┌────────────▼─────────────────────────────┐ │
│  │  Execution Service                        │ │
│  │  (services/execution_service.py)          │ │
│  │  - Coordinate MuJoCo + VLA                │ │
│  │  - Episode loop management                │ │
│  │  - Termination logic                      │ │
│  └───────┬──────────────┬───────────────────┘ │
│          │              │                       │
│  ┌───────▼──────┐  ┌───▼─────────────────┐   │
│  │ MuJoCo Env   │  │ VLA Model Manager   │   │
│  │ Manager      │  │                     │   │
│  │              │  │                     │   │
│  │ - Load robot │  │ - Load model        │   │
│  │ - Load scene │  │ - Inference         │   │
│  │ - Step sim   │  │ - Device mgmt       │   │
│  │ - Get obs    │  │ - Model caching     │   │
│  └──────────────┘  └─────────────────────┘   │
└────────────────────────────────────────────────┘
```

---

## 📦 Project Structure

```
vla-server/
├── src/vla_server/
│   ├── __init__.py
│   ├── main.py                    # FastAPI app
│   ├── config.py                  # Settings (Pydantic)
│   │
│   ├── api/
│   │   ├── __init__.py
│   │   ├── execute.py             # POST /execute
│   │   ├── health.py              # GET /health
│   │   └── models.py              # GET /models
│   │
│   ├── services/
│   │   ├── __init__.py
│   │   ├── execution_service.py   # Main orchestration
│   │   ├── mujoco_env.py          # MuJoCo environment
│   │   └── vla_model.py           # VLA model loading
│   │
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── execute.py             # ExecuteRequest/Response
│   │   └── common.py              # Shared schemas
│   │
│   └── utils/
│       ├── __init__.py
│       └── device.py              # GPU/CPU/MPS detection
│
├── tests/
│   ├── conftest.py                # Pytest fixtures
│   ├── test_mujoco_env.py         # MuJoCo tests (15 tests)
│   ├── test_vla_model.py          # VLA model tests (12 tests)
│   ├── test_execution_service.py  # Integration tests (20 tests)
│   └── test_execute_api.py        # API tests (10 tests)
│
├── models/
│   ├── robots/
│   │   └── franka_panda/          # MuJoCo Menagerie
│   │       ├── scene.xml
│   │       ├── franka.xml
│   │       └── meshes/
│   └── scenes/
│       └── table/
│           └── table.xml
│
├── model_cache/                   # HuggingFace cache
│
├── pyproject.toml                 # Dependencies
├── README.md
└── .env.example
```

---

## 🔧 Implementation Guide

### Step 1: Project Setup

**Create directory structure:**

```bash
cd mujoco-web
mkdir -p vla-server/src/vla_server/{api,services,schemas,utils}
mkdir -p vla-server/tests
mkdir -p vla-server/models/{robots,scenes}
```

**Initialize pyproject.toml:**

```toml
[project]
name = "vla-server"
version = "0.1.0"
description = "VLA Execution Server for VLA Arena"
requires-python = ">=3.11"
dependencies = [
    "fastapi>=0.115.12",
    "uvicorn[standard]>=0.34.0",
    "pydantic>=2.10.6",
    "pydantic-settings>=2.7.1",
    "mujoco>=3.2.0",
    "torch>=2.5.0",
    "transformers>=4.46.0",
    "accelerate>=1.2.0",
    "pillow>=11.0.0",
    "numpy>=2.2.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=8.3.5",
    "pytest-asyncio>=0.25.2",
    "pytest-cov>=6.0.0",
    "httpx>=0.28.1",
    "ruff>=0.9.2",
]

[tool.pytest.ini_options]
testpaths = ["tests"]
asyncio_mode = "auto"
pythonpath = ["src"]
```

**Install dependencies:**

```bash
cd vla-server
uv sync --all-extras
```

---

### Step 2: Configuration (config.py)

```python
"""
Configuration for VLA Server
Uses Pydantic Settings for environment variable management
"""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """VLA Server settings"""

    # Server
    host: str = "0.0.0.0"
    port: int = 8001
    reload: bool = True

    # Models
    mujoco_model_path: str = "./models"
    vla_model_cache: str = "./model_cache"
    default_vla_model: str = "octo-small"

    # Execution
    max_episode_steps: int = 50
    control_frequency: float = 10.0  # Hz
    device: str = "auto"  # auto, cuda, cpu, mps

    # Logging
    log_level: str = "INFO"

    class Config:
        env_prefix = "VLA_"
        env_file = ".env"


settings = Settings()
```

---

### Step 3: Schemas (schemas/execute.py)

```python
"""
Request/Response schemas for /execute endpoint
"""

from pydantic import BaseModel, Field


class ExecuteRequest(BaseModel):
    """Request to execute VLA model in MuJoCo environment"""

    model_id: str = Field(..., description="VLA model identifier")
    robot_id: str = Field(..., description="Robot type (e.g., 'franka')")
    scene_id: str = Field(..., description="Scene type (e.g., 'table')")
    instruction: str = Field(..., min_length=1, description="Natural language instruction")

    class Config:
        json_schema_extra = {
            "example": {
                "model_id": "octo-small",
                "robot_id": "franka",
                "scene_id": "table",
                "instruction": "Pick up the red cube",
            }
        }


class State(BaseModel):
    """MuJoCo state at a single timestep"""

    qpos: list[float] = Field(..., description="Joint positions")
    qvel: list[float] = Field(..., description="Joint velocities")
    time: float = Field(..., description="Simulation time")


class ExecuteResponse(BaseModel):
    """Response containing episode data"""

    actions: list[list[float]] = Field(..., description="Action sequence (variable length)")
    states: list[State] = Field(..., description="State sequence (same length as actions)")
    duration_ms: int = Field(..., description="Execution duration in milliseconds")
    metadata: dict = Field(
        default_factory=dict,
        description="Additional metadata (num_steps, early_termination, etc.)",
    )

    class Config:
        json_schema_extra = {
            "example": {
                "actions": [[0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]],
                "states": [{"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}],
                "duration_ms": 5120,
                "metadata": {"num_steps": 35, "max_steps": 50, "early_termination": True},
            }
        }
```

---

### Step 4: MuJoCo Environment (TDD)

**Test first (tests/test_mujoco_env.py):**

```python
"""
Tests for MuJoCo environment management
Following TDD: Red → Green → Refactor
"""

import pytest
from vla_server.services.mujoco_env import MuJoCoEnvironment


class TestMuJoCoEnvironment:
    """Test suite for MuJoCo environment"""

    def test_load_franka_robot(self):
        """
        Test loading Franka Emika Panda robot

        Arrange: Create environment with franka robot
        Act: Load environment
        Assert: Model loaded successfully, correct DOF
        """
        # Arrange & Act
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Assert
        assert env.model is not None
        assert env.data is not None
        assert env.model.nq >= 7  # At least 7 joints for Franka

    def test_reset_environment(self):
        """
        Test environment reset to initial state

        Arrange: Create environment and modify state
        Act: Reset environment
        Assert: State returns to initial configuration
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        initial_qpos = env.data.qpos.copy()

        # Modify state
        env.data.qpos[:] = 999.0

        # Act
        env.reset()

        # Assert
        assert not (env.data.qpos == 999.0).all()

    def test_step_simulation(self):
        """
        Test stepping simulation with action

        Arrange: Create environment
        Act: Apply action and step
        Assert: Simulation time advances
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")
        initial_time = env.data.time
        action = [0.0] * 8  # 8-dim action

        # Act
        env.step(action)

        # Assert
        assert env.data.time > initial_time

    def test_get_observation(self):
        """
        Test getting observation from environment

        Arrange: Create environment
        Act: Get observation
        Assert: Returns dict with required keys
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        obs = env.get_observation()

        # Assert
        assert "image" in obs
        assert "qpos" in obs
        assert "qvel" in obs
        assert obs["image"].shape[-1] == 3  # RGB

    def test_get_state(self):
        """
        Test getting current state

        Arrange: Create environment
        Act: Get state
        Assert: Returns qpos, qvel, time
        """
        # Arrange
        env = MuJoCoEnvironment(robot_id="franka", scene_id="table")

        # Act
        state = env.get_state()

        # Assert
        assert "qpos" in state
        assert "qvel" in state
        assert "time" in state
        assert len(state["qpos"]) == env.model.nq
        assert len(state["qvel"]) == env.model.nv

    # Add 10 more tests...
    # - test_invalid_robot_id
    # - test_invalid_scene_id
    # - test_action_dimension_mismatch
    # - test_render_image
    # - test_multiple_steps
    # ...
```

**Then implement (services/mujoco_env.py):**

```python
"""
MuJoCo environment management for VLA execution
Handles robot/scene loading, simulation stepping, observation extraction
"""

import mujoco
import numpy as np
from pathlib import Path
from ..config import settings


class MuJoCoEnvironment:
    """MuJoCo simulation environment"""

    def __init__(self, robot_id: str, scene_id: str):
        """
        Initialize MuJoCo environment

        Args:
            robot_id: Robot identifier (e.g., "franka")
            scene_id: Scene identifier (e.g., "table")
        """
        self.robot_id = robot_id
        self.scene_id = scene_id

        # Load model
        model_path = self._get_model_path(robot_id, scene_id)
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        # Renderer for observations
        self.renderer = mujoco.Renderer(self.model, height=224, width=224)

        # Store initial state
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()

    def _get_model_path(self, robot_id: str, scene_id: str) -> Path:
        """Get path to MuJoCo XML model"""
        base_path = Path(settings.mujoco_model_path)
        model_path = base_path / "robots" / robot_id / "scene.xml"

        if not model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")

        return model_path

    def reset(self) -> dict:
        """Reset environment to initial state"""
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = self.initial_qvel
        self.data.time = 0.0
        mujoco.mj_forward(self.model, self.data)

        return self.get_observation()

    def step(self, action: list[float]) -> dict:
        """
        Step simulation with action

        Args:
            action: 8-dim action vector

        Returns:
            Observation after step
        """
        # Apply action to actuators
        self.data.ctrl[:] = action[: self.model.nu]

        # Step simulation
        mujoco.mj_step(self.model, self.data)

        return self.get_observation()

    def get_observation(self) -> dict:
        """
        Get current observation

        Returns:
            Dictionary with image, qpos, qvel
        """
        # Render image
        self.renderer.update_scene(self.data)
        image = self.renderer.render()

        return {
            "image": image,
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
        }

    def get_state(self) -> dict:
        """
        Get current state for episode recording

        Returns:
            Dictionary with qpos, qvel, time
        """
        return {
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "time": float(self.data.time),
        }
```

---

### Step 5: VLA Model Manager (TDD)

**Test first (tests/test_vla_model.py):**

```python
"""
Tests for VLA model loading and inference
"""

import pytest
from vla_server.services.vla_model import VLAModelManager


class TestVLAModelManager:
    """Test suite for VLA model management"""

    def test_load_octo_small(self):
        """
        Test loading Octo-Small model

        Arrange: Create model manager
        Act: Load octo-small
        Assert: Model loaded successfully
        """
        # Arrange & Act
        manager = VLAModelManager(model_id="octo-small", device="cpu")

        # Assert
        assert manager.model is not None
        assert manager.device == "cpu"

    def test_inference_returns_action(self):
        """
        Test VLA inference returns valid action

        Arrange: Load model, create observation
        Act: Run inference
        Assert: Returns 8-dim action
        """
        # Arrange
        manager = VLAModelManager(model_id="octo-small", device="cpu")
        obs = {"image": np.zeros((224, 224, 3)), "qpos": [0.0] * 7}
        instruction = "Pick up the cube"

        # Act
        action = manager.predict(obs, instruction)

        # Assert
        assert len(action) == 8
        assert all(isinstance(a, float) for a in action)

    # Add 10 more tests...
```

**Then implement (services/vla_model.py):**

```python
"""
VLA model loading and inference
Supports Octo-Small, SmolVLA from HuggingFace
"""

import torch
from transformers import AutoModel, AutoProcessor
from ..config import settings


class VLAModelManager:
    """VLA model manager"""

    MODEL_HUB_IDS = {
        "octo-small": "octo-models/octo-small",
        "smolvla": "HuggingFaceTB/SmolVLA",
    }

    def __init__(self, model_id: str, device: str = "auto"):
        """
        Initialize VLA model

        Args:
            model_id: Model identifier
            device: Device (auto, cuda, cpu, mps)
        """
        self.model_id = model_id
        self.device = self._get_device(device)

        # Load model
        hub_id = self.MODEL_HUB_IDS.get(model_id)
        if not hub_id:
            raise ValueError(f"Unknown model: {model_id}")

        self.processor = AutoProcessor.from_pretrained(
            hub_id, cache_dir=settings.vla_model_cache
        )
        self.model = AutoModel.from_pretrained(
            hub_id, cache_dir=settings.vla_model_cache
        )
        self.model = self.model.to(self.device)
        self.model.eval()

    def _get_device(self, device: str) -> str:
        """Get torch device"""
        if device == "auto":
            if torch.cuda.is_available():
                return "cuda"
            elif torch.backends.mps.is_available():
                return "mps"
            else:
                return "cpu"
        return device

    def predict(self, observation: dict, instruction: str) -> list[float]:
        """
        Predict action from observation and instruction

        Args:
            observation: Dict with image, qpos, qvel
            instruction: Natural language instruction

        Returns:
            8-dim action vector
        """
        # Process inputs
        inputs = self.processor(
            images=observation["image"],
            text=instruction,
            return_tensors="pt",
        )
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Inference
        with torch.no_grad():
            outputs = self.model(**inputs)
            action = outputs.action  # Model-specific output

        # Convert to list
        return action.cpu().numpy().tolist()[0]
```

---

### Step 6: Execution Service (TDD)

```python
"""
Execution service - orchestrates MuJoCo + VLA
"""

import time
from .mujoco_env import MuJoCoEnvironment
from .vla_model import VLAModelManager
from ..schemas.execute import ExecuteRequest, ExecuteResponse, State


class ExecutionService:
    """Service for executing VLA episodes"""

    def __init__(self):
        # Lazy loading - models cached here
        self.envs = {}  # robot_id+scene_id → MuJoCoEnvironment
        self.models = {}  # model_id → VLAModelManager

    async def execute(self, request: ExecuteRequest) -> ExecuteResponse:
        """
        Execute VLA episode

        Args:
            request: Execution request

        Returns:
            Episode data with actions and states
        """
        start_time = time.time()

        # 1. Get or create environment
        env = self._get_environment(request.robot_id, request.scene_id)

        # 2. Get or load model
        model = self._get_model(request.model_id)

        # 3. Reset environment
        env.reset()

        # 4. Run episode
        actions = []
        states = []

        for step in range(50):  # Max 50 steps
            # Get observation
            obs = env.get_observation()

            # VLA inference
            action = model.predict(obs, request.instruction)

            # Step simulation
            env.step(action)

            # Record
            actions.append(action)
            states.append(State(**env.get_state()))

            # Check termination (TODO: implement goal checking)
            # if self._is_goal_reached(env, request):
            #     break

        # 5. Calculate duration
        duration_ms = int((time.time() - start_time) * 1000)

        # 6. Return response
        return ExecuteResponse(
            actions=actions,
            states=states,
            duration_ms=duration_ms,
            metadata={
                "num_steps": len(actions),
                "max_steps": 50,
                "early_termination": len(actions) < 50,
            },
        )

    def _get_environment(self, robot_id: str, scene_id: str) -> MuJoCoEnvironment:
        """Get or create MuJoCo environment (cached)"""
        key = f"{robot_id}_{scene_id}"
        if key not in self.envs:
            self.envs[key] = MuJoCoEnvironment(robot_id, scene_id)
        return self.envs[key]

    def _get_model(self, model_id: str) -> VLAModelManager:
        """Get or load VLA model (cached)"""
        if model_id not in self.models:
            self.models[model_id] = VLAModelManager(model_id)
        return self.models[model_id]
```

---

### Step 7: API Endpoint (api/execute.py)

```python
"""
Execute endpoint - POST /execute
"""

from fastapi import APIRouter, HTTPException, status
from ..schemas.execute import ExecuteRequest, ExecuteResponse
from ..services.execution_service import ExecutionService


router = APIRouter(prefix="", tags=["execution"])

# Global service instance (models cached here)
execution_service = ExecutionService()


@router.post("/execute", response_model=ExecuteResponse)
async def execute_vla(request: ExecuteRequest):
    """
    Execute VLA model in MuJoCo environment

    Args:
        request: Execution request with model, robot, scene, instruction

    Returns:
        Episode data with actions and states
    """
    try:
        response = await execution_service.execute(request)
        return response

    except FileNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Model or scene not found: {str(e)}",
        )

    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid request: {str(e)}",
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Execution failed: {str(e)}",
        )
```

---

## 📅 Implementation Timeline

### Week 3: Core Infrastructure
- [x] Project setup (directory, dependencies)
- [x] MuJoCo environment (17 tests, TDD) ✅
- [x] VLA model manager (12 tests, TDD) ✅
- [x] Execution service (13 tests, TDD) ✅
- [x] Download Franka model from Menagerie ✅

### Week 4: Integration & Testing
- [ ] API endpoints (10 tests)
- [ ] Full integration tests
- [ ] MacBook testing (CPU/MPS)
- [ ] Backend integration (TurnService)
- [ ] Error handling

### Week 5: Optimization & Documentation
- [ ] Model caching optimization
- [ ] Performance tuning
- [ ] API documentation (OpenAPI)
- [ ] Deployment guide
- [ ] README

---

## 🧪 Testing Strategy

### Unit Tests (57 total)

**MuJoCo Environment (15 tests):**
- Load robot/scene
- Reset environment
- Step simulation
- Get observation/state
- Error handling

**VLA Model (12 tests):**
- Load models (Octo, SmolVLA)
- Inference
- Device management
- Error handling

**Execution Service (20 tests):**
- End-to-end execution
- Model caching
- Environment caching
- Early termination
- Error scenarios

**API (10 tests):**
- POST /execute success
- POST /execute errors (404, 400, 500)
- Request validation

### Integration Tests

```python
def test_full_episode_generation():
    """Test full episode from API to response"""
    client = TestClient(app)

    response = client.post("/execute", json={
        "model_id": "octo-small",
        "robot_id": "franka",
        "scene_id": "table",
        "instruction": "Pick up the red cube"
    })

    assert response.status_code == 200
    data = response.json()
    assert len(data["actions"]) > 0
    assert len(data["states"]) == len(data["actions"])
```

---

## 🚀 Deployment

### Local Development

```bash
# Start VLA server
cd vla-server
uv run uvicorn vla_server.main:app --reload --port 8001

# Test endpoint
curl -X POST http://localhost:8001/execute \
  -H "Content-Type: application/json" \
  -d '{
    "model_id": "octo-small",
    "robot_id": "franka",
    "scene_id": "table",
    "instruction": "Pick up the red cube"
  }'
```

### Docker

```dockerfile
FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0

# Install uv
RUN pip install uv

# Copy project
WORKDIR /app
COPY pyproject.toml .
COPY src/ src/

# Install dependencies
RUN uv sync --no-dev

# Download models (at build time)
RUN uv run python -c "from vla_server.services.vla_model import VLAModelManager; VLAModelManager('octo-small')"

CMD ["uv", "run", "uvicorn", "vla_server.main:app", "--host", "0.0.0.0", "--port", "8001"]
```

---

## 📊 Success Criteria

**MVP Complete When:**
1. ✅ VLA server runs independently
2. ✅ Franka robot loads in MuJoCo
3. ✅ Octo-Small inference works
4. ✅ Episodes generated (max 50 steps)
5. ✅ Backend integration successful
6. ✅ All tests pass (57 tests)
7. ✅ MacBook compatible (CPU/MPS)

---

## 📚 References

### Models
- **MuJoCo Menagerie:** https://github.com/google-deepmind/mujoco_menagerie
- **Octo:** https://octo-models.github.io/
- **SmolVLA:** https://huggingface.co/HuggingFaceTB/SmolVLA

### Documentation
- **ADR-003:** VLA Server Separation
- **ROADMAP.md:** Development timeline
- **001_MVP.md:** Overall MVP scope

---

**Created:** 2025-01-06
**Last Updated:** 2025-01-06
**Status:** Implementation (Week 3)
