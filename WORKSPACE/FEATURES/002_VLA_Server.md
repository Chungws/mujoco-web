# Feature: VLA Execution Server

**Status:** Week 3 - Phase 1 Complete (Infrastructure)
**Priority:** HIGH
**Timeline:** Week 3-5 (3 weeks)
**Target Date:** Week 5 completion
**Related ADR:** [ADR-003: VLA Server Separation](../ARCHITECTURE/ADR_003-VLA_Server_Separation.md)
**Phase 1 Progress:** 26/26 tests passing (config + MuJoCo env)

---

## 📋 Overview

Independent microservice for executing VLA (Vision-Language-Action) models in MuJoCo simulated environments. Each VLA model runs as a separate server instance with model-specific adapters for input/output processing.

**Key Features:**
- Stateless MuJoCo simulation (XML-based, no file system dependency)
- VLA Adapter pattern (model-specific preprocessing/postprocessing)
- Dynamic robot/scene composition
- Episode generation (RT-1/Octo standard: 5 Hz, 15s max)
- State-based recording (qpos, qvel, time)
- GPU/CPU/MPS support

---

## 🎯 Objectives

### MVP Scope

**What's Included:**
- ✅ 1 robot (Franka Emika Panda)
- ✅ 1 scene (Table pick-and-place)
- ✅ 2 VLA models (OpenVLA 7B, Octo-Base)
- ✅ Adapter pattern (model-specific input/output handling)
- ✅ Stateless MuJoCo (XML string composition)
- ✅ Episode execution (max 15 seconds @ 5 Hz = 75 steps)
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
Backend → HTTP POST /execute (to specific VLA server)
  ↓
VLA Server (per model):
  1. Parse request (robot_id, scene_id, instruction)
  2. Compose MuJoCo XML (robot + scene, stateless)
  3. Create MuJoCo environment (from XML string)
  4. Get model adapter (OpenVLA or Octo specific)
  5. Run episode loop (max 15s @ 5 Hz):
     a. Get observation (camera + proprioception)
     b. Adapter preprocessing (model-specific format)
     c. VLA inference (action from observation + instruction)
     d. Adapter postprocessing (standardize to 8-dim action)
     e. MuJoCo step (apply action, update state)
     f. Record (action, qpos, qvel, time)
     g. Check termination (time limit)
  6. Return episode data
  ↓
Response: {actions, states, duration_ms, metadata}
```

### Component Diagram

```
┌────────────────────────────────────────────────────────┐
│         VLA Server (FastAPI) - Per Model               │
│         Started with VLA_MODEL_ID=openvla-7b           │
│                                                         │
│  ┌──────────────────────────────────────────────────┐ │
│  │  API Layer (api/execute.py)                       │ │
│  │  - POST /execute                                  │ │
│  │  - GET /health                                    │ │
│  │  - GET /info (model info)                        │ │
│  └────────────┬─────────────────────────────────────┘ │
│               │                                         │
│  ┌────────────▼─────────────────────────────────────┐ │
│  │  Execution Service                                │ │
│  │  (services/execution_service.py)                  │ │
│  │  - Coordinate MuJoCo + VLA Adapter                │ │
│  │  - Episode loop management                        │ │
│  │  - Termination logic                              │ │
│  └───────┬──────────────┬───────────────────────────┘ │
│          │              │                              │
│  ┌───────▼──────┐  ┌───▼─────────────────────────┐   │
│  │ MuJoCo Env   │  │ VLA Adapter (Factory)       │   │
│  │ (Stateless)  │  │                             │   │
│  │              │  │ - get_adapter(model_id)     │   │
│  │ - from_xml() │  │                             │   │
│  │ - step()     │  ├─────────────────────────────┤   │
│  │ - get_obs()  │  │ OpenVLAAdapter              │   │
│  │ - get_state()│  │ - preprocess_obs()          │   │
│  └──────────────┘  │ - preprocess_instruction()  │   │
│                    │ - predict()                 │   │
│  ┌──────────────┐  │ - postprocess_action()      │   │
│  │ Model Loader │  ├─────────────────────────────┤   │
│  │ (config/)    │  │ OctoAdapter                 │   │
│  │              │  │ - preprocess_obs()          │   │
│  │ - get_xml()  │  │ - preprocess_instruction()  │   │
│  │   (robot+    │  │ - predict()                 │   │
│  │    scene)    │  │ - postprocess_action()      │   │
│  └──────────────┘  └─────────────────────────────┘   │
└────────────────────────────────────────────────────────┘
```

### Multi-Server Architecture

```
Backend (config/models.yaml):
├── openvla-7b → http://localhost:8001
└── octo-base  → http://localhost:8002

VLA Server Instance 1 (Port 8001):
VLA_MODEL_ID=openvla-7b
└── Uses OpenVLAAdapter internally

VLA Server Instance 2 (Port 8002):
VLA_MODEL_ID=octo-base
└── Uses OctoAdapter internally
```

---

## 📦 Project Structure

```
mujoco-web-vla/
├── config/
│   ├── models.yaml                    # VLA model endpoints
│   └── mujoco/
│       ├── template.xml               # Base MuJoCo template
│       ├── robots/
│       │   ├── franka.xml            # Franka robot body
│       │   └── widowx.xml            # (Future)
│       └── scenes/
│           ├── table.xml             # Table scene body
│           └── kitchen.xml           # (Future)
│
└── vla-server/
    ├── src/vla_server/
    │   ├── __init__.py
    │   ├── main.py                   # FastAPI app
    │   │
    │   ├── config/
    │   │   ├── __init__.py
    │   │   ├── settings.py           # Settings (model_id, port, etc.)
    │   │   └── model_loader.py       # XML composition logic
    │   │
    │   ├── api/
    │   │   ├── __init__.py
    │   │   ├── execute.py            # POST /execute
    │   │   ├── health.py             # GET /health
    │   │   └── info.py               # GET /info
    │   │
    │   ├── adapters/
    │   │   ├── __init__.py           # get_adapter() factory
    │   │   ├── base.py               # VLAModelAdapter ABC
    │   │   ├── openvla_adapter.py    # OpenVLA specific
    │   │   └── octo_adapter.py       # Octo specific
    │   │
    │   ├── services/
    │   │   ├── __init__.py
    │   │   ├── execution_service.py  # Main orchestration
    │   │   └── mujoco_env.py         # Stateless MuJoCo (XML input)
    │   │
    │   ├── schemas/
    │   │   ├── __init__.py
    │   │   ├── execute.py            # ExecuteRequest/Response
    │   │   └── common.py             # Shared schemas
    │   │
    │   └── utils/
    │       ├── __init__.py
    │       └── device.py             # GPU/CPU/MPS detection
    │
    ├── tests/
    │   ├── conftest.py               # Pytest fixtures
    │   ├── test_model_loader.py      # XML composition tests (10 tests)
    │   ├── test_mujoco_env.py        # MuJoCo tests (15 tests)
    │   ├── test_adapters.py          # Adapter tests (20 tests)
    │   ├── test_execution_service.py # Integration tests (20 tests)
    │   └── test_execute_api.py       # API tests (10 tests)
    │
    ├── model_cache/                  # HuggingFace cache
    │
    ├── pyproject.toml                # Dependencies
    ├── README.md
    └── .env.example
```

---

## 🔧 Implementation Guide

### Step 1: Root Config Setup

**Create config/mujoco/ structure:**

```bash
mkdir -p config/mujoco/{robots,scenes}
```

**config/mujoco/template.xml:**

```xml
<mujoco model="{model_name}">
  <compiler angle="radian"/>
  <option timestep="0.002"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" pos="0 0 0" size="0 0 .05" type="plane" material="grid"/>

    {scene_body}
    {robot_body}
  </worldbody>
</mujoco>
```

**config/mujoco/robots/franka.xml:**

```xml
<body name="franka_base" pos="0 0 0">
  <inertial pos="0 0 0" mass="4" diaginertia="0.4 0.4 0.4"/>
  <geom type="cylinder" size="0.06 0.05" rgba="0.7 0.7 0.7 1"/>

  <!-- Simplified Franka arm for MVP -->
  <body name="link1" pos="0 0 0.1">
    <joint name="joint1" type="hinge" axis="0 0 1"/>
    <geom type="cylinder" size="0.05 0.1" rgba="0.9 0.9 0.9 1"/>

    <body name="link2" pos="0 0 0.2">
      <joint name="joint2" type="hinge" axis="0 1 0"/>
      <geom type="cylinder" size="0.045 0.1" rgba="0.9 0.9 0.9 1"/>

      <!-- Add remaining joints... -->
      <!-- Total: 7 joints + gripper (8 actuators) -->
    </body>
  </body>
</body>
```

**config/mujoco/scenes/table.xml:**

```xml
<body name="table" pos="0.5 0 0">
  <geom name="table_top" type="box" size="0.4 0.4 0.02" pos="0 0 0.4" rgba="0.8 0.6 0.4 1"/>
  <geom name="table_leg1" type="cylinder" size="0.03 0.2" pos="-0.35 -0.35 0.2"/>
  <geom name="table_leg2" type="cylinder" size="0.03 0.2" pos="-0.35 0.35 0.2"/>
  <geom name="table_leg3" type="cylinder" size="0.03 0.2" pos="0.35 -0.35 0.2"/>
  <geom name="table_leg4" type="cylinder" size="0.03 0.2" pos="0.35 0.35 0.2"/>

  <!-- Add objects for manipulation -->
  <body name="red_cube" pos="0 0 0.45">
    <joint type="free"/>
    <geom type="box" size="0.03 0.03 0.03" rgba="1 0 0 1" mass="0.1"/>
  </body>
</body>
```

---

### Step 2: VLA Server Configuration

**vla-server/src/vla_server/config/settings.py:**

```python
"""
Configuration for VLA Server
Uses Pydantic Settings for environment variable management
"""

from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict

# Root .env file location (mujoco-web-vla/.env)
ROOT_ENV_FILE = Path(__file__).parent.parent.parent.parent.parent / ".env"


class Settings(BaseSettings):
    """VLA Server settings"""

    model_config = SettingsConfigDict(env_prefix="VLA_", env_file=str(ROOT_ENV_FILE))

    # Server
    host: str = "0.0.0.0"
    port: int = 8001
    reload: bool = True

    # Model (REQUIRED - determines which adapter to use)
    model_id: str  # "openvla-7b", "octo-base"

    # Models & Paths
    vla_model_cache: str = "./model_cache"
    device: str = "auto"  # auto (cuda → mps → cpu), cuda, cpu, mps

    # Execution - based on RT-1/Octo standards
    control_frequency: float = 5.0  # Hz (RT-1/Octo standard: 3-5 Hz)
    max_episode_seconds: float = 15.0  # seconds (typical task duration)

    # Logging
    log_level: str = "INFO"


settings = Settings()
```

**vla-server/src/vla_server/config/model_loader.py:**

```python
"""
MuJoCo model XML composition
Dynamically combines robot and scene XMLs
"""

from pathlib import Path


# Root config directory
ROOT_CONFIG = Path(__file__).parent.parent.parent.parent.parent / "config" / "mujoco"


def get_model_xml(robot_id: str, scene_id: str) -> str:
    """
    Dynamically compose MuJoCo XML from robot and scene

    Args:
        robot_id: Robot identifier (e.g., "franka")
        scene_id: Scene identifier (e.g., "table")

    Returns:
        Complete MuJoCo XML string

    Raises:
        FileNotFoundError: If robot or scene XML not found
    """
    # Load template
    template_path = ROOT_CONFIG / "template.xml"
    if not template_path.exists():
        raise FileNotFoundError(f"Template not found: {template_path}")
    template = template_path.read_text()

    # Load robot body
    robot_path = ROOT_CONFIG / "robots" / f"{robot_id}.xml"
    if not robot_path.exists():
        raise FileNotFoundError(f"Robot not found: {robot_path}")
    robot_xml = robot_path.read_text()

    # Load scene body
    scene_path = ROOT_CONFIG / "scenes" / f"{scene_id}.xml"
    if not scene_path.exists():
        raise FileNotFoundError(f"Scene not found: {scene_path}")
    scene_xml = scene_path.read_text()

    # Compose final XML
    final_xml = template.format(
        model_name=f"{robot_id}_{scene_id}",
        robot_body=robot_xml,
        scene_body=scene_xml
    )

    return final_xml
```

---

### Step 3: Stateless MuJoCo Environment

**vla-server/src/vla_server/services/mujoco_env.py:**

```python
"""
MuJoCo environment management for VLA execution
Stateless design - accepts XML string directly
"""

import mujoco
import numpy as np


class MuJoCoEnvironment:
    """Stateless MuJoCo simulation environment"""

    def __init__(self, xml_string: str):
        """
        Initialize MuJoCo environment from XML string

        Args:
            xml_string: Complete MuJoCo XML model
        """
        # Load model from XML string (stateless!)
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)

        # Renderer for observations (224x224 for VLA models)
        self.renderer = mujoco.Renderer(self.model, height=224, width=224)

        # Store initial state
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()

    def reset(self) -> dict:
        """
        Reset environment to initial state

        Returns:
            Observation after reset
        """
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = self.initial_qvel
        self.data.time = 0.0
        mujoco.mj_forward(self.model, self.data)

        return self.get_observation()

    def step(self, action: list[float]) -> dict:
        """
        Step simulation with action

        Args:
            action: 8-dim action vector (7 joints + gripper)

        Returns:
            Observation after step
        """
        # Convert to numpy array
        action_array = np.array(action, dtype=np.float32)

        # Apply action to actuators (truncate or pad as needed)
        num_actuators = self.model.nu
        if len(action_array) >= num_actuators:
            self.data.ctrl[:] = action_array[:num_actuators]
        else:
            self.data.ctrl[:len(action_array)] = action_array
            self.data.ctrl[len(action_array):] = 0.0

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
            Dictionary with qpos, qvel, time (as lists for JSON serialization)
        """
        return {
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "time": float(self.data.time),
        }
```

---

### Step 4: VLA Adapter Pattern

**vla-server/src/vla_server/adapters/base.py:**

```python
"""
Abstract base class for VLA model adapters
Defines interface for model-specific preprocessing/postprocessing
"""

from abc import ABC, abstractmethod
from typing import Any
import numpy as np


class VLAModelAdapter(ABC):
    """Abstract VLA model adapter"""

    @abstractmethod
    def load_model(self, model_id: str, device: str, cache_dir: str):
        """
        Load VLA model

        Args:
            model_id: Model identifier
            device: torch device (cuda, mps, cpu)
            cache_dir: HuggingFace cache directory
        """
        pass

    @abstractmethod
    def preprocess_observation(self, obs: dict) -> Any:
        """
        Preprocess observation to model-specific format

        Args:
            obs: Standard observation dict
                - image: (H, W, 3) RGB uint8
                - qpos: (n,) joint positions
                - qvel: (n,) joint velocities

        Returns:
            Model-specific observation format
        """
        pass

    @abstractmethod
    def preprocess_instruction(self, instruction: str) -> Any:
        """
        Preprocess text instruction to model-specific format

        Args:
            instruction: Natural language instruction

        Returns:
            Model-specific instruction format
        """
        pass

    @abstractmethod
    def predict(self, obs: Any, instruction: Any) -> Any:
        """
        Run model inference

        Args:
            obs: Preprocessed observation
            instruction: Preprocessed instruction

        Returns:
            Raw model output
        """
        pass

    @abstractmethod
    def postprocess_action(self, raw_action: Any) -> list[float]:
        """
        Postprocess model output to standard 8-dim action

        Args:
            raw_action: Raw model output

        Returns:
            8-dim action list [j1, j2, ..., j7, gripper]
        """
        pass
```

**vla-server/src/vla_server/adapters/openvla_adapter.py:**

```python
"""
OpenVLA adapter
Model-specific preprocessing/postprocessing for OpenVLA 7B
"""

import torch
from transformers import AutoModel, AutoProcessor
from .base import VLAModelAdapter


class OpenVLAAdapter(VLAModelAdapter):
    """Adapter for OpenVLA 7B model"""

    def load_model(self, model_id: str, device: str, cache_dir: str):
        """Load OpenVLA model from HuggingFace"""
        hub_id = "openvla/openvla-7b"

        self.device = device
        self.processor = AutoProcessor.from_pretrained(hub_id, cache_dir=cache_dir)
        self.model = AutoModel.from_pretrained(
            hub_id,
            cache_dir=cache_dir,
            torch_dtype=torch.float16 if device == "cuda" else torch.float32
        )
        self.model = self.model.to(device)
        self.model.eval()

    def preprocess_observation(self, obs: dict):
        """OpenVLA expects 256x256 images"""
        from PIL import Image

        # Convert to PIL Image and resize
        image = Image.fromarray(obs["image"])
        image_resized = image.resize((256, 256))

        return {
            "image": image_resized,
            "qpos": obs["qpos"],
            "qvel": obs["qvel"],
        }

    def preprocess_instruction(self, instruction: str):
        """OpenVLA uses standard text encoding"""
        return instruction

    def predict(self, obs, instruction):
        """Run OpenVLA inference"""
        # Process inputs
        inputs = self.processor(
            images=obs["image"],
            text=instruction,
            return_tensors="pt"
        )
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Inference
        with torch.no_grad():
            outputs = self.model(**inputs)
            action = outputs.action  # Model-specific output format

        return action

    def postprocess_action(self, raw_action):
        """Convert OpenVLA output to 8-dim action"""
        # OpenVLA outputs 8-dim action directly
        action_np = raw_action.cpu().numpy()
        if action_np.ndim > 1:
            action_np = action_np[0]  # Remove batch dimension

        return action_np.tolist()
```

**vla-server/src/vla_server/adapters/octo_adapter.py:**

```python
"""
Octo adapter
Model-specific preprocessing/postprocessing for Octo-Base
"""

import torch
from transformers import AutoModel, AutoProcessor
from .base import VLAModelAdapter


class OctoAdapter(VLAModelAdapter):
    """Adapter for Octo-Base model"""

    def load_model(self, model_id: str, device: str, cache_dir: str):
        """Load Octo model from HuggingFace"""
        hub_id = "octo-models/octo-base"

        self.device = device
        self.processor = AutoProcessor.from_pretrained(hub_id, cache_dir=cache_dir)
        self.model = AutoModel.from_pretrained(hub_id, cache_dir=cache_dir)
        self.model = self.model.to(device)
        self.model.eval()

    def preprocess_observation(self, obs: dict):
        """Octo expects 224x224 images (already correct size)"""
        return {
            "image": obs["image"],
            "qpos": obs["qpos"],
        }

    def preprocess_instruction(self, instruction: str):
        """Octo uses standard text encoding"""
        return instruction

    def predict(self, obs, instruction):
        """Run Octo inference"""
        # Process inputs
        inputs = self.processor(
            images=obs["image"],
            text=instruction,
            return_tensors="pt"
        )
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Inference
        with torch.no_grad():
            outputs = self.model(**inputs)
            action = outputs.action

        return action

    def postprocess_action(self, raw_action):
        """Convert Octo output to 8-dim action"""
        action_np = raw_action.cpu().numpy()
        if action_np.ndim > 1:
            action_np = action_np[0]

        # Octo outputs 7-dim (no gripper), pad with 0
        if len(action_np) == 7:
            action_np = np.append(action_np, 0.0)

        return action_np.tolist()
```

**vla-server/src/vla_server/adapters/__init__.py:**

```python
"""
VLA adapter factory
Returns appropriate adapter based on model_id
"""

from .base import VLAModelAdapter
from .openvla_adapter import OpenVLAAdapter
from .octo_adapter import OctoAdapter


def get_adapter(model_id: str) -> VLAModelAdapter:
    """
    Get VLA adapter for given model

    Args:
        model_id: Model identifier

    Returns:
        Appropriate VLAModelAdapter instance

    Raises:
        ValueError: If model_id not recognized
    """
    adapters = {
        "openvla-7b": OpenVLAAdapter,
        "octo-base": OctoAdapter,
    }

    adapter_class = adapters.get(model_id)
    if not adapter_class:
        raise ValueError(f"Unknown model_id: {model_id}. Available: {list(adapters.keys())}")

    return adapter_class()


__all__ = ["VLAModelAdapter", "get_adapter", "OpenVLAAdapter", "OctoAdapter"]
```

---

### Step 5: Execution Service

**vla-server/src/vla_server/services/execution_service.py:**

```python
"""
Execution service - orchestrates MuJoCo + VLA Adapter
"""

import time
from ..config import settings
from ..config.model_loader import get_model_xml
from ..adapters import get_adapter
from .mujoco_env import MuJoCoEnvironment
from ..schemas.execute import ExecuteRequest, ExecuteResponse, State


class ExecutionService:
    """Service for executing VLA episodes"""

    def __init__(self):
        # Load adapter for this server's model
        self.adapter = get_adapter(settings.model_id)
        self.adapter.load_model(
            model_id=settings.model_id,
            device=settings.device,
            cache_dir=settings.vla_model_cache
        )

        # Detect actual device
        if settings.device == "auto":
            import torch
            if torch.cuda.is_available():
                self.device = "cuda"
            elif torch.backends.mps.is_available():
                self.device = "mps"
            else:
                self.device = "cpu"
        else:
            self.device = settings.device

    async def execute(self, request: ExecuteRequest) -> ExecuteResponse:
        """
        Execute VLA episode

        Args:
            request: Execution request (robot_id, scene_id, instruction)

        Returns:
            Episode data with actions and states
        """
        start_time = time.time()

        # 1. Compose MuJoCo XML (stateless)
        xml_string = get_model_xml(request.robot_id, request.scene_id)

        # 2. Create MuJoCo environment (from XML string)
        env = MuJoCoEnvironment(xml_string)

        # 3. Reset environment
        env.reset()

        # 4. Calculate max steps
        max_steps = int(settings.max_episode_seconds * settings.control_frequency)

        # 5. Run episode loop
        actions = []
        states = []

        for step in range(max_steps):
            # Get observation
            obs = env.get_observation()

            # Adapter preprocessing
            obs_processed = self.adapter.preprocess_observation(obs)
            instruction_processed = self.adapter.preprocess_instruction(request.instruction)

            # VLA inference
            raw_action = self.adapter.predict(obs_processed, instruction_processed)

            # Adapter postprocessing
            action = self.adapter.postprocess_action(raw_action)

            # Step simulation
            env.step(action)

            # Record
            actions.append(action)
            states.append(State(**env.get_state()))

        # 6. Calculate duration
        duration_ms = int((time.time() - start_time) * 1000)

        # 7. Return response
        return ExecuteResponse(
            actions=actions,
            states=states,
            duration_ms=duration_ms,
            metadata={
                "model_id": settings.model_id,
                "num_steps": len(actions),
                "max_steps": max_steps,
                "control_frequency": settings.control_frequency,
                "device": self.device,
            },
        )
```

---

### Step 6: Schemas

**vla-server/src/vla_server/schemas/execute.py:**

```python
"""
Request/Response schemas for /execute endpoint
"""

from pydantic import BaseModel, Field


class ExecuteRequest(BaseModel):
    """Request to execute VLA model in MuJoCo environment"""

    # Note: model_id is NOT in request - server already knows its model
    robot_id: str = Field(..., description="Robot type (e.g., 'franka')")
    scene_id: str = Field(..., description="Scene type (e.g., 'table')")
    instruction: str = Field(..., min_length=1, description="Natural language instruction")

    class Config:
        json_schema_extra = {
            "example": {
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

    actions: list[list[float]] = Field(..., description="Action sequence")
    states: list[State] = Field(..., description="State sequence (same length as actions)")
    duration_ms: int = Field(..., description="Execution duration in milliseconds")
    metadata: dict = Field(
        default_factory=dict,
        description="Additional metadata (model_id, num_steps, device, etc.)",
    )

    class Config:
        json_schema_extra = {
            "example": {
                "actions": [[0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]],
                "states": [{"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}],
                "duration_ms": 5120,
                "metadata": {
                    "model_id": "openvla-7b",
                    "num_steps": 75,
                    "max_steps": 75,
                    "control_frequency": 5.0,
                    "device": "cuda"
                },
            }
        }
```

---

### Step 7: API Endpoint

**vla-server/src/vla_server/api/execute.py:**

```python
"""
Execute endpoint - POST /execute
"""

from fastapi import APIRouter, HTTPException, status
from ..schemas.execute import ExecuteRequest, ExecuteResponse
from ..services.execution_service import ExecutionService


router = APIRouter(prefix="", tags=["execution"])

# Global service instance (model loaded once at startup)
execution_service = ExecutionService()


@router.post("/execute", response_model=ExecuteResponse)
async def execute_vla(request: ExecuteRequest):
    """
    Execute VLA model in MuJoCo environment

    Args:
        request: Execution request (robot, scene, instruction)

    Returns:
        Episode data with actions and states
    """
    try:
        response = await execution_service.execute(request)
        return response

    except FileNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot or scene not found: {str(e)}",
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


@router.get("/info")
async def get_server_info():
    """Get information about this VLA server instance"""
    from ..config import settings

    return {
        "model_id": settings.model_id,
        "device": execution_service.device,
        "max_episode_seconds": settings.max_episode_seconds,
        "control_frequency": settings.control_frequency,
    }
```

---

## 📅 Implementation Timeline

### Week 3: Core Infrastructure
- [x] Root config setup (config/mujoco/) ✅ Phase 1
- [x] XML composition logic (10 tests) ✅ Phase 1
- [x] Stateless MuJoCo environment (16 tests) ✅ Phase 1
- [ ] VLA adapter base class (Phase 2)
- [ ] OpenVLA adapter (10 tests) (Phase 2)
- [ ] Octo adapter (10 tests) (Phase 2)

### Week 4: Integration & Testing
- [ ] Execution service with adapters (20 tests)
- [ ] API endpoints (10 tests)
- [ ] Full integration tests
- [ ] Multi-server testing (2 instances)
- [ ] Error handling

### Week 5: Optimization & Documentation
- [ ] Performance tuning
- [ ] Model caching optimization
- [ ] API documentation (OpenAPI)
- [ ] Deployment guide
- [ ] README

---

## 🧪 Testing Strategy

### Unit Tests (75 total)

**XML Composition (10 tests):**
- Load template/robot/scene
- Compose final XML
- Error handling (missing files)

**MuJoCo Environment (15 tests):**
- Initialize from XML string
- Reset environment
- Step simulation
- Get observation/state
- Error handling

**Adapters (20 tests):**
- OpenVLA preprocessing (5)
- OpenVLA postprocessing (5)
- Octo preprocessing (5)
- Octo postprocessing (5)

**Execution Service (20 tests):**
- End-to-end execution
- Adapter integration
- XML composition integration
- Error scenarios

**API (10 tests):**
- POST /execute success
- GET /info
- Error handling (404, 400, 500)

---

## 🚀 Deployment

### Starting VLA Servers

```bash
# Start OpenVLA server (port 8001)
cd vla-server
VLA_MODEL_ID=openvla-7b VLA_PORT=8001 uv run uvicorn vla_server.main:app --reload

# Start Octo server (port 8002)
VLA_MODEL_ID=octo-base VLA_PORT=8002 uv run uvicorn vla_server.main:app --reload
```

### Testing Endpoints

```bash
# Test OpenVLA server
curl -X POST http://localhost:8001/execute \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "franka",
    "scene_id": "table",
    "instruction": "Pick up the red cube"
  }'

# Get server info
curl http://localhost:8001/info
```

### Backend Integration

Backend uses `config/models.yaml` to route requests to appropriate VLA server:

```yaml
models:
  - id: openvla-7b
    base_url: http://localhost:8001  # OpenVLA server

  - id: octo-base
    base_url: http://localhost:8002  # Octo server
```

---

## 📊 Success Criteria

**Phase 1 Complete (Infrastructure):** ✅
1. ✅ Root config/mujoco/ structure created
2. ✅ XML composition works (dynamic robot+scene) - 10 tests passing
3. ✅ Stateless MuJoCo environment (XML string input) - 16 tests passing

**Phase 2 Remaining (VLA Integration):**
4. ⏳ Adapter pattern implemented (OpenVLA, Octo)
5. ⏳ Multi-server deployment works
6. ⏳ Episodes generated (75 steps @ 5 Hz, 15s)
7. ⏳ Backend integration successful
8. ⏳ All tests pass (75 tests target)
9. ⏳ MacBook compatible (CPU/MPS)

---

## 🔄 Migration from Previous Architecture

**Changes from v1:**
1. ❌ Removed: `vla_model.py` (monolithic model manager)
2. ✅ Added: `adapters/` (model-specific preprocessing)
3. ✅ Added: `config/model_loader.py` (XML composition)
4. ✅ Changed: `MuJoCoEnvironment.__init__(xml_string)` (was file path)
5. ✅ Changed: API request - no `model_id` field (server knows its model)
6. ✅ Added: `VLA_MODEL_ID` environment variable (server startup)

**Why?**
- **Extensibility**: Easy to add new models (just add adapter)
- **Stateless**: No file system dependency for MuJoCo
- **Scalability**: Each model runs as separate server instance
- **Testability**: Mock XML strings for tests

---

## 📚 References

### Models
- **OpenVLA:** https://openvla.github.io/
- **Octo:** https://octo-models.github.io/

### Documentation
- **ADR-003:** VLA Server Separation
- **config/models.yaml:** VLA model endpoints
- **ROADMAP.md:** Development timeline
- **001_MVP.md:** Overall MVP scope

---

**Created:** 2025-01-06
**Last Updated:** 2025-01-07
**Status:** Phase 1 Complete - Infrastructure (26 tests passing)
**Next Phase:** Phase 2 - VLA Adapters (OpenVLA, Octo)
