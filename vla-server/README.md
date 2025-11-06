# VLA Execution Server

Independent microservice for executing VLA (Vision-Language-Action) models in MuJoCo simulated environments.

## Architecture

```
Backend (Orchestrator) → HTTP → VLA Server (Execution)
```

## Features

- MuJoCo simulation (Franka Emika Panda robot)
- VLA model inference (Octo-Small, SmolVLA)
- Episode generation (up to 50 steps)
- State-based recording (qpos, qvel, time)
- GPU/CPU/MPS support

## Quick Start

```bash
# Install dependencies
uv sync --all-extras

# Download Franka robot model from MuJoCo Menagerie
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/mujoco_menagerie
mkdir -p models/robots/franka models/scenes/table
cp -r /tmp/mujoco_menagerie/franka_emika_panda/* models/robots/franka/

# Run server
uv run uvicorn vla_server.main:app --reload --port 8001

# Run tests
uv run pytest -s
```

## API

### POST /execute

Generate episode from instruction.

**Request:**
```json
{
  "model_id": "octo-small",
  "robot_id": "franka",
  "scene_id": "table",
  "instruction": "Pick up the red cube"
}
```

**Response:**
```json
{
  "actions": [[0.1, 0.2, ...], ...],
  "states": [
    {"qpos": [...], "qvel": [...], "time": 0.0},
    ...
  ],
  "duration_ms": 5120,
  "metadata": {
    "num_steps": 35,
    "max_steps": 50,
    "early_termination": true
  }
}
```

## Documentation

See: `WORKSPACE/FEATURES/002_VLA_Server.md` for complete specification.

## Development

- Week 3: Infrastructure (MuJoCo + Models) - 27 tests
- Week 4: Core Logic (Execution + API) - 30 tests
- Week 5: Backend Integration

Total: 57 tests (TDD)
