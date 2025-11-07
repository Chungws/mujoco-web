# Octo-Small VLA Service

FastAPI service for Octo-Small (27M parameters) VLA model inference.

## Overview

This service provides HTTP API for Octo-Small model predictions:
- **Model**: `hf://rail-berkeley/octo-small-1.5` (27M params)
- **Framework**: JAX 0.4.20
- **Python**: 3.10-3.11
- **Performance**: ~17 it/sec on NVIDIA 4090

## Architecture

Uses microservice pattern with:
- `vla-server-base`: Common library (MuJoCo, adapters)
- Independent Python environment and dependencies
- Isolated from other VLA model services

## Installation

```bash
cd vla-servers/octo-small

# Development (CPU only)
uv sync

# GPU support (CUDA 12)
uv sync --extra cuda12

# GPU support (CUDA 13)
uv sync --extra cuda13
```

### Manual GPU Installation (Alternative)

If you need a specific CUDA version:

```bash
# After uv sync, install JAX with CUDA manually
pip install --upgrade "jax[cuda12]"  # CUDA 12
pip install --upgrade "jax[cuda13]"  # CUDA 13
```

### Verify GPU Detection

```bash
uv run python -c "import jax; print(f'JAX devices: {jax.devices()}'); print(f'Device platform: {jax.devices()[0].platform}')"
```

Expected output:
- CPU: `Device platform: cpu`
- GPU: `Device platform: gpu`

## Running

```bash
# Development server (port 8002)
uv run uvicorn octo_service.main:app --reload --port 8002

# Production
uv run uvicorn octo_service.main:app --host 0.0.0.0 --port 8002
```

## API Endpoints

### POST /predict
Run VLA inference with Octo-Small model.

**Request:**
```json
{
  "instruction": "pick up the red cube",
  "xml_string": "<mujoco>...</mujoco>"
}
```

**Response:**
```json
{
  "action": [0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
}
```

### GET /health
Health check endpoint.

### GET /info
Model information and status.

## Implementation Details

### Octo-Small Specifications
- **Image input**: 256×256 RGB (resized from MuJoCo 224×224)
- **Action output**: 7-dim (end-effector control)
- **Action chunking**: Predicts 4 actions, uses first (receding horizon)
- **Observation window**: 2 timesteps (first step uses pad mask)

### Adapter Pattern
`OctoSmallAdapter` implements `VLAModelAdapter` interface:
- `load_model()`: Load from HuggingFace
- `preprocess_observation()`: Resize to 256×256, add batch/time dims
- `preprocess_instruction()`: Create task with language
- `predict()`: JAX inference with action chunking
- `postprocess_action()`: Extract first action, pad to 8-dim

### Action Handling
- Octo outputs: `[batch=1, pred_horizon=4, action_dim=7]`
- We use: First action `[7,]` → pad gripper → `[8,]`
- Standard format: `[j1, j2, j3, j4, j5, j6, j7, gripper]`

## Testing

```bash
# Run tests
uv run pytest -v

# Run with coverage
uv run pytest --cov=octo_service --cov-report=term-missing
```

## References

- **Octo Model**: https://octo-models.github.io/
- **GitHub**: https://github.com/octo-models/octo
- **HuggingFace**: https://huggingface.co/rail-berkeley/octo-small-1.5
- **Paper**: https://arxiv.org/abs/2405.12213

## Notes

- **MPS not supported**: JAX has limited Apple Silicon support
- **CPU mode**: Works for development (slower inference)
- **GPU recommended**: For production use with NVIDIA GPU
