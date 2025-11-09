"""
Octo-Small VLA service - FastAPI server

Provides:
- POST /predict: VLA inference
- GET /health: Health check
- GET /info: Model information

Note:
    Requires nest_asyncio to allow orbax.checkpoint's nested asyncio.run() calls.
    Must run uvicorn with --loop asyncio flag to avoid uvloop conflict.
    See README.md for details.
"""

from typing import Any

import nest_asyncio
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from vla_server_base import MuJoCoEnvironment

from octo_service.adapters.octo_small_adapter import OctoSmallAdapter

# Apply nest_asyncio to allow nested asyncio.run() calls
# This is required by orbax.checkpoint which calls asyncio.run() during model loading
nest_asyncio.apply()

app = FastAPI(
    title="Octo-Small VLA Service",
    version="0.1.0",
    description="Octo-Small (27M) VLA model inference service",
)

# Global adapter instance (model loaded once at startup)
adapter = OctoSmallAdapter()


@app.on_event("startup")
async def startup_event():
    """Load model on startup"""
    try:
        print("Loading Octo-Small model...")
        adapter.load_model(
            model_id="octo-small",
            device="cpu",  # JAX device management via environment
            cache_dir="/tmp/octo_cache",
        )
        print("✓ Octo-Small model loaded successfully")
    except Exception as e:
        print(f"✗ Failed to load Octo-Small model: {e}")
        raise


class PredictRequest(BaseModel):
    """Prediction request"""

    instruction: str
    xml_string: str


class PredictResponse(BaseModel):
    """Prediction response"""

    action: list[float]


@app.post("/predict", response_model=PredictResponse)
async def predict(request: PredictRequest) -> PredictResponse:
    """
    Run VLA inference with Octo-Small

    Args:
        request: Prediction request with instruction and MuJoCo XML

    Returns:
        8-dim action

    Raises:
        HTTPException: 400 for invalid input, 500 for inference errors
    """
    try:
        # Create environment and get observation
        env = MuJoCoEnvironment(request.xml_string)
        obs = env.reset()

        # Preprocess
        processed_obs = adapter.preprocess_observation(obs)
        processed_instruction = adapter.preprocess_instruction(request.instruction)

        # Predict
        raw_action = adapter.predict(processed_obs, processed_instruction)

        # Postprocess
        action = adapter.postprocess_action(raw_action)

        return PredictResponse(action=action)

    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e


@app.get("/health")
async def health() -> dict[str, str]:
    """Health check endpoint"""
    model_status = "loaded" if adapter.model is not None else "not_loaded"
    return {
        "status": "healthy" if adapter.model is not None else "unhealthy",
        "model": "octo-small",
        "model_status": model_status,
    }


@app.get("/info")
async def info() -> dict[str, Any]:
    """Model information endpoint"""
    return {
        "model_id": adapter.model_id or "not_loaded",
        "device": adapter.device or "unknown",
        "loaded": adapter.model is not None,
        "type": "octo-small",
        "parameters": "27M",
        "framework": "JAX",
        "version": "1.5",
    }


if __name__ == "__main__":
    import uvicorn

    # Use asyncio loop instead of uvloop to avoid nest_asyncio conflicts
    # orbax.checkpoint applies nest_asyncio which conflicts with uvloop
    uvicorn.run(app, host="0.0.0.0", port=8002, loop="asyncio")
