"""
Mock VLA service - FastAPI server

Provides:
- POST /predict: VLA inference
- GET /health: Health check
- GET /info: Model information
"""

from typing import Any

import numpy as np
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from vla_server_base import MuJoCoEnvironment

from vla_mock.adapters.mock_adapter import MockVLAAdapter

app = FastAPI(title="Mock VLA Service", version="0.1.0")

# Global adapter instance
adapter = MockVLAAdapter()
adapter.load_model(model_id="mock-vla", device="cpu", cache_dir="/tmp")


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
    Run VLA inference

    Args:
        request: Prediction request with instruction and MuJoCo XML

    Returns:
        8-dim action
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
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health")
async def health() -> dict[str, str]:
    """Health check endpoint"""
    return {"status": "healthy", "model": "mock-vla"}


@app.get("/info")
async def info() -> dict[str, Any]:
    """Model information endpoint"""
    return {
        "model_id": adapter.model_id,
        "device": adapter.device,
        "loaded": adapter.model_loaded,
        "type": "mock",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8001)
