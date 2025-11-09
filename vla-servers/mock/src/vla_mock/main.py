"""
Mock VLA service - FastAPI server

Provides:
- POST /execute: Full episode execution
- GET /health: Health check
- GET /info: Model information
"""

from vla_server_base import create_vla_app

from vla_mock.adapters.mock_adapter import MockVLAAdapter

# Create and load adapter
adapter = MockVLAAdapter()
adapter.load_model(model_id="mock-vla", device="cpu", cache_dir="/tmp")

# Create FastAPI app using common factory
app = create_vla_app(
    adapter=adapter,
    service_name="Mock VLA Service",
    service_version="0.1.0",
    max_steps=75,  # 15s @ 5Hz
)

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8001)
