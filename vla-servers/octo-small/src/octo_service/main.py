"""
Octo-Small VLA service - FastAPI server

Provides:
- POST /execute: Full episode execution
- GET /health: Health check
- GET /info: Model information

Note:
    Requires nest_asyncio to allow orbax.checkpoint's nested asyncio.run() calls.
    Must run uvicorn with --loop asyncio flag to avoid uvloop conflict.
    See README.md for details.
"""

import nest_asyncio
from vla_server_base import create_vla_app

from octo_service.adapters.octo_small_adapter import OctoSmallAdapter

# Apply nest_asyncio to allow nested asyncio.run() calls
# This is required by orbax.checkpoint which calls asyncio.run() during model loading
nest_asyncio.apply()

# Create adapter
adapter = OctoSmallAdapter()

# Create FastAPI app using common factory
app = create_vla_app(
    adapter=adapter,
    service_name="Octo-Small VLA Service",
    service_version="0.1.0",
    max_steps=75,  # 15s @ 5Hz
)


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


if __name__ == "__main__":
    import uvicorn

    # Use asyncio loop instead of uvloop to avoid nest_asyncio conflicts
    # orbax.checkpoint applies nest_asyncio which conflicts with uvloop
    uvicorn.run(app, host="0.0.0.0", port=8002, loop="asyncio")
