"""
Common FastAPI application factory for VLA services

Provides standardized endpoints:
- POST /execute: Full episode execution
- GET /health: Health check
- GET /info: Model information
"""

from typing import Any

from fastapi import FastAPI, HTTPException

from vla_server_base.adapters.base import VLAModelAdapter
from vla_server_base.schemas import ExecuteRequest, ExecuteResponse, State
from vla_server_base.services.episode_executor import EpisodeExecutor


def create_vla_app(
    adapter: VLAModelAdapter,
    service_name: str,
    service_version: str = "0.1.0",
    max_steps: int = 75,
) -> FastAPI:
    """
    Create FastAPI application for VLA service

    Args:
        adapter: VLA model adapter (must be loaded)
        service_name: Service name (e.g., "Mock VLA Service")
        service_version: Service version (default: "0.1.0")
        max_steps: Maximum steps per episode (default: 75)

    Returns:
        Configured FastAPI application
    """
    app = FastAPI(
        title=service_name,
        version=service_version,
        description=f"VLA model inference service: {service_name}",
    )

    # Create episode executor
    executor = EpisodeExecutor(adapter=adapter, max_steps=max_steps)

    @app.post("/execute", response_model=ExecuteResponse)
    async def execute(request: ExecuteRequest) -> ExecuteResponse:
        """
        Execute full episode with VLA model

        Args:
            request: Execution request (instruction, xml_string)

        Returns:
            Episode data (actions, states, duration_ms)

        Raises:
            HTTPException: 400 for invalid input, 500 for execution errors
        """
        try:
            # Execute episode
            result = executor.execute_episode(
                instruction=request.instruction,
                xml_string=request.xml_string,
            )

            # Convert states to State objects
            states = [State(**state) for state in result["states"]]

            return ExecuteResponse(
                actions=result["actions"],
                states=states,
                duration_ms=result["duration_ms"],
            )

        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e)) from e
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e)) from e

    @app.get("/health")
    async def health() -> dict[str, str]:
        """Health check endpoint"""
        model_status = "loaded" if adapter.model_loaded else "not_loaded"
        return {
            "status": "healthy" if adapter.model_loaded else "unhealthy",
            "model": adapter.model_id or "unknown",
            "model_status": model_status,
        }

    @app.get("/info")
    async def info() -> dict[str, Any]:
        """Model information endpoint"""
        return {
            "service_name": service_name,
            "service_version": service_version,
            "model_id": adapter.model_id or "not_loaded",
            "device": adapter.device or "unknown",
            "loaded": adapter.model_loaded,
            "max_steps": max_steps,
        }

    return app
