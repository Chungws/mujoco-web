"""
VLA Server FastAPI Application

Main entry point for VLA execution server.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import settings

app = FastAPI(
    title="VLA Execution Server",
    description="Vision-Language-Action model execution service",
    version="0.1.0",
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "service": "VLA Execution Server",
        "version": "0.1.0",
        "status": "running",
    }


@app.get("/health")
async def health():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "device": settings.device,
        "max_steps": settings.max_episode_steps,
    }


# Import routers when implemented
# from .api import execute
# app.include_router(execute.router)
