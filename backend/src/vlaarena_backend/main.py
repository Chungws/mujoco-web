"""
FastAPI application entry point
"""

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from vlaarena_backend.api import battles, leaderboard, models, sessions
from vlaarena_backend.services.llm_client import (
    MockLLMClient,
    OpenAILLMClient,
    set_llm_client,
)
from vlaarena_shared.config import settings
from vlaarena_shared.logging_config import setup_logging


# Configure logging
logger = setup_logging("vlaarena_backend")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    logger.info("Starting vlaarena-backend...")

    # Initialize LLM client based on configuration
    if settings.use_mock_llm:
        logger.info("🎭 Using Mock LLM client (development/testing mode)")
        set_llm_client(MockLLMClient())
    else:
        logger.info("🚀 Using OpenAI-compatible LLM client (production mode)")
        set_llm_client(OpenAILLMClient())

    # TODO: Initialize database connections
    # - MongoDB (Motor)
    # - PostgreSQL (SQLAlchemy async)
    # - Create MongoDB indexes

    logger.info("Backend startup complete")

    yield

    logger.info("Shutting down vlaarena-backend...")

    # TODO: Close database connections

    logger.info("Backend shutdown complete")


# Create FastAPI app
app = FastAPI(
    title="vlaarena API",
    description="AI Language Model Battle Arena - Backend API",
    version="0.1.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "vlaarena-backend"}


# Include API routers
app.include_router(models.router, prefix="/api", tags=["models"])
app.include_router(sessions.router, prefix="/api", tags=["sessions"])
app.include_router(battles.router, prefix="/api", tags=["battles"])
app.include_router(leaderboard.router, prefix="/api", tags=["leaderboard"])
