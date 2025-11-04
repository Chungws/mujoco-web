# Worker Dockerfile for llmbattler

FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    postgresql-client \
    && rm -rf /var/lib/apt/lists/*

# Copy uv from official image (recommended by uv docs)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Copy workspace root files
COPY pyproject.toml uv.lock ./

# Copy shared package
COPY shared/ ./shared/

# Copy worker package
COPY worker/ ./worker/

# Copy config files
COPY config/ ./config/

# Sync workspace dependencies (includes shared package)
RUN --mount=type=cache,target=/root/.cache/uv \
    UV_COMPILE_BYTECODE=1 \
    uv sync --frozen --package llmbattler-worker

# Copy entrypoint script
COPY worker/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]

# Run worker via uv run
CMD ["uv", "run", "--package", "llmbattler-worker", "python", "-m", "llmbattler_worker.main"]
