#!/bin/bash
# Backend entrypoint script
# Automatically runs database migrations before starting the server

set -e

# Ensure PATH includes virtual environment
export PATH="/app/.venv/bin:$PATH"

echo "=========================================="
echo "Backend Initialization"
echo "=========================================="

# Wait for PostgreSQL to be ready
echo "[INFO] Waiting for PostgreSQL to be ready..."
max_attempts=30
attempt=0

# Extract connection details from POSTGRES_URI
# Format: postgresql+asyncpg://user:password@host:port/database
DB_HOST=$(echo "$POSTGRES_URI" | sed -n 's/.*@\([^:]*\):.*/\1/p')
DB_PORT=$(echo "$POSTGRES_URI" | sed -n 's/.*:\([0-9]*\)\/.*/\1/p')
DB_USER=$(echo "$POSTGRES_URI" | sed -n 's/.*\/\/\([^:]*\):.*/\1/p')

echo "[INFO] Database host: $DB_HOST:$DB_PORT"

while [ $attempt -lt $max_attempts ]; do
    if pg_isready -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" > /dev/null 2>&1; then
        echo "[SUCCESS] PostgreSQL is ready!"
        break
    fi
    attempt=$((attempt + 1))
    echo "[INFO] Attempt $attempt/$max_attempts - PostgreSQL not ready yet, waiting 2 seconds..."
    sleep 2
done

if [ $attempt -eq $max_attempts ]; then
    echo "[ERROR] PostgreSQL failed to become ready after $max_attempts attempts"
    exit 1
fi

# Run database migrations
echo "=========================================="
echo "[INFO] Running database migrations..."

# Run alembic using uv run (which handles the virtual environment)
cd /app/backend

if uv run --package llmbattler-backend alembic upgrade head; then
    echo "[SUCCESS] Database migrations completed"
else
    echo "[ERROR] Database migrations failed"
    exit 1
fi

cd /app

echo "=========================================="
echo "[INFO] Starting FastAPI server..."
echo "=========================================="

# Start the application
exec "$@"
