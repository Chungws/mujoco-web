#!/bin/bash
# Worker entrypoint script
# Waits for PostgreSQL and backend migrations before starting worker

set -e

echo "=========================================="
echo "Worker Initialization"
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

# Additional wait for backend migrations to complete
echo "[INFO] Waiting for backend migrations to complete..."
sleep 5

echo "=========================================="
echo "[INFO] Starting worker..."
echo "=========================================="

# Start the worker
exec "$@"
