# llmbattler-backend

FastAPI backend for LLM Battle Arena.

## Quick Start

### Prerequisites
- Python 3.11+
- uv package manager
- PostgreSQL 16+
- MongoDB

### Development Setup

```bash
# From project root
uv sync

# Run migrations (TODO: will be added later)
# cd backend
# uv run alembic upgrade head

# Start development server
cd backend
uv run uvicorn llmbattler_backend.main:app --reload --port 8000
```

**Alternative (from workspace root):**
```bash
uv run --package llmbattler-backend uvicorn llmbattler_backend.main:app --reload
```

### Environment Variables

Copy `.env.example` to `.env` and configure:

```bash
cp .env.example .env
```

Key settings:
- `POSTGRES_URI`: PostgreSQL connection string
- `MONGODB_URI`: MongoDB connection string
- `CORS_ORIGINS`: Allowed frontend origins (comma-separated)

### Running Tests

```bash
cd backend
uv run pytest -s
```

### Code Quality

```bash
# Linting
uvx ruff check

# Formatting
uvx ruff format --check

# Import sorting
uvx isort --check --profile black .
```

## Project Structure

```
backend/
├── src/
│   └── llmbattler_backend/
│       ├── __init__.py
│       ├── main.py           # FastAPI app entry
│       ├── api/              # API routes
│       ├── services/         # Business logic
│       └── mongodb/          # MongoDB operations
├── tests/                    # pytest tests
├── Dockerfile
├── pyproject.toml
├── .env.example
└── README.md
```

## API Documentation

Once the server is running, visit:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## TODO

- [ ] Add database initialization logic
- [ ] Implement battle creation API
- [ ] Implement voting API
- [ ] Implement model management
- [ ] Implement leaderboard API
- [ ] Add Alembic migrations (after data model review)
- [ ] Add comprehensive tests

## Related Documentation

- [WORKSPACE/CONVENTIONS/backend/](../WORKSPACE/CONVENTIONS/backend/) - Backend conventions
- [WORKSPACE/FEATURES/001_BATTLE_MVP.md](../WORKSPACE/FEATURES/001_BATTLE_MVP.md) - Battle mode spec
- [WORKSPACE/FEATURES/002_LEADERBOARD_MVP.md](../WORKSPACE/FEATURES/002_LEADERBOARD_MVP.md) - Leaderboard spec
