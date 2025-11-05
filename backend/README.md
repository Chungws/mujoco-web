# VLA Arena Backend

FastAPI backend for VLA model comparison platform.

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
uv run uvicorn vlaarena_backend.main:app --reload --port 8000
```

**Alternative (from workspace root):**
```bash
uv run --package vlaarena-backend uvicorn vlaarena_backend.main:app --reload
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
# Linting (includes import sorting)
uvx ruff check

# Formatting
uvx ruff format --check
```

## Project Structure

```
backend/
├── src/
│   └── vlaarena_backend/
│       ├── __init__.py
│       ├── main.py           # FastAPI app entry
│       ├── api/              # API routers
│       ├── services/         # Business logic
│       └── repositories/     # Data access layer
├── alembic/                  # Database migrations
├── tests/                    # pytest tests
├── pyproject.toml
├── .env.example
└── README.md
```

## API Documentation

Once the server is running, visit:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Architecture

**4-Layer Pattern:**
1. **Models** (shared/src/vlaarena_shared/models.py) - SQLModel database models
2. **Schemas** (shared/src/vlaarena_shared/schemas.py) - Pydantic request/response schemas
3. **Services** (services/) - Business logic layer
4. **Routers** (api/) - API endpoints

**See:** `.claude/skills/fastapi-patterns/SKILL.md` for complete architecture guide.

## Related Documentation

- [FastAPI Patterns](./.claude/skills/fastapi-patterns/SKILL.md) - Architecture patterns
- [Backend TDD Workflow](./.claude/skills/backend-tdd-workflow/SKILL.md) - Test-driven development
- [SQLModel No Foreign Keys](./.claude/skills/sqlmodel-no-foreign-keys/SKILL.md) - Database modeling
- [Managing Python Deps](./.claude/skills/managing-python-deps/SKILL.md) - uv dependency management
- [MVP Feature Spec](../WORKSPACE/FEATURES/001_MVP.md) - Current feature specification

---

**Last Updated:** 2025-11-05
