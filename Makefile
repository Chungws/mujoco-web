.PHONY: help setup dev dev-infra dev-backend dev-frontend dev-worker stop clean test lint prod-setup prod prod-build prod-logs prod-stop prod-clean

help:
	@echo "vlaarena Development Commands"
	@echo ""
	@echo "Development:"
	@echo "  make setup        - Initial setup (install dependencies)"
	@echo "  make dev          - Start infrastructure and show service commands"
	@echo "  make dev-infra    - Start PostgreSQL + MinIO"
	@echo "  make dev-backend  - Start Backend API (port 8000)"
	@echo "  make dev-frontend - Start Frontend (port 3000)"
	@echo "  make dev-worker   - Start Worker (manual run)"
	@echo "  make stop         - Stop Docker services"
	@echo "  make clean        - Stop and remove all data (WARNING: deletes DB)"
	@echo ""
	@echo "Production:"
	@echo "  make prod-setup   - Create .env.prod file (first time only)"
	@echo "  make prod         - Start all services in production mode"
	@echo "  make prod-build   - Rebuild and start production services"
	@echo "  make prod-logs    - View production logs (Ctrl+C to exit)"
	@echo "  make prod-stop    - Stop production services"
	@echo "  make prod-clean   - Stop and remove all production data"
	@echo ""
	@echo "Testing:"
	@echo "  make test         - Run all tests"
	@echo "  make lint         - Run all linters"

# Initial setup
setup:
	@echo "📦 Installing dependencies..."
	@echo ""
	@echo "🐍 Syncing Python workspace (backend + worker + shared)..."
	@uv sync --all-extras
	@echo ""
	@echo "📦 Installing frontend dependencies..."
	@cd frontend && npm install
	@echo ""
	@echo "✅ Setup complete!"
	@echo ""
	@echo "📝 Next steps:"
	@echo "  1. Copy environment file: cp .env.example .env"
	@echo "  2. Start services: make dev"

# Start infrastructure (PostgreSQL + MinIO)
dev-infra:
	@echo "🚀 Starting infrastructure (PostgreSQL + MinIO)..."
	docker compose --profile dev up -d
	@echo "✅ PostgreSQL started on localhost:5432"
	@echo "✅ MinIO started on localhost:9000 (API) and localhost:9001 (Console)"

# Start Backend (requires PostgreSQL)
dev-backend:
	@echo "🚀 Starting Backend API on http://localhost:8000..."
	@echo "🔄 Running database migrations..."
	@cd backend && uv run alembic upgrade head
	@cd backend && uv run uvicorn vlaarena_backend.main:app --reload --port 8000

# Start Frontend
dev-frontend:
	@if [ ! -d "frontend/node_modules" ]; then \
		echo "⚠️  node_modules not found. Running npm install..."; \
		cd frontend && npm install; \
	fi
	@echo "🎨 Starting Frontend on http://localhost:3000..."
	@cd frontend && npm run dev

# Start Worker (manual run)
dev-worker:
	@echo "⚙️  Running Worker (vote aggregation)..."
	@cd worker && uv run python -m vlaarena_worker.main

# Start all services (convenience command)
dev:
	@make dev-infra
	@sleep 3
	@echo ""
	@echo "✅ Infrastructure started!"
	@echo ""
	@echo "📝 Now run in separate terminals:"
	@echo "  Terminal 1: make dev-backend"
	@echo "  Terminal 2: make dev-frontend"
	@echo "  Terminal 3: make dev-worker (optional)"
	@echo ""

# Stop Docker services
stop:
	@echo "🛑 Stopping Docker services..."
	@docker compose --profile dev down
	@echo "✅ Services stopped"

# Clean all data (WARNING)
clean:
	@echo "⚠️  WARNING: This will delete all database data!"
	@read -p "Are you sure? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		docker compose --profile dev down -v; \
		echo "✅ All data deleted"; \
	else \
		echo "❌ Cancelled"; \
	fi

# Run tests
test:
	@echo "🧪 Running backend tests..."
	@cd backend && uv run pytest -s
	@echo ""
	@echo "🧪 Running worker tests..."
	@cd worker && uv run pytest -s
	@echo ""
	@echo "✅ All tests completed"

# Run linters
lint:
	@echo "🔍 Running backend linters..."
	@uvx ruff check && uvx ruff format --check
	@echo ""
	@echo "🔍 Running frontend linter..."
	@cd frontend && npm run lint
	@echo ""
	@echo "✅ All linters passed"

# ==========================================
# Production Commands
# ==========================================

# Create production environment file
prod-setup:
	@if [ -f .env.prod ]; then \
		echo "⚠️  .env.prod already exists"; \
		echo "Edit it manually or delete it first"; \
	else \
		echo "📝 Creating .env.prod..."; \
		cp .env.example .env.prod; \
		echo ""; \
		echo "✅ .env.prod created!"; \
		echo ""; \
		echo "📝 Important: Edit .env.prod and set:"; \
		echo "  - POSTGRES_URI=postgresql+asyncpg://postgres:postgres@postgres:5432/vlaarena"; \
		echo "  - NEXT_PUBLIC_API_URL (your backend URL)"; \
		echo "  - CORS_ORIGINS (your frontend URL)"; \
	fi

# Start production services
prod:
	@echo "🚀 Starting production services..."
	@if [ ! -f .env.prod ]; then \
		echo "❌ .env.prod not found!"; \
		echo "Run: make prod-setup"; \
		exit 1; \
	fi
	docker compose --env-file .env.prod --profile prod up -d
	@echo ""
	@echo "✅ Production services starting..."
	@echo ""
	@echo "🔗 Services:"
	@echo "  Frontend:  http://localhost:3000"
	@echo "  Backend:   http://localhost:8000"
	@echo "  MinIO API: http://localhost:9000"
	@echo "  MinIO Console: http://localhost:9001"
	@echo "  Postgres:  localhost:5432"
	@echo ""
	@echo "📊 Check status: docker compose ps"
	@echo "📋 View logs:    make prod-logs"

# Rebuild and start production services
prod-build:
	@echo "🔨 Building and starting production services..."
	@if [ ! -f .env.prod ]; then \
		echo "❌ .env.prod not found!"; \
		echo "Run: make prod-setup"; \
		exit 1; \
	fi
	docker compose --env-file .env.prod --profile prod up -d --build
	@echo ""
	@echo "✅ Production services rebuilt and started"

# View production logs
prod-logs:
	@echo "📋 Viewing production logs (Ctrl+C to exit)..."
	docker compose --env-file .env.prod logs -f

# Stop production services
prod-stop:
	@echo "🛑 Stopping production services..."
	docker compose --env-file .env.prod --profile prod down
	@echo "✅ Production services stopped"

# Clean production data (WARNING)
prod-clean:
	@echo "⚠️  WARNING: This will delete all production data!"
	@read -p "Are you sure? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		docker compose --env-file .env.prod --profile prod down -v; \
		echo "✅ All production data deleted"; \
	else \
		echo "❌ Cancelled"; \
	fi
