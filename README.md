# VLA Arena

**Vision-Language-Action Model Comparison Platform**

A web-based arena for blind A/B testing of VLA models in MuJoCo-simulated robot manipulation tasks.

---

## 🚀 Quick Start

**Prerequisites:**
- Node.js 18+ (Frontend)
- Python 3.11+ with uv (Backend)
- Docker & Docker Compose (Databases)

**Start Development:**

```bash
# 1. Start databases
docker compose up -d

# 2. Install dependencies
uv sync

# 3. Start backend (Terminal 1)
cd backend
uv run uvicorn vlaarena_backend.main:app --reload --port 8000

# 4. Start frontend (Terminal 2)
cd frontend
npm install
npm run dev
```

**Endpoints:**
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs

---

## 📚 Documentation

**Complete documentation:** [`WORKSPACE/00_PROJECT.md`](./WORKSPACE/00_PROJECT.md)

**Quick Links:**
- [Project Overview](./WORKSPACE/00_PROJECT.md#project-overview)
- [Development Roadmap](./WORKSPACE/ROADMAP.md)
- [Current Feature: MVP](./WORKSPACE/FEATURES/001_MVP.md)
- [Architecture Decisions](./WORKSPACE/ARCHITECTURE/)

---

## 🏗️ Project Structure

```
mujoco-web/
├── frontend/           # Next.js 15 application
├── backend/            # FastAPI application
├── worker/             # Vote aggregation worker
├── shared/             # Shared Python code (models, schemas)
├── WORKSPACE/          # Documentation
├── .claude/            # AI assistant skills & commands
└── docker-compose.yml  # PostgreSQL + MongoDB
```

---

## 🛠️ Tech Stack

- **Frontend:** Next.js 15, React, Three.js, MuJoCo WASM, shadcn/ui
- **Backend:** FastAPI, SQLModel, Beanie (MongoDB ODM)
- **Databases:** PostgreSQL, MongoDB
- **Worker:** Python, APScheduler
- **Tooling:** uv (Python), npm (Node.js), Docker Compose

---

## 📖 For AI Assistants

See [`CLAUDE.md`](./CLAUDE.md) for development guidelines, skills, and slash commands.

---

## 📄 License

MIT

---

**Created:** 2025-11-01
**Status:** MVP Development - Backend Foundation (Week 2-3)
