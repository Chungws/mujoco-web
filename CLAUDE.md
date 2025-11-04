# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## 📖 Project Overview

**mujoco-web** - VLA Model Testing & Evaluation Platform

A web-based platform for testing and evaluating Vision-Language-Action (VLA) models in simulated environments using MuJoCo physics simulator.

**Key Features:**
- Phase 1: VLA policy visualization in MuJoCo simulator
- Phase 2: Blind A/B testing and ELO-based leaderboard for VLA models

---

## 🔴 CRITICAL RULES

### 1. Branch Safety
**BEFORE doing ANY work, ALWAYS check current branch:**
```bash
git branch --show-current
```

**Rules:**
- ❌ NEVER work on `main` branch directly (once feature branches are set up)
- ✅ ALWAYS switch to feature branch FIRST
- ✅ Create feature branch: `git checkout -b feature/your-feature-name`

**Current Status:**
- Phase 0-1: May work on `main` branch (early development)
- Phase 2+: Switch to feature branch workflow

### 2. Project Policies ⚠️

| Policy | This Project | Reference |
|--------|--------------|-----------|
| Foreign Keys | ❌ **NOT used** (Phase 2) | [lmarena-clone ADR-001](../lmarena-clone/WORKSPACE/ARCHITECTURE/ADR_001-No_Foreign_Keys.md) |
| Main Branch | ✅ `main` | Standard Git Flow |
| PR Language | ✅ **English** | - |
| Git Host | ✅ **GitHub** | - |
| Package Manager (Frontend) | ✅ **npm** | - |
| Package Manager (Backend) | ✅ **uv** (Phase 2) | - |

**🚨 ALWAYS check WORKSPACE/00_PROJECT.md for complete policies**

### 3. Pre-Commit Checklist

**Phase 1 (Frontend Only):**
```bash
cd frontend
npm run lint    # ESLint check
npm run build   # Verify build works
# UI changes: Test manually in browser
```

**Phase 2 (Backend + Frontend):**
```bash
# Frontend
cd frontend
npm run lint

# Backend
cd backend
uvx ruff check
uvx ruff format --check
uv run pytest -s

# Worker
cd worker
uvx ruff check
uvx ruff format --check
uv run pytest -s
```

**All checks must pass before creating PR.**

---

## 📚 Documentation

**⭐ All detailed rules and conventions are in WORKSPACE:**

| Category | Location | Description |
|----------|----------|-------------|
| **Project Info** | [WORKSPACE/00_PROJECT.md](./WORKSPACE/00_PROJECT.md) | Project overview, policies, Quick Start |
| **Roadmap** | [WORKSPACE/ROADMAP.md](./WORKSPACE/ROADMAP.md) | Development roadmap and milestones |
| **Features** | [WORKSPACE/FEATURES/](./WORKSPACE/FEATURES/) | Feature specifications and phase tracking |
| **Reference** | [../lmarena-clone](../lmarena-clone) | Reference project for Phase 2 architecture |

---

## 🚀 Quick Start

### Phase 1 Setup (Current)

```bash
# Install frontend dependencies
cd frontend
npm install

# Start development server
npm run dev
```

**Endpoints:**
- Frontend: http://localhost:3000
- MuJoCo Viewer: Integrated in home page

### Phase 2 Setup (Planned)

Will include backend API, worker, and database similar to lmarena-clone:

```bash
# Install dependencies
make setup

# Start infrastructure (PostgreSQL)
make dev-infra

# Run services in separate terminals
Terminal 1: make dev-backend   # Backend API (port 8000)
Terminal 2: make dev-frontend  # Frontend (port 3000)
Terminal 3: make dev-worker    # Worker (optional)
```

🔗 **Full setup guide:** [WORKSPACE/00_PROJECT.md#quick-start](./WORKSPACE/00_PROJECT.md)

---

## 🤖 Workflow Commands

### Phase 1 (Current)

**Frontend Development:**
```bash
cd frontend
npm run dev        # Start dev server (port 3000)
npm run build      # Production build
npm run lint       # Run ESLint
```

### Phase 2 (Planned)

**Makefile Commands:**
```bash
make help         # Show all available commands
make setup        # Install dependencies
make dev-infra    # Start PostgreSQL
make dev-backend  # Start Backend API
make dev-frontend # Start Frontend
make dev-worker   # Run Worker
make stop         # Stop Docker services
make test         # Run all tests
make lint         # Run all linters
```

**Common Workflow:**
1. Check branch: `git branch --show-current`
2. Create feature branch: `git checkout -b feature/your-feature-name`
3. Start development server
4. Develop and test locally
5. Run pre-commit checks
6. Commit and push
7. Create GitHub Pull Request

---

## 🏗️ Architecture Summary

### Phase 1: VLA Visualization
```
┌─────────────────┐
│ Next.js Client  │ (MuJoCo Viewer, VLA Controls)
└────────┬────────┘
         │
    ┌────▼─────────┐
    │ MuJoCo WASM  │ (Physics Simulation)
    └──────────────┘
```

**Tech Stack:**
- Frontend: Next.js 15 with App Router
- 3D Rendering: Three.js + MuJoCo WASM
- ML Inference: ONNX Runtime Web (client-side)
- Styling: Tailwind CSS

### Phase 2: Model Comparison (Planned)
```
┌─────────────────┐
│ Next.js Client  │ (Battle UI, Leaderboard, MuJoCo Viewer)
└────────┬────────┘
         │
    ┌────▼─────────┐
    │ FastAPI API  │ (Sessions, Battles, Voting)
    └────┬─────────┘
         │
         ├──► VLA Model A
         ├──► VLA Model B
         └──► PostgreSQL
                  │
             ┌────▼────────┐
             │   Worker    │ (ELO aggregation)
             └─────────────┘
```

**Tech Stack (Additional):**
- Backend: FastAPI with SQLModel
- Database: PostgreSQL with JSONB
- Worker: Python with APScheduler
- Package Manager: uv (Python workspace)

---

## 📂 Key Directories

### Phase 1 (Current)
```
frontend/
├── app/              # Next.js pages
├── components/       # React components
│   └── mujoco-viewer.tsx  # Main viewer
├── lib/              # Utilities
│   └── mujoco/       # MuJoCo integration
└── public/
    └── mujoco/       # Scene files

mujoco_wasm/          # MuJoCo WebAssembly build
└── dist/             # Compiled WASM files

WORKSPACE/            # Documentation
├── 00_PROJECT.md     # Project overview
├── ROADMAP.md        # Development roadmap
└── FEATURES/         # Feature specs
```

### Phase 2 (Planned)
```
backend/              # FastAPI application
worker/               # Vote aggregation worker
shared/               # Shared Python code
docker-compose.yml    # Docker services
```

---

## 🎯 Development Phases

| Phase | Status | Description |
|-------|--------|-------------|
| Phase 0 | ✅ Done | Project setup and documentation |
| Phase 1 | 🔄 Current | VLA visualization (MuJoCo + ONNX) |
| Phase 2 | ⏸️ Planned | Model comparison and leaderboard |

**Current Focus:** Phase 1 - VLA Visualization
**See:** [WORKSPACE/ROADMAP.md](./WORKSPACE/ROADMAP.md)

---

## 🔑 Key Concepts

### VLA (Vision-Language-Action) Models
- Input: Visual observations + Natural language instructions
- Output: Actions for robot control
- Example: "Pick up the red cube" → Robot arm movements

### MuJoCo
- Physics simulator for robotics
- Runs in browser via WebAssembly
- Real-time simulation and rendering

### Phase 2: Battle System (Planned)
- Blind A/B testing of VLA models
- Users vote on better performance
- ELO rankings like chess ratings
- Based on lmarena-clone architecture

---

## 📝 Conventions

### Frontend (Phase 1)
- **Components:** Use React Server Components where possible
- **Styling:** Tailwind CSS with shadcn/ui components
- **State Management:** React Context for global state
- **File Naming:** kebab-case for files, PascalCase for components

### Backend (Phase 2 - Planned)
- **Architecture:** 4-layer structure (models → schemas → service → router)
- **TDD:** Test-Driven Development with pytest
- **No Foreign Keys:** Application-level relationships only
- **Package Manager:** Use `uv` (NOT pip)

### Git Workflow
- **Branches:** `feature/*` for new features
- **Commits:** Conventional Commits format
  ```
  feat: add VLA policy loader
  fix: resolve camera control bug
  docs: update README
  ```
- **PRs:** English language, descriptive titles

---

## 🚨 Important Notes

### Phase 1 Development
1. **Performance:** Target 30+ FPS for MuJoCo rendering
2. **Browser Compatibility:** Test on Chrome, Firefox, Safari
3. **Model Size:** Keep VLA models < 100 MB for browser loading
4. **WASM Loading:** Optimize loading time < 5 seconds

### Phase 2 Preparation
1. **Reference:** Study lmarena-clone architecture thoroughly
2. **No Foreign Keys:** Application-level relationships only
3. **PostgreSQL:** JSONB for VLA execution logs
4. **ELO System:** Bradley-Terry model with confidence intervals

### Testing
- **Phase 1:** Manual browser testing + ESLint
- **Phase 2:** pytest (backend) + Playwright (frontend)

---

## 🔗 Reference Projects

- **lmarena-clone** (`../lmarena-clone`): Battle system and leaderboard reference
- **muwanx** (`./muwanx`): MuJoCo viewer reference (Vue.js)
- **mujoco_wasm** (`./mujoco_wasm`): MuJoCo WebAssembly build

---

## 📞 Support

- **Documentation:** Check WORKSPACE/ folder first
- **Issues:** GitHub Issues (to be set up)
- **Reference:** Consult lmarena-clone for Phase 2 patterns

---

**Created:** 2025-11-01
**Last Updated:** 2025-11-01
**Current Phase:** Phase 1 - VLA Visualization
