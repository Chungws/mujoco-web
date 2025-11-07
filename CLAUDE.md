# CLAUDE.md

**AI Assistant guidance for the mujoco-web repository**

---

## 📖 Project Overview

**VLA Arena** - Vision-Language-Action Model Comparison Platform

A web-based arena for blind A/B testing of VLA models in MuJoCo-simulated robot manipulation tasks. Users provide natural language instructions, watch two anonymized models execute side-by-side, vote on performance, and contribute to an ELO-based leaderboard.

**Tech Stack:**
- **Frontend:** Next.js 15 (App Router), Three.js, MuJoCo WASM, shadcn/ui
- **Backend:** FastAPI, SQLModel, PostgreSQL, MongoDB
- **Worker:** Python, APScheduler (ELO aggregation)
- **Tooling:** uv (Python), npm (Frontend), Docker Compose

**Reference Project:** `../lmarena-clone` (architecture patterns)

---

## 🔴 CRITICAL RULES

### 1. Branch Safety

**ALWAYS check current branch before ANY work:**
```bash
git branch --show-current
```

- ✅ Work on `feature/*` branches for new features
- ✅ Target `develop` branch for PRs
- ❌ NEVER work directly on `main` or `develop`

**See:** `git-branching` skill for complete branching strategy

---

### 2. Core Policies

| Policy | Value | Skill Reference |
|--------|-------|-----------------|
| **Main Branch** | `develop` | `git-branching` |
| **PR Target** | `develop` | `creating-pull-requests` |
| **PR Language** | English | `creating-pull-requests` |
| **PR Size** | < 300 lines | `creating-pull-requests` |
| **Foreign Keys** | ❌ NOT used | `sqlmodel-no-foreign-keys` |
| **Python Deps** | `uv` (NOT pip) | `managing-python-deps` |
| **Linting** | `ruff` (NOT isort) | Root `pyproject.toml` |

**Complete policies:** `WORKSPACE/00_PROJECT.md`

---

### 3. Pre-Commit Checklist

**Backend:**
```bash
uvx ruff check        # Linting (includes import sorting)
uvx ruff format --check
uv run pytest -s
```

**Frontend:**
```bash
npm run lint
# If UI changed: Verify with Chrome DevTools MCP
```

**All checks must pass before creating PR.**

**See:** `reviewing-code` skill for complete checklist

---

### 4. Workspace & Dependencies Management

**Project Structure:** uv workspace with centralized dev dependencies

```
mujoco-web/
├── pyproject.toml          # Root: Dev dependencies & ruff config
├── backend/pyproject.toml  # Only production dependencies
├── worker/pyproject.toml   # Only production dependencies
└── shared/pyproject.toml   # Only production dependencies
```

**CRITICAL RULES:**

1. **Dev Dependencies ONLY in root:**
   - ✅ `pytest`, `pytest-asyncio`, `pytest-cov` → Root
   - ✅ `ruff` → Root
   - ✅ `aiosqlite`, `greenlet`, `mongomock-motor` → Root
   - ❌ NEVER add dev dependencies to sub-packages

2. **Ruff Configuration ONLY in root:**
   - ✅ `[tool.ruff]` → Root pyproject.toml only
   - ❌ NO ruff config in backend/worker/shared

3. **NO isort:**
   - ❌ isort is redundant (ruff handles import sorting with `I` rule)
   - ✅ Use `uvx ruff check --fix` for import sorting

4. **Testing from root:**
   ```bash
   # ALWAYS run tests from project root
   make test                                    # All tests
   uv run --directory backend pytest -s         # Backend only
   uv run --directory worker pytest -s          # Worker only
   ```

5. **Adding Dependencies:**
   ```bash
   # Production dependency → Sub-package
   cd backend && uv add fastapi

   # Dev dependency → Root
   cd /path/to/root && uv add --dev pytest-mock
   ```

**Benefits:**
- ✅ Single source of truth for dev tools
- ✅ No version conflicts
- ✅ Simpler maintenance
- ✅ Workspace-level tooling available everywhere

**See:** `managing-python-deps` skill for complete dependency management

---

## 🎯 Skills & Slash Commands

### Available Skills

Use these skills for specialized workflows:

| Skill | Purpose |
|-------|---------|
| `alembic-migrations` | Database migration management (ALWAYS use --autogenerate) |
| `backend-tdd-workflow` | TDD workflow (Red-Green-Refactor, pytest, AAA pattern) |
| `committing-changes` | Git commit format (`<type>: <subject>`, Co-authored-by) |
| `creating-pull-requests` | GitHub PR creation (English, develop target, <300 lines) |
| `fastapi-patterns` | Backend architecture (4-layer: models→schemas→service→router) |
| `fixing-linting-errors` | Ruff linting workflow (ALWAYS run from root, auto-fix → manual fix) |
| `frontend-ui-testing` | Chrome DevTools MCP verification (MANDATORY for UI changes) |
| `git-branching` | Git Flow branching (feature/*, develop, main) |
| `managing-python-deps` | uv dependency management (NEVER use pip) |
| `nextjs-rsc-patterns` | Next.js RSC patterns (page.tsx async, *-client.tsx) |
| `reviewing-code` | Self code review checklist (before MR) |
| `sqlmodel-no-foreign-keys` | Database modeling with NO FKs (ADR-001) |
| `using-shadcn-components` | shadcn/ui components (NEVER edit components/ui/) |

**Invoke skills:** Use the Skill tool with skill name (e.g., `Skill(command="fastapi-patterns")`)

---

### Available Slash Commands

Workflow automation commands:

| Command | Purpose |
|---------|---------|
| `/start-phase` | Start new development phase (create branch, read conventions) |
| `/verify-phase` | Run all quality checks (lint, format, tests) |
| `/review-phase` | Self code review + update docs before MR |
| `/end-phase` | Complete phase and prepare for next |
| `/create-pr` | Create GitHub PR with English format |
| `/clarify` | Clarify requirements and design architecture |
| `/new-feature` | Create new feature spec in WORKSPACE/FEATURES/ |
| `/sync-docs` | Check WORKSPACE docs sync with code |
| `/check-outdated` | Check if WORKSPACE docs are outdated |
| `/help-kr` | Korean help guide for all commands |

**Invoke commands:** Use the SlashCommand tool (e.g., `SlashCommand(command="/start-phase")`)

---

## 📚 Documentation Structure

**All detailed documentation lives in WORKSPACE:**

```
WORKSPACE/
├── 00_PROJECT.md              # Project overview, policies, Quick Start
├── ROADMAP.md                 # Development roadmap and milestones
├── FEATURES/
│   ├── 001_MVP.md             # VLA Arena MVP specification (current)
│   └── README.md              # Feature tracking guide
└── ARCHITECTURE/
    ├── ADR_001-No_Foreign_Keys.md
    └── ADR_002-Database_Schema_Design.md

.claude/
├── skills/                     # Specialized workflows (13 skills)
└── commands/                   # Slash commands (10 commands)
```

**Always check WORKSPACE first for project-specific rules.**

---

## 🚀 Quick Start

### Development Setup

**Infrastructure:**
```bash
# Start PostgreSQL + MongoDB
docker compose up -d
```

**Backend:**
```bash
# Install dependencies (from root)
uv sync --all-extras

# Run backend
cd backend
uv run uvicorn vlaarena_backend.main:app --reload --port 8000
```

**Frontend:**
```bash
cd frontend
npm install
npm run dev  # Port 3000
```

**Worker (optional):**
```bash
cd worker
uv run python -m vlaarena_worker.main
```

**Complete setup:** `WORKSPACE/00_PROJECT.md#quick-start`

---

## 📊 Current Status

**Phase:** MVP Development - Week 3-5 (VLA Server Phase 2)

**Completed:**
- ✅ Week 1: Project setup, ADR-001, ADR-002, ADR-003, Docker Compose
- ✅ Week 2-3: Backend Foundation (PostgreSQL, MongoDB, APIs, 88 backend tests)
- ✅ Frontend & Worker (Battle page, Leaderboard, ELO aggregation, 40 worker tests)
- ✅ VLA Server Phase 1: Infrastructure (Config + MuJoCo Environment, 26 tests)

**Current Work:**
- 🔄 VLA Server Phase 2: VLA Integration (Adapters, Execution Service, API)

**Next:**
- ⏭️ Complete VLA Server Phase 2
- ⏭️ Backend integration with VLA Server

**Detailed roadmap:** `WORKSPACE/FEATURES/001_MVP.md`

---

## 🏗️ Architecture Overview

**High-Level Flow:**
```
User → Battle Page (Next.js)
  ↓
POST /api/sessions/init → Create session + assign models
  ↓
POST /api/battles/{id}/turns → Server-side VLA execution
  ↓
Episodes saved to MongoDB (actions, states, metrics)
  ↓
GET /api/episodes/{id} → Frontend loads states
  ↓
MuJoCo WASM renders side-by-side replay
  ↓
POST /api/votes → Submit vote + reveal models
  ↓
Worker (hourly) → Aggregate votes → Update ELO rankings
```

**Key Design Decisions:**
- **Server-side execution:** MuJoCo + VLA inference on backend (not browser)
- **State-based replay:** Store qpos/qvel, not videos
- **No Foreign Keys:** Application-level relationships (ADR-001)
- **Robot-specific ELO:** Separate rankings per robot + global rankings

**Architecture details:** `WORKSPACE/00_PROJECT.md` and `WORKSPACE/ARCHITECTURE/`

---

## 📂 Project Structure

```
mujoco-web/
├── frontend/                  # Next.js 15 application
│   ├── app/                   # App Router pages
│   ├── components/            # React components
│   └── lib/                   # Utilities, MuJoCo integration
├── backend/                   # FastAPI application
│   ├── src/vlaarena_backend/
│   │   ├── api/               # Routers
│   │   └── services/          # Business logic
│   └── alembic/               # Database migrations
├── worker/                    # Vote aggregation worker
│   └── src/vlaarena_worker/
│       └── aggregators/       # ELO calculation
├── shared/                    # Shared Python code
│   ├── src/vlaarena_shared/
│   │   ├── models.py          # PostgreSQL models (SQLModel)
│   │   ├── schemas.py         # Pydantic schemas
│   │   ├── mongodb_models.py  # MongoDB models (Beanie)
│   │   └── mongodb.py         # MongoDB connection
│   └── tests/                 # Shared tests
├── WORKSPACE/                 # Documentation
├── .claude/                   # Skills & slash commands
├── docker-compose.yml         # PostgreSQL + MongoDB
└── pyproject.toml             # uv workspace config
```

---

## 🔑 Key Concepts

**VLA (Vision-Language-Action) Models:**
- Input: Camera observations + natural language instruction
- Output: 8-dim action vectors (position, rotation, gripper)
- Examples: OpenVLA 7B, Octo-base

**Episode:**
- Single execution of one VLA model for one instruction
- Stored in MongoDB: actions, states (qpos/qvel), metrics
- Max 50 steps, ~13KB per episode

**Battle:**
- Two episodes (left vs right) for same instruction
- Blind comparison (models hidden until vote)
- User votes: left_better, right_better, tie, both_bad

**ELO System:**
- Robot-specific rankings (e.g., WidowX leaderboard)
- Global rankings (across all robots)
- Bradley-Terry model with confidence intervals

---

## 🛠️ Common Workflows

### Starting New Work

```bash
/start-phase
# Creates feature branch, reads conventions, sets up environment
```

### Before Creating PR

```bash
/review-phase
# Runs self code review, updates WORKSPACE docs
```

### Creating PR

```bash
/create-pr
# Creates GitHub PR with English format, targeting develop
```

### Ending Phase

```bash
/end-phase
# Merges PR, cleans up branches, prepares for next phase
```

**Detailed workflows:** Use corresponding skills and slash commands

---

## 🚨 Critical Reminders

1. **ALWAYS check branch** before starting work
2. **NEVER use foreign keys** in database models
3. **ALWAYS use English** for PRs and commits
4. **NEVER edit `components/ui/`** directly (shadcn/ui components)
5. **ALWAYS use `uv`** for Python deps (NOT pip)
6. **ALWAYS run ruff** for linting (includes import sorting)
7. **MANDATORY Chrome DevTools MCP** verification for UI changes
8. **ALWAYS use --autogenerate** for Alembic migrations

**When in doubt:** Check WORKSPACE docs or invoke relevant skill

---

## 📞 Getting Help

1. **Documentation:** Check `WORKSPACE/` folder first
2. **Skills:** Invoke skill for specialized guidance (e.g., `fastapi-patterns`)
3. **Slash Commands:** Use `/help-kr` for Korean help guide
4. **Reference Project:** Consult `../lmarena-clone` for Phase 2 patterns

---

**Created:** 2025-11-01
**Last Updated:** 2025-11-07
**Current Phase:** MVP Development - VLA Server Phase 2 (Week 3-5)
**Current Feature:** `WORKSPACE/FEATURES/001_MVP.md`
