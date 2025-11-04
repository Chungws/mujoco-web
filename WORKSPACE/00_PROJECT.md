# VLA Arena - Project Overview

**Vision-Language-Action Model Comparison Platform**

A web-based arena for comparing VLA models through blind A/B testing with robot manipulation tasks. Similar to LM Arena but for embodied AI models.

---

## 📋 Project Information

| Category | Details |
|----------|---------|
| **Project Name** | VLA Arena (mujoco-web) |
| **Repository** | mujoco-web |
| **Tech Stack** | Next.js 15, FastAPI, PostgreSQL, MuJoCo, Ray, vLLM/TGI |
| **Architecture** | Stateless microservices |
| **Development Start** | 2025-01 |
| **Status** | MVP Development |

---

## 🎯 Project Mission

**Enable fair, unbiased comparison of Vision-Language-Action models through interactive robot manipulation tasks.**

Users select a robot and scene, provide natural language instructions, watch two anonymized VLA models execute in parallel, vote on performance, and contribute to an ELO-based leaderboard.

---

## 🔴 Critical Project Decisions

### Architecture Decision (Based on Benchmark Results)

**Decision:** **All Server-Side Execution**

**Benchmark Results (2025-01):**
```
OpenVLA 7B Inference: 283ms (mean)
Network Latency: 1.6ms (localhost)
Total: 284.7ms → 3.51 Hz

Target: ≥ 5 Hz
Result: FAIL ❌
```

**Conclusion:**
- ❌ Client MuJoCo + Server VLA: Too slow (3.51 Hz < 5 Hz target)
- ✅ **Server MuJoCo + Server VLA: Chosen architecture**

**Benefits:**
- No network latency in control loop
- Consistent performance (GPU)
- Fair model comparison
- Episode recording for replay

See [ADR-001](./ARCHITECTURE/ADR_001-Server_Side_Execution.md) for details.

---

## 🔑 Core Concepts

### 1. Stateless Architecture

**All components are stateless:**
- MuJoCo state (~500 bytes) passed in requests
- No session affinity required
- Any worker can handle any request
- Horizontal scaling friendly

### 2. Episode-Based Execution

```
User Flow:
1. Select Robot (WidowX/Franka/UR5) + Scene (Table/Kitchen/Warehouse)
2. System assigns 2 anonymous VLA models
3. User provides instruction
4. Server generates 50-step episodes (5 seconds each)
5. User watches side-by-side videos
6. User votes (A/B/Tie)
7. Models revealed, ELO updated
```

**Episode = 50 steps = 5 seconds @ 10Hz control**

### 3. Blind A/B Testing

- Models assigned randomly
- Identities hidden until after vote
- Left/right position randomized
- Eliminates brand bias

### 4. Inference Latency Simulation

```python
Realistic latencies simulated:
- OpenVLA 7B: 200ms (based on our 283ms benchmark)
- RT-2: 100ms
- Octo: 50ms

Implementation:
- During latency period, previous action held
- No actual sleep (simulation time only)
- Fair comparison across models
```

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────┐
│         Frontend (Next.js)                   │
│  - Robot/Scene selector                      │
│  - Instruction input                         │
│  - Side-by-side video players               │
│  - Vote buttons (A/B/Tie)                    │
│  - Session history (past battles)            │
│  - Leaderboard (ELO rankings)                │
└──────────────────┬──────────────────────────┘
                   │ HTTP REST API
     ┌─────────────▼─────────────────────────┐
     │   Orchestrator (FastAPI, Stateless)    │
     │   - Session management                 │
     │   - Episode coordination               │
     │   - Vote processing                    │
     └──────┬──────────────┬──────────────────┘
            │              │
   ┌────────▼────────┐  ┌─▼──────────────────┐
   │ VLA Inference   │  │ Environment Workers │
   │ (vLLM/TGI)      │  │ (Ray, Stateless)    │
   │                 │  │                     │
   │ - OpenVLA 7B    │  │ MuJoCo Episodes:    │
   │ - RT-2          │  │ 1. Restore state    │
   │ - Octo          │  │ 2. Run 50 steps     │
   │ - Auto-batching │  │ 3. Record trajectory │
   │                 │  │ 4. Generate video   │
   └─────────────────┘  └─────────────────────┘
            │                      │
            └──────────┬───────────┘
                       │
            ┌──────────▼───────────┐
            │   Storage & Database  │
            │   - PostgreSQL        │
            │   - S3/MinIO (videos) │
            └──────────┬────────────┘
                       │
            ┌──────────▼───────────┐
            │   Worker (Python)     │
            │   - Hourly ELO calc   │
            │   - APScheduler       │
            └──────────────────────┘
```

### Key Components

#### 1. Orchestrator (FastAPI)
- **Stateless**: No in-memory sessions
- **API Gateway**: Coordinates all requests
- **Endpoints:**
  - `POST /session/init` - Create session, assign models
  - `POST /session/execute` - Generate episode
  - `POST /session/vote` - Submit vote
  - `GET /leaderboard` - Get rankings

#### 2. Environment Workers (Ray Serve)
- **Stateless**: State passed in request
- **MuJoCo Simulation**: dm_control or mujoco-py
- **Episode Generation:**
  ```python
  1. Decode base64 state
  2. Restore MuJoCo environment
  3. Loop 50 steps:
     - Get observation
     - Call VLA model
     - Apply action
     - Record frame
  4. Encode video
  5. Upload to S3
  6. Return new state + video URL
  ```

#### 3. VLA Inference (vLLM/TGI)
- **Auto-batching**: Automatically batch multiple requests
- **GPU Optimized**: Efficient inference
- **Model Serving**: Multiple models simultaneously

#### 4. Storage
- **PostgreSQL**: Sessions, episodes, votes, model stats
- **S3/MinIO**: Video files, trajectory data

#### 5. Worker
- **Hourly Aggregation**: ELO calculation
- **Bradley-Terry**: Confidence intervals

---

## 🔴 Project Policies

| Policy | This Project | Reference |
|--------|--------------|-----------|
| **Foreign Keys** | ❌ **NO FKs** | lmarena-clone ADR-001 |
| **State Management** | ✅ **Stateless** | All components |
| **Main Branch** | ✅ `main` | Standard Git Flow |
| **PR Language** | ✅ **English** | - |
| **Git Host** | ✅ **GitHub** | - |
| **Package Manager** | ✅ **uv** (Backend), **npm** (Frontend) | - |

### 🚨 Key Policies

#### 1. No Foreign Keys
- ❌ NO database-level FK constraints
- ✅ Application-level relationships
- **Benefit**: Simpler testing, faster development

#### 2. Stateless Components
- ❌ NO in-memory sessions
- ✅ Pass state in requests/responses
- **Benefit**: Horizontal scaling, any worker can handle any request

#### 3. PRs Always in English
- Title, description, code comments in English
- Korean discussions OK in Slack/internal docs

---

## ⚡ Quick Start

### Prerequisites
- **Python**: 3.11+
- **Node.js**: 18+
- **Docker**: For PostgreSQL, MinIO
- **GPU**: NVIDIA GPU with CUDA (for VLA inference)

### Setup (MVP)

```bash
# 1. Clone repository
git clone [repo-url]
cd mujoco-web

# 2. Install dependencies (uv workspace)
uv sync

# 3. Start infrastructure
make dev-infra  # PostgreSQL + MinIO in Docker

# 4. Run database migrations
make migrate

# 5. Start services (3 terminals)

# Terminal 1: Backend API
make dev-backend  # http://localhost:8000

# Terminal 2: Frontend
make dev-frontend  # http://localhost:3000

# Terminal 3: Worker (optional)
make dev-worker
```

### Quick Commands

```bash
make help         # Show all commands
make setup        # Initial setup
make dev-infra    # Start PostgreSQL + MinIO
make dev-backend  # Start FastAPI
make dev-frontend # Start Next.js
make dev-worker   # Start ELO worker
make stop         # Stop all services
make test         # Run tests
make lint         # Lint all code
```

---

## 📁 Project Structure

```
mujoco-web/
├── backend/                    # FastAPI Orchestrator
│   ├── src/vlaarena_backend/
│   │   ├── api/
│   │   │   ├── sessions.py     # Session CRUD
│   │   │   ├── episodes.py     # Episode generation
│   │   │   ├── models.py       # Model management
│   │   │   └── leaderboard.py  # Rankings
│   │   ├── repositories/       # DB access layer
│   │   ├── services/
│   │   │   ├── session_service.py
│   │   │   ├── vla_service.py      # VLA client
│   │   │   ├── mujoco_service.py   # MuJoCo sim
│   │   │   └── episode_service.py  # Episode logic
│   │   ├── main.py
│   │   └── database.py
│   └── alembic/                # DB migrations
│
├── shared/                     # Shared Python code
│   └── src/vlaarena_shared/
│       ├── models.py           # SQLModel schemas
│       ├── schemas.py          # API schemas
│       └── config.py
│
├── worker/                     # ELO aggregation
│   └── src/vlaarena_worker/
│       ├── main.py
│       └── aggregators/
│           └── elo_calculator.py
│
├── frontend/                   # Next.js UI
│   ├── app/
│   │   ├── arena/             # Main battle page
│   │   ├── leaderboard/       # Rankings
│   │   └── history/           # Past sessions
│   ├── components/
│   │   ├── arena/             # Video players, controls
│   │   └── leaderboard/       # Ranking tables
│   └── lib/
│       └── api-client.ts      # API client
│
├── config/                     # Robot/Scene configs
│   ├── robots.yaml            # WidowX, Franka, UR5
│   └── scenes.yaml            # Table, Kitchen, Warehouse
│
├── WORKSPACE/                  # Documentation
│   ├── 00_PROJECT.md          # This file
│   ├── ROADMAP.md
│   ├── ARCHITECTURE/
│   │   └── ADR_001-*.md
│   └── FEATURES/
│
├── .claude/                    # Claude Code skills
├── docker-compose.yml
├── Makefile
└── pyproject.toml             # uv workspace
```

---

## 🗄️ Database Schema

```sql
-- Session
CREATE TABLE sessions (
    id SERIAL PRIMARY KEY,
    user_id TEXT NOT NULL,
    robot_type TEXT NOT NULL,
    scene_type TEXT NOT NULL,
    model_a_id TEXT NOT NULL,
    model_b_id TEXT NOT NULL,
    status TEXT DEFAULT 'active',
    vote TEXT,  -- 'A', 'B', 'Tie'
    created_at TIMESTAMP,
    completed_at TIMESTAMP
);

-- Episode (one per instruction in a session)
CREATE TABLE episodes (
    id SERIAL PRIMARY KEY,
    session_id INT NOT NULL,  -- No FK!
    instruction TEXT NOT NULL,
    model_a_state TEXT,       -- base64 MuJoCo state
    model_b_state TEXT,
    model_a_video_url TEXT,
    model_b_video_url TEXT,
    model_a_steps JSONB,      -- Trajectory data
    model_b_steps JSONB,
    created_at TIMESTAMP
);

-- Model Stats
CREATE TABLE model_stats (
    model_id TEXT PRIMARY KEY,
    robot_type TEXT,
    scene_type TEXT,
    elo_rating FLOAT DEFAULT 1500,
    elo_ci_lower FLOAT,
    elo_ci_upper FLOAT,
    vote_count INT DEFAULT 0,
    win_rate FLOAT,
    updated_at TIMESTAMP
);

-- Worker Status
CREATE TABLE worker_status (
    id SERIAL PRIMARY KEY,
    last_run TIMESTAMP,
    status TEXT,
    votes_processed INT
);
```

**Note:** No foreign key constraints (following lmarena-clone pattern)

---

## 📊 MVP Scope

### MVP Phase (4-6 weeks)

**Goal:** Working VLA Arena with minimal features

**Included:**
- ✅ 1 robot (WidowX)
- ✅ 1 scene (Table)
- ✅ 2 VLA models (OpenVLA 7B, Octo)
- ✅ Session management
- ✅ Episode generation (50 steps)
- ✅ Video playback
- ✅ Voting (A/B/Tie)
- ✅ Basic leaderboard
- ✅ Hourly ELO calculation

**Deferred (Post-MVP):**
- Multiple robots/scenes
- Trajectory visualization (MuJoCo Marker API)
- Step-by-step replay
- Advanced leaderboard filtering
- User authentication
- Model upload

---

## 🧪 Development Workflow

### Backend Development

```bash
# Install dependencies
uv sync

# Run linting
uvx ruff check backend/ shared/ worker/
uvx ruff format backend/ shared/ worker/

# Run tests
uv run pytest -s

# Database migrations
uv run alembic revision --autogenerate -m "description"
uv run alembic upgrade head

# Start server
uv run python -m uvicorn vlaarena_backend.main:app --reload
```

### Frontend Development

```bash
cd frontend

# Install dependencies
npm install

# Development
npm run dev

# Linting
npm run lint

# Build
npm run build
```

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| **[00_PROJECT.md](./00_PROJECT.md)** | This file - Project overview |
| **[ROADMAP.md](./ROADMAP.md)** | Development roadmap |
| **[ARCHITECTURE/](./ARCHITECTURE/)** | Architecture Decision Records |
| **[FEATURES/](./FEATURES/)** | Feature specifications |
| **[../lmarena-clone](../../lmarena-clone/)** | Reference project |

---

## 🎯 Success Metrics

### MVP Success Criteria

1. ✅ Users can complete full arena session
2. ✅ Episode generation < 10 seconds
3. ✅ Video playback smooth (30 FPS)
4. ✅ Voting works correctly
5. ✅ ELO scores update hourly
6. ✅ Leaderboard displays correctly

### Performance Targets

- **Episode Generation**: < 10 seconds (50 steps)
- **Video Encoding**: < 2 seconds
- **API Response**: < 500ms (95th percentile)
- **Page Load**: < 2 seconds
- **Concurrent Sessions**: 25+

---

## 🔗 Resources

### Technical References
- **MuJoCo**: https://mujoco.org
- **MuJoCo Python**: https://mujoco.readthedocs.io/en/stable/python.html
- **FastAPI**: https://fastapi.tiangolo.com
- **Ray Serve**: https://docs.ray.io/en/latest/serve/
- **vLLM**: https://docs.vllm.ai/
- **Next.js**: https://nextjs.org/docs

### Related Projects
- **lmarena-clone**: `../lmarena-clone/` (reference architecture)
- **LM Arena**: https://lmarena.ai (inspiration)
- **Chatbot Arena**: https://chat.lmsys.org (original concept)

### VLA Research
- **OpenVLA**: https://openvla.github.io/
- **RT-1**: https://arxiv.org/abs/2212.06817
- **RT-2**: https://arxiv.org/abs/2307.15818
- **Octo**: https://octo-models.github.io/

---

## 🤝 Contributing

### Before Contributing
1. Read this document
2. Check [ROADMAP.md](./ROADMAP.md) for current status
3. Review [ARCHITECTURE/](./ARCHITECTURE/) for design decisions
4. Check [.claude/skills/](../.claude/skills/) for project conventions

### Key Conventions
- ❌ **NO Foreign Keys** in database
- ✅ **Stateless** all components
- ✅ **English** for all PRs
- ✅ **uv** for Python dependencies
- ✅ **Type hints** in Python code
- ✅ **TypeScript** in frontend

---

**Last Updated:** 2025-01-04
**Status:** MVP Development
**Next Milestone:** Session Management API
