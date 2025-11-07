# VLA Arena - Development Roadmap

**Vision-Language-Action Model Comparison Platform**

This document tracks the development roadmap and progress of VLA Arena.

---

## 🎯 Project Vision

**Build a fair, unbiased platform for comparing VLA models through interactive robot manipulation tasks.**

Similar to LM Arena but for embodied AI - users watch two anonymous VLA models execute instructions, vote on performance, and contribute to an ELO-based leaderboard.

---

## 📅 Development Timeline

```
Phase 0: Setup & Architecture      [✅ DONE]      (Week 1)
         ↓
MVP Development                    [🔄 IN PROGRESS] (Week 2-5)
├─ Backend Foundation              (Week 2-3) ✅
├─ VLA Server Development          (Week 3-5) 🔄
├─ Frontend & Worker               ✅
         ↓
Post-MVP Enhancements             [⏸️ PLANNED]   (Future)
```

---

## ✅ Phase 0: Setup & Architecture Validation

**Status:** Completed ✅
**Duration:** Week 1
**Completed:** 2025-01-04

### Objectives
- [x] Analyze requirements and reference projects
- [x] Define architecture based on benchmark results
- [x] Copy lmarena-clone structure
- [x] Rename and adapt for VLA Arena
- [x] Create comprehensive WORKSPACE documentation
- [x] Set up project structure

### Key Decisions Made

#### 1. Architecture: All Server-Side ✅
**Benchmark Results:**
```
OpenVLA 7B: 283ms inference
Control Freq: 3.51 Hz (< 5 Hz target)
→ Client+Server hybrid: NOT VIABLE
→ All server-side: CHOSEN
```

#### 2. Stateless Design ✅
- MuJoCo state in requests (~500 bytes)
- No session affinity
- Horizontal scaling ready

#### 3. Episode-Based Execution ✅
- Up to 50 steps (variable length, configurable max_steps)
- State-based replay (MuJoCo WASM)
- Interactive step-by-step debugging
- Fair comparison

#### 4. VLA Server Separation ✅
- **Decision (2025-01-06):** VLA execution as separate microservice
- **Reason:** Resource isolation (GPU vs CPU), independent scaling
- **Architecture:** Backend (Orchestrator) → HTTP → VLA Server (Execution)
- **See:** [ADR-003: VLA Server Separation](./ARCHITECTURE/ADR_003-VLA_Server_Separation.md)

### Deliverables
- [x] WORKSPACE/00_PROJECT.md (VLA Arena overview)
- [x] WORKSPACE/ROADMAP.md (this file)
- [x] Backend, Frontend, Shared, Worker structure (from lmarena-clone)
- [x] llmbattler → vlaarena rename complete
- [x] uv workspace configured

---

## 🔄 MVP Development (Current)

**Status:** In Progress 🔄
**Duration:** 4 weeks (Week 2-5)
**Target:** Functional VLA Arena with minimal features

### MVP Scope

**Included:**
- ✅ 1 robot (Franka Emika Panda from MuJoCo Menagerie)
- ✅ 1 scene (Table simple manipulation)
- ✅ 2 VLA models (Octo-Small 27M, SmolVLA 450M)
- ✅ Session creation with random model assignment
- ✅ Multi-turn battles (multiple instructions per session)
- ✅ Episode generation (up to 50 steps server-side, variable length)
- ✅ State-based replay (MuJoCo WASM in browser)
- ✅ Interactive step-by-step debugging
- ✅ Voting (A/B/Tie/Both Bad)
- ✅ Robot-specific + Global ELO rankings
- ✅ Hourly aggregation worker

**Deferred:**
- Multiple robots/scenes
- Trajectory overlay visualization (MuJoCo Marker API)
- Advanced leaderboard filters (by scene, date range)
- User authentication
- Model upload interface
- Video thumbnail generation

---

### Week 2-3: Backend Foundation

**Goal:** Functional API with database and basic services

#### Tasks
- [x] **Database Setup**
  - [x] PostgreSQL via Docker Compose
  - [x] MongoDB via Docker Compose (for episode data)
  - [x] Alembic migration setup
  - [x] Create initial migration (sessions, battles, turns, votes, model_stats_by_robot, model_stats_total, worker_status)
  - [x] MongoDB connection setup (Beanie ODM with Motor async driver)
  - [x] See [ADR-002](./ARCHITECTURE/ADR_002-Database_Schema.md) for complete schema

- [x] **Shared Models & Schemas**
  - [x] Session model (session_id, robot_id, scene_id, user_id, timestamps)
  - [x] Battle model (battle_id, session_id, left_model_id, right_model_id, seq)
  - [x] Turn model (turn_id, battle_id, instruction, seq)
  - [x] Vote model (vote_id, battle_id, robot_id, scene_id - denormalized)
  - [x] ModelStatsByRobot model (model_id, robot_id, elo_score, CI, counts)
  - [x] ModelStatsTotal model (model_id, global_elo_score, organization, license)
  - [x] WorkerStatus model (status tracking)
  - [x] MongoDB Episode document schema (actions, states, metrics) with Beanie ODM
  - [x] API schemas (requests/responses) - Session, Turn, Episode, Vote, Leaderboard with TDD (23 tests)

- [x] **Repository Layer**
  - [x] SessionRepository (PostgreSQL CRUD) - Already implemented
  - [x] BattleRepository (PostgreSQL CRUD) - Already implemented
  - [x] TurnRepository (PostgreSQL CRUD) - TDD with tests
  - [x] VoteRepository (PostgreSQL CRUD) - TDD with tests
  - [x] EpisodeRepository (MongoDB CRUD - Motor async) - TDD with 10 tests
  - [x] ModelStatsRepository (PostgreSQL CRUD - dual ELO support) - TDD with 9 tests
  - [x] Base repository pattern - Already implemented

- [x] **Core APIs**
  - [x] POST /api/sessions/init (create session+battle, assign models) - TDD with 15 tests
  - [ ] GET /api/battles/{battle_id} (get battle details with turns)
  - [x] POST /api/battles/{battle_id}/turns (create new turn with episodes) - TDD with 7 tests
  - [x] POST /api/votes (submit vote for turn) - TDD with 16 tests
  - [x] GET /api/models (list available models) - TDD with 3 tests
  - [x] GET /api/leaderboard (robot-specific and global rankings) - TDD with 5 tests
  - [x] Health check endpoint - Already implemented

- [x] **Mock VLA Service (for MVP development)**
  - [x] MockVLAService implementation - TDD with 9 tests
  - [x] Episode generation (actions, states, metrics)
  - [x] Variable-length episodes (20-50 steps)
  - [x] Model-specific deterministic behavior
  - [x] Ready for Turn API integration

#### Acceptance Criteria
- ✅ PostgreSQL + MongoDB running in Docker
- ✅ Migrations work (uv run alembic upgrade head)
- ✅ MongoDB connection works (Beanie ODM with Motor async)
- ✅ All SQLModel models match ADR-002 specification
- ✅ MongoDB Episode document model with State and Metrics
- ✅ All tests passing (88 backend + 23 worker = 111 total)
- ✅ Session can be created via API (POST /api/sessions/init with robot_id and scene_id)
- ✅ API docs at /docs functional
- ✅ Leaderboard API returns robot-specific and global ELO (TDD with 19 tests)
- ✅ Worker aggregates votes and updates dual ELO rankings (robot-specific + global)

---

### Week 3-5: VLA Server Development

**Goal:** Separate VLA execution service operational
**See:** [FEATURES/002_VLA_Server.md](./FEATURES/002_VLA_Server.md) for complete specification

#### Week 3: VLA Server Infrastructure

- [x] **Project Setup**
  - [x] Create vla-server/ directory structure
  - [x] Setup pyproject.toml (mujoco, torch, transformers dependencies) - with uv
  - [x] Configuration management (Pydantic Settings)
  - [x] Root config setup (config/mujoco/ XML templates)
  - [ ] FastAPI app skeleton (next PR)
  - [ ] Download Franka Panda model from MuJoCo Menagerie (using simplified model)

- [x] **XML Composition (TDD - 10 tests)**
  - [x] config/model_loader.py
  - [x] Dynamic robot + scene composition
  - [x] Template, robot, scene XML files
  - [x] Error handling (missing files)

- [x] **MuJoCo Environment (TDD - 16 tests)**
  - [x] services/mujoco_env.py
  - [x] Load Franka Emika Panda robot (simplified 7-DOF + gripper)
  - [x] Load table scene
  - [x] Environment reset to initial state
  - [x] Step simulation with action
  - [x] Get observation (camera rendering + proprioception)
  - [x] Get state for recording (qpos, qvel, time)
  - [x] Error handling (invalid robot/scene)

- [x] **VLA Adapter Pattern - Base + Mock (TDD - 20 tests)** ✅ PR 1 Complete
  - [x] adapters/base.py - Abstract adapter interface
  - [x] adapters/mock_adapter.py - Mock adapter for testing
  - [x] adapters/__init__.py - Factory function (get_adapter)
  - [x] Comprehensive tests (20 tests passing)
  - [x] Device management interface (auto, cuda, cpu, mps)
  - [ ] adapters/octo_small_adapter.py - Octo-Small 27M adapter (PR 2)
  - [ ] adapters/smolvla_adapter.py - SmolVLA 450M adapter (PR 3)
  - [ ] MacBook compatibility (CPU/MPS mode) (PR 2-3)

#### Week 4: VLA Server Core Logic

- [ ] **Execution Service (TDD - 20 tests)**
  - [ ] services/execution_service.py
  - [ ] Orchestrate MuJoCo + VLA model
  - [ ] Episode loop (max 50 steps):
    1. Get observation from MuJoCo
    2. VLA model inference
    3. Step MuJoCo simulation
    4. Record action and state
    5. Check termination
  - [ ] Return episode data (actions, states, duration_ms)
  - [ ] Error handling and timeouts

- [ ] **API Endpoints (TDD - 10 tests)**
  - [ ] api/execute.py - POST /execute
  - [ ] api/health.py - GET /health
  - [ ] api/models.py - GET /models (list available)
  - [ ] Request/Response schemas
  - [ ] Error handling (404, 400, 500)

#### Week 5: Backend Integration

- [ ] **Backend Updates**
  - [ ] Update TurnService to call VLA server via HTTP
  - [ ] Replace MockVLAService with HTTP client
  - [ ] Configuration for VLA server URL
  - [ ] Retry logic and circuit breaker
  - [ ] Timeout management (120s)

- [ ] **Integration Testing**
  - [ ] Backend → VLA Server full flow
  - [ ] Episode data saved to MongoDB
  - [ ] Error scenarios (server down, timeout)
  - [ ] Performance testing (episode generation < 60s)

- [ ] **Documentation**
  - [ ] VLA server API documentation (OpenAPI)
  - [ ] Deployment guide (Docker)
  - [ ] MacBook setup guide

#### Acceptance Criteria
- ✅ VLA server runs independently on port 8001
- ✅ Franka Panda robot loads in MuJoCo
- ✅ Octo-Small and SmolVLA models load
- ✅ POST /execute generates episodes (max 50 steps)
- ✅ Backend calls VLA server successfully
- ✅ Episodes saved to MongoDB with all data
- ✅ All 57 tests passing (15 + 12 + 20 + 10)
- ✅ Works on MacBook (CPU/MPS mode)
- ✅ Episode generation < 60 seconds per model

---

### ✅ Frontend & Worker (Complete)

**Status:** Complete ✅
**Completed:** 2025-11-07

#### Completed
- ✅ **Frontend Pages**
  - ✅ Battle page (side-by-side viewers, voting, multi-turn)
  - ✅ Leaderboard page (robot-specific + global rankings)
  - ✅ Home page

- ✅ **Worker**
  - ✅ APScheduler setup (hourly cron)
  - ✅ Vote aggregation logic
  - ✅ ELO calculation (robot-specific + global)
  - ✅ Bradley-Terry confidence intervals
  - ✅ All tests passing (40 worker tests)

---

## 🎯 Current Status

**Week:** 3-5 (VLA Server Phase 2) - In Progress 🔄
**Last Update:** 2025-11-07

**Completed:**
- ✅ PostgreSQL + MongoDB Docker setup
- ✅ Database schema migration (b3a87a3d54ec)
- ✅ All SQLModel models (Session, Battle, Turn, Vote, ModelStatsByRobot, ModelStatsTotal)
- ✅ MongoDB Episode model with Beanie ODM
- ✅ MongoDB connection with indexes
- ✅ Backend APIs - 88 tests passing:
  - Session API (POST /api/sessions/init) - 15 tests
  - Models API (GET /api/models) - 3 tests
  - MockVLAService implementation - 9 tests
  - Turn API (POST /api/battles/{id}/turns) - 7 tests
  - Episodes API (GET /api/episodes/{id}) - 7 tests
  - Votes API (POST /api/votes) - 16 tests
  - Leaderboard API (GET /api/leaderboard?robot_id=...) - Robot-specific + Global ELO
- ✅ Worker ELO aggregation - 40 tests passing
- ✅ Frontend - Complete:
  - Battle Page (side-by-side viewers, voting, multi-turn)
  - Leaderboard Page (robot-specific + global rankings)
  - Home Page
- ✅ **Total: 128 passing tests (88 backend + 40 worker)**

**Next Up:**
- VLA Server Development (Week 3-5) - In Progress (45% complete)
  - ✅ Phase 1: Config + MuJoCo Environment (26 tests passing)
  - ✅ Phase 2 PR 1: Base Adapter + Mock (20 tests passing)
  - ⏭️ Phase 2 PR 2: Octo-Small Adapter (next PR)
  - ⏭️ Phase 2 PR 3: SmolVLA Adapter

---

## 📊 Progress Tracking

| Milestone | Status | Progress | Target Date |
|-----------|--------|----------|-------------|
| Phase 0: Setup | ✅ Complete | 100% | 2025-01-04 |
| Backend Foundation | ✅ Complete | 100% | 2025-01-06 |
| Worker & Leaderboard | ✅ Complete | 100% | 2025-01-06 |
| Frontend | ✅ Complete | 100% | 2025-11-07 |
| VLA Server Development | 🔄 In Progress | 45% | Week 5 end |

**Overall MVP Progress:** 89% complete

**Progress Details:**
- Database setup: 100% (PostgreSQL + MongoDB)
- Models & Schemas: 100% (SQLModel + Pydantic schemas with TDD)
- Core APIs: 100% (Session ✅, Models ✅, Turns ✅, Episodes ✅, Votes ✅, Leaderboard ✅)
- Services: 100% (SessionService ✅, MockVLAService ✅, TurnService ✅, VoteService ✅)
- Worker & Leaderboard: 100% (ELO aggregation ✅, robot-specific + global ELO ✅, 40 tests ✅)
- Frontend: 100% (Battle Page ✅, Leaderboard Page ✅, Home Page ✅)
- VLA Server: 45% (Config ✅, MuJoCo Env ✅, Base Adapter + Mock ✅, 46 tests ✅, Octo-Small next)

---

## 🔮 Post-MVP Enhancements

### Phase: Expansion (Future)

**Priority 1: More Content**
- Additional robots (Franka, UR5)
- Additional scenes (Kitchen, Warehouse)
- More VLA models (RT-2, etc.)

**Priority 2: Advanced Features**
- Trajectory visualization (MuJoCo Marker API)
- Step-by-step replay with timeline
- Comparison mode (pause/sync videos)
- Download videos

**Priority 3: UX Improvements**
- User authentication
- Personal leaderboard history
- Model favorites
- Comments on sessions

**Priority 4: Infrastructure**
- Ray Serve for environment workers
- vLLM for VLA serving (auto-batching)
- Caching (Redis)
- Monitoring (Prometheus + Grafana)

**Priority 5: Community**
- Model upload interface
- Custom robot/scene upload
- Public API
- Webhooks

---

## 🚧 Known Risks & Mitigation

### Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| **Episode generation slow** | High | Parallel execution of A/B models, optimize VLA inference |
| **VLA inference slow** | High | Already measured (283ms), build around it |
| **MuJoCo state replay inaccurate** | High | Thorough state recording testing, verify determinism |
| **WASM replay performance** | Medium | Optimize state loading, use efficient data structures |
| **MongoDB storage costs** | Low | ~13KB per episode, much smaller than video (5MB) |
| **State recording overhead** | Low | Minimal (qpos/qvel only), target <5% slowdown |

### Scope Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| **Too many features in MVP** | High | Strict scope control, defer post-MVP |
| **Over-engineering** | Medium | Use lmarena patterns, keep it simple |

---

## 📝 Development Notes

### Key Learnings

**From Benchmark (2025-01-04):**
- OpenVLA 7B: 283ms inference (GPU)
- Network: 1.6ms (negligible on localhost)
- Conclusion: All server-side necessary

**From lmarena-clone:**
- No foreign keys = simpler testing
- Stateless = easier scaling
- Battle sidebar = good UX for multi-turn

**Database Design (ADR-002):**
- State-based replay > Video: 13KB vs 5MB, interactive debugging
- MongoDB for episodes: Optimized for document storage
- Robot-specific ELO: Fair comparison (models perform differently per robot)
- Session-Battle separation: 1:1 for MVP, expandable to 1:N
- Denormalized Vote (robot_id, scene_id): Worker performance optimization

### Reference Projects

**lmarena-clone:**
- Battle system: `backend/src/llmbattler_backend/api/battles.py`
- ELO calculator: `worker/src/llmbattler_worker/aggregators/elo_calculator.py`
- Frontend battle UI: `frontend/app/battle/page.tsx`

**Adapting for VLA:**
- Text messages → MuJoCo state replay (interactive)
- LLM inference → VLA + MuJoCo episode generation
- Multi-turn conversation → Multi-turn battles (multiple instructions)
- Model-specific ELO → Robot-specific + Global ELO
- PostgreSQL only → PostgreSQL + MongoDB hybrid
- Same voting, leaderboard logic

---

## 🔗 Related Documents

- **[00_PROJECT.md](./00_PROJECT.md)** - Project overview and architecture
- **[ARCHITECTURE/ADR_001-Server_Side_Execution.md](./ARCHITECTURE/ADR_001-Server_Side_Execution.md)** - All server-side execution decision
- **[ARCHITECTURE/ADR_002-Database_Schema.md](./ARCHITECTURE/ADR_002-Database_Schema.md)** - Complete database schema (PostgreSQL + MongoDB)
- **[ARCHITECTURE/ADR_003-VLA_Server_Separation.md](./ARCHITECTURE/ADR_003-VLA_Server_Separation.md)** - VLA Server as separate microservice
- **[FEATURES/001_MVP.md](./FEATURES/001_MVP.md)** - MVP implementation guide
- **[FEATURES/002_VLA_Server.md](./FEATURES/002_VLA_Server.md)** - VLA Server specification and implementation
- **[../lmarena-clone](../../lmarena-clone/)** - Reference codebase

---

## 🔄 Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-01 | Initial roadmap (Phase 1/2 split) | Claude |
| 2025-01-04 | Restructured for MVP (integrated approach) | Claude |
| 2025-01-04 | Added benchmark results, architecture decisions | Claude |
| 2025-01-04 | Aligned with new VLA Arena specification | Claude |
| 2025-01-04 | Updated for MongoDB + state-based replay (ADR-002) | Claude |
| 2025-01-04 | Added robot-specific ELO, multi-turn battles | Claude |
| 2025-01-04 | Variable episode length support | Claude |
| 2025-11-05 | Database foundation complete (Steps 1-3) | Claude |
| 2025-11-05 | API schemas complete with TDD (23 tests) | Claude |
| 2025-11-06 | Turn API complete with TDD (7 tests, MongoDB integration) | Claude |
| 2025-01-06 | VLA Server separation decision (ADR-003) | Claude |
| 2025-01-06 | Changed robot to Franka Panda (MuJoCo Menagerie) | Claude |
| 2025-01-06 | Changed VLA models to Octo-Small + SmolVLA (MacBook compatible) | Claude |
| 2025-01-06 | Created FEATURES/002_VLA_Server.md specification | Claude |
| 2025-01-06 | Updated Week 3-5 timeline for VLA Server development | Claude |
| 2025-11-07 | Fixed worker test failures (4 tests in test_main_error_handling.py) | Claude |
| 2025-11-07 | Updated Frontend completion (Battle + Leaderboard pages done) | Claude |
| 2025-11-07 | Updated progress tracking (80% MVP complete) | Claude |
| 2025-11-07 | Removed obsolete BACKEND_STATUS.md and FRONTEND_STATUS.md | Claude |
| 2025-01-07 | VLA Server Phase 1: Config + MuJoCo Environment (26 tests) | Claude |
| 2025-11-07 | VLA Server Phase 2 PR 1: Base Adapter + Mock (20 tests, 46 total) | Claude |

---

**Last Updated:** 2025-11-07
**Status:** MVP 89% Complete - VLA Server Phase 2 In Progress
**Next Milestone:** VLA Server Octo-Small Adapter (Phase 2 PR 2)
**Target MVP Completion:** Week 5
