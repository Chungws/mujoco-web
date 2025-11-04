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
MVP Development                    [🔄 IN PROGRESS] (Week 2-7)
├─ Backend Foundation              (Week 2-3)
├─ VLA & MuJoCo Integration       (Week 4-5)
├─ Frontend & Worker              (Week 6)
└─ Testing & Polish               (Week 7)
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

### Deliverables
- [x] WORKSPACE/00_PROJECT.md (VLA Arena overview)
- [x] WORKSPACE/ROADMAP.md (this file)
- [x] Backend, Frontend, Shared, Worker structure (from lmarena-clone)
- [x] llmbattler → vlaarena rename complete
- [x] uv workspace configured

---

## 🔄 MVP Development (Current)

**Status:** In Progress 🔄
**Duration:** 6 weeks (Week 2-7)
**Target:** Functional VLA Arena with minimal features

### MVP Scope

**Included:**
- ✅ 1 robot (WidowX)
- ✅ 1 scene (Table simple manipulation)
- ✅ 2 VLA models (OpenVLA 7B, Octo-base)
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
- [ ] **Database Setup**
  - [ ] PostgreSQL via Docker Compose
  - [ ] MongoDB via Docker Compose (for episode data)
  - [ ] Alembic migration setup
  - [ ] Create initial migration (sessions, battles, turns, votes, model_stats_by_robot, model_stats_total, worker_status)
  - [ ] MongoDB connection setup (Motor async driver)
  - [ ] See [ADR-002](./ARCHITECTURE/ADR_002-Database_Schema.md) for complete schema

- [ ] **Shared Models & Schemas**
  - [ ] Session model (session_id, robot_id, scene_id, user_id, timestamps)
  - [ ] Battle model (battle_id, session_id, model_a_id, model_b_id, seq)
  - [ ] Turn model (turn_id, battle_id, instruction, episode_a_id, episode_b_id, seq)
  - [ ] Vote model (vote_id, turn_id, winner, robot_id, scene_id - denormalized)
  - [ ] ModelStatsByRobot model (model_id, robot_id, elo_score, CI, counts)
  - [ ] ModelStatsTotal model (model_id, global_elo_score, organization, license)
  - [ ] WorkerStatus model (status tracking)
  - [ ] MongoDB Episode document schema (actions, states, metrics)
  - [ ] API schemas (requests/responses)

- [ ] **Repository Layer**
  - [ ] SessionRepository (PostgreSQL CRUD)
  - [ ] BattleRepository (PostgreSQL CRUD)
  - [ ] TurnRepository (PostgreSQL CRUD)
  - [ ] VoteRepository (PostgreSQL CRUD)
  - [ ] EpisodeRepository (MongoDB CRUD - Motor async)
  - [ ] ModelStatsByRobotRepository (PostgreSQL CRUD)
  - [ ] ModelStatsTotalRepository (PostgreSQL CRUD)
  - [ ] Base repository pattern

- [ ] **Core APIs**
  - [ ] POST /api/battles/init (create session+battle, assign models)
  - [ ] GET /api/battles/{battle_id} (get battle details with turns)
  - [ ] POST /api/battles/{battle_id}/turns (create new turn with episodes)
  - [ ] POST /api/votes (submit vote for turn)
  - [ ] GET /api/models (list available models)
  - [ ] GET /api/leaderboard (robot-specific and global rankings)
  - [ ] Health check endpoint

#### Acceptance Criteria
- ✅ PostgreSQL + MongoDB running in Docker
- ✅ Migrations work (uv run alembic upgrade head)
- ✅ MongoDB connection works (Motor async queries)
- ✅ Battle can be created via API (session + battle + empty turns list)
- ✅ API docs at /docs functional
- ✅ Leaderboard API returns robot-specific and global ELO

---

### Week 4-5: VLA & MuJoCo Integration

**Goal:** Episode generation working end-to-end

#### Tasks
- [ ] **MuJoCo Service**
  - [ ] Install dm_control or mujoco-py
  - [ ] Environment creation (robot + scene XML)
  - [ ] State encode/decode (base64 serialization)
  - [ ] Observation extraction (camera rendering)
  - [ ] Step execution (action → next state)

- [ ] **VLA Service**
  - [ ] Model registry (config-based)
  - [ ] OpenVLA 7B integration
    - [ ] Reuse our FastAPI server code (from benchmark)
    - [ ] Or use transformers directly
  - [ ] Octo model integration
  - [ ] Inference latency simulation (200ms for OpenVLA, 50ms for Octo)

- [ ] **Episode Service**
  - [ ] POST /api/battles/{battle_id}/turns endpoint (embedded in turn creation)
  - [ ] Stateless episode generation:
    1. Load robot XML + scene configuration
    2. Initialize MuJoCo environments (parallel for A/B)
    3. Run up to max_steps (default 50, variable length)
    4. For each step:
       - Extract observation (camera rendering)
       - Call VLA model for action
       - Apply action to environment
       - Record state (qpos, qvel, time)
       - Check termination conditions
    5. Save episode to MongoDB:
       - actions[] (all actions taken)
       - states[] (qpos, qvel for replay)
       - metrics (success, total_steps, max_steps, terminated_early, etc.)
    6. Return episode_ids to Turn
  - [ ] Error handling (timeouts, failures, early termination)
  - [ ] Variable episode length support (terminated_early flag)

#### Acceptance Criteria
- ✅ MuJoCo env can be created from config
- ✅ State recording works (qpos, qvel at each step)
- ✅ OpenVLA + Octo both generate actions
- ✅ Turn creation endpoint returns 2 episode_ids
- ✅ Episodes saved to MongoDB with all states
- ✅ Variable episode length works (early termination detection)
- ✅ Episode size < 15KB per episode (~13KB for 50 steps)

---

### Week 6: Frontend & Worker

**Goal:** Complete user flow + ELO calculation

#### Tasks
- [ ] **Frontend Pages**
  - [ ] Arena page (from lmarena battle page)
    - [ ] Robot/Scene selector (first turn only)
    - [ ] Instruction input
    - [ ] "Generate Episode" button
    - [ ] Loading state (episode generation ~5-10s)
    - [ ] Side-by-side MuJoCo WASM replay viewers
    - [ ] Playback controls (play/pause, speed, step-by-step)
    - [ ] Vote buttons (A/B/Tie/Both Bad)
    - [ ] Model reveal after vote
    - [ ] Multi-turn support (continue with new instruction)
  - [ ] Leaderboard page (from lmarena leaderboard)
    - [ ] Robot filter tabs (WidowX, Global, etc.)
    - [ ] ELO rankings table (robot-specific + global)
    - [ ] Confidence intervals
    - [ ] Vote counts, win rates
  - [ ] History page (from lmarena sessions)
    - [ ] Past battles list (with turns)
    - [ ] Click to replay any turn
    - [ ] Model names revealed
    - [ ] Interactive state replay

- [ ] **Frontend Components**
  - [ ] MuJocoReplayViewer component (WASM-based)
  - [ ] StateTimeline component (step-by-step scrubber)
  - [ ] VotePanel component
  - [ ] RobotSceneSelector component
  - [ ] InstructionInput component
  - [ ] BattleSidebar component (multi-turn history)

- [ ] **API Integration**
  - [ ] API client (fetch wrapper)
  - [ ] Error handling
  - [ ] Loading states
  - [ ] Anonymous user ID (localStorage UUID)

- [ ] **Worker**
  - [ ] APScheduler setup (hourly cron)
  - [ ] Vote aggregation logic (from lmarena)
  - [ ] ELO calculation (K=32, initial=1500)
    - [ ] Robot-specific ELO (ModelStatsByRobot table)
    - [ ] Global ELO (ModelStatsTotal table)
    - [ ] Query votes with robot_id denormalized field
  - [ ] Bradley-Terry confidence intervals
  - [ ] Update both model_stats_by_robot and model_stats_total tables
  - [ ] Worker status tracking

#### Acceptance Criteria
- ✅ User can complete full battle flow (init → turn → vote → next turn)
- ✅ MuJoCo WASM replay works smoothly (30+ FPS)
- ✅ Step-by-step playback controls work
- ✅ Multi-turn battles work (new instructions on same session)
- ✅ Voting works, models revealed after vote
- ✅ Leaderboard displays robot-specific + global ELO correctly
- ✅ Worker runs hourly, updates both ELO tables

---

### Week 7: Testing & Polish

**Goal:** Production-ready MVP

#### Tasks
- [ ] **Testing**
  - [ ] Backend unit tests (pytest)
    - [ ] Repository tests
    - [ ] Service tests
    - [ ] API tests
  - [ ] Integration tests
    - [ ] Full episode generation flow
    - [ ] Vote → ELO update flow
  - [ ] Frontend tests
    - [ ] Component tests
    - [ ] E2E tests (Playwright)

- [ ] **Performance**
  - [ ] Episode generation < 10s (2 models × 50 steps)
  - [ ] API response < 500ms (p95) for GET requests
  - [ ] State recording overhead < 5% (vs no recording)
  - [ ] MongoDB insert < 100ms per episode
  - [ ] Episode fetch from MongoDB < 50ms
  - [ ] MuJoCo WASM replay initialization < 1s
  - [ ] Database query optimization (indexes on robot_id, scene_id, model_id)

- [ ] **Polish**
  - [ ] Error messages user-friendly
  - [ ] Loading states everywhere
  - [ ] Responsive design (desktop-first)
  - [ ] Accessibility (keyboard nav)

- [ ] **Documentation**
  - [ ] API documentation (FastAPI auto-docs)
  - [ ] User guide (how to use arena)
  - [ ] Developer guide (how to add models/robots)
  - [ ] Deployment guide (Docker Compose)

- [ ] **Deployment Prep**
  - [ ] Environment variables documented
  - [ ] Docker Compose production config
  - [ ] Database backups
  - [ ] Monitoring (optional)

#### Acceptance Criteria
- ✅ All tests pass
- ✅ Performance targets met
- ✅ Documentation complete
- ✅ MVP deployable

---

## 🎯 Current Status

**Week:** 2 (Backend Foundation)
**Active Tasks:**
- Setting up database migrations
- Implementing Session API
- Configuring Docker Compose

**Next Up:**
- Episode API scaffolding
- MuJoCo environment setup

---

## 📊 Progress Tracking

| Milestone | Status | Progress | Target Date |
|-----------|--------|----------|-------------|
| Phase 0: Setup | ✅ Complete | 100% | 2025-01-04 |
| Backend Foundation | 🔄 In Progress | 20% | Week 3 end |
| VLA & MuJoCo | ⏸️ Not Started | 0% | Week 5 end |
| Frontend & Worker | ⏸️ Not Started | 0% | Week 6 end |
| Testing & Polish | ⏸️ Not Started | 0% | Week 7 end |

**Overall MVP Progress:** 15% complete

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
- **[FEATURES/001_MVP.md](./FEATURES/001_MVP.md)** - MVP implementation guide
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

---

**Last Updated:** 2025-01-04
**Status:** MVP Week 2 - Backend Foundation
**Next Milestone:** Database migrations (PostgreSQL + MongoDB) + Battle API
**Target MVP Completion:** Week 7
