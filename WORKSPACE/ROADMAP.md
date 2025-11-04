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
- 50 steps = 5 seconds
- Side-by-side videos
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
- ✅ Episode generation (50 steps server-side)
- ✅ Side-by-side video playback
- ✅ Voting (A/B/Tie)
- ✅ ELO leaderboard
- ✅ Hourly aggregation worker

**Deferred:**
- Multiple robots/scenes
- Trajectory overlay visualization
- Step-by-step replay
- Advanced leaderboard filters
- User authentication
- Model upload interface

---

### Week 2-3: Backend Foundation

**Goal:** Functional API with database and basic services

#### Tasks
- [ ] **Database Setup**
  - [ ] PostgreSQL via Docker Compose
  - [ ] MinIO (S3-compatible) for videos
  - [ ] Alembic migration setup
  - [ ] Create initial migration (sessions, episodes, model_stats, worker_status)

- [ ] **Shared Models & Schemas**
  - [ ] Session model (user_id, robot, scene, models, vote)
  - [ ] Episode model (instruction, states, video URLs, steps)
  - [ ] ModelStats model (ELO, CI, counts)
  - [ ] API schemas (requests/responses)

- [ ] **Repository Layer**
  - [ ] SessionRepository (CRUD)
  - [ ] EpisodeRepository (CRUD)
  - [ ] ModelStatsRepository (CRUD)
  - [ ] Base repository pattern

- [ ] **Core APIs**
  - [ ] POST /api/session/init (create session, assign models)
  - [ ] GET /api/session/{id} (get session details)
  - [ ] GET /api/models (list available models)
  - [ ] Health check endpoint

#### Acceptance Criteria
- ✅ PostgreSQL + MinIO running in Docker
- ✅ Migrations work (uv run alembic upgrade head)
- ✅ Session can be created via API
- ✅ API docs at /docs functional

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
  - [ ] POST /api/session/execute endpoint
  - [ ] Stateless episode generation:
    1. Decode states
    2. Restore MuJoCo envs
    3. Run 50 steps (parallel for A/B)
    4. Call VLA models
    5. Record trajectories
    6. Render videos
    7. Upload to MinIO
    8. Save to database
  - [ ] Error handling (timeouts, failures)

- [ ] **Video Generation**
  - [ ] Frame-by-frame rendering
  - [ ] Video encoding (mp4, 30 FPS)
  - [ ] MinIO upload
  - [ ] URL generation

#### Acceptance Criteria
- ✅ MuJoCo env can be created from config
- ✅ State serialization/deserialization works
- ✅ OpenVLA + Octo both generate actions
- ✅ Episode endpoint returns 2 videos + new states
- ✅ Videos playable in browser

---

### Week 6: Frontend & Worker

**Goal:** Complete user flow + ELO calculation

#### Tasks
- [ ] **Frontend Pages**
  - [ ] Arena page (from lmarena battle page)
    - [ ] Robot/Scene selector
    - [ ] Instruction input
    - [ ] "Generate Episode" button
    - [ ] Loading state (episode generation)
    - [ ] Side-by-side video players
    - [ ] Vote buttons (A/B/Tie)
    - [ ] Model reveal after vote
  - [ ] Leaderboard page (from lmarena leaderboard)
    - [ ] ELO rankings table
    - [ ] Confidence intervals
    - [ ] Vote counts
  - [ ] History page (from lmarena sessions)
    - [ ] Past sessions list
    - [ ] Click to replay
    - [ ] Model names revealed

- [ ] **Frontend Components**
  - [ ] VideoPlayer component
  - [ ] VotePanel component
  - [ ] RobotSceneSelector component
  - [ ] InstructionInput component
  - [ ] SessionSidebar component (ChatGPT-like)

- [ ] **API Integration**
  - [ ] API client (fetch wrapper)
  - [ ] Error handling
  - [ ] Loading states
  - [ ] Anonymous user ID (localStorage UUID)

- [ ] **Worker**
  - [ ] APScheduler setup (hourly cron)
  - [ ] Vote aggregation logic (from lmarena)
  - [ ] ELO calculation (K=32, initial=1500)
  - [ ] Bradley-Terry confidence intervals
  - [ ] Update model_stats table
  - [ ] Worker status tracking

#### Acceptance Criteria
- ✅ User can complete full session (init → execute → vote)
- ✅ Videos play smoothly
- ✅ Voting works, models revealed
- ✅ Leaderboard displays correctly
- ✅ Worker runs hourly, ELO updates

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
  - [ ] Episode generation < 10s
  - [ ] API response < 500ms (p95)
  - [ ] Video encoding < 2s
  - [ ] Database query optimization

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
| **Video generation slow** | High | Use efficient encoding (ffmpeg), optimize rendering |
| **VLA inference slow** | High | Already measured (283ms), build around it |
| **MuJoCo instability** | Medium | Thorough state serialization testing |
| **Storage costs (videos)** | Medium | Compression, cleanup old sessions |

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
- Session sidebar = good UX

### Reference Projects

**lmarena-clone:**
- Battle system: `backend/src/llmbattler_backend/api/battles.py`
- ELO calculator: `worker/src/llmbattler_worker/aggregators/elo_calculator.py`
- Frontend battle UI: `frontend/app/battle/page.tsx`

**Adapting for VLA:**
- Text messages → Videos
- LLM inference → VLA + MuJoCo episode generation
- Same voting, ELO, leaderboard logic

---

## 🔗 Related Documents

- **[00_PROJECT.md](./00_PROJECT.md)** - Project overview and architecture
- **[ARCHITECTURE/ADR_001-*.md](./ARCHITECTURE/)** - Architecture decisions
- **[FEATURES/](./FEATURES/)** - Detailed feature specs
- **[../lmarena-clone](../../lmarena-clone/)** - Reference codebase

---

## 🔄 Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-01 | Initial roadmap (Phase 1/2 split) | Claude |
| 2025-01-04 | Restructured for MVP (integrated approach) | Claude |
| 2025-01-04 | Added benchmark results, architecture decisions | Claude |
| 2025-01-04 | Aligned with new VLA Arena specification | Claude |

---

**Last Updated:** 2025-01-04
**Status:** MVP Week 2 - Backend Foundation
**Next Milestone:** Database migrations + Session API
**Target MVP Completion:** Week 7
