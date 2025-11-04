---
**⚠️ ARCHIVED SPECIFICATION**

This document represents the original Phase 2 (Battle system) approach and has been **superseded by the integrated MVP specification** (2025-01-04).

**Why Archived:**
- Battle system now part of MVP (not separate phase)
- Database schema refined based on lmarena-clone
- ELO system directly adopted from reference project
- All server-side execution confirmed by benchmarks

**Current Specification:** See [../ROADMAP.md](../ROADMAP.md) and [../00_PROJECT.md](../00_PROJECT.md)

**Useful Content:**
- Battle flow concepts (now called "sessions" and "episodes")
- ELO calculation details remain accurate
- Leaderboard requirements still valid

**Date Archived:** 2025-01-04
---

# Feature: VLA Model Comparison & Leaderboard - Phase 2

**Status:** ~~Not Started~~ ARCHIVED
**Priority:** Medium (Post-MVP)
**Estimated Time:** 4-6 weeks
**Dependencies:** Phase 1 (VLA Visualization)

---

## Overview

Build a battle arena system for comparing Vision-Language-Action (VLA) models through blind A/B testing, similar to [lmarena-clone](https://github.com/Chungws/lmarena-clone). Users can select a task and instruction, watch two VLA models execute policies side-by-side, vote on the better performance, and contribute to comprehensive ELO-based leaderboards.

**Goals:**
- Blind side-by-side VLA model comparison
- Task-based evaluation with natural language instructions
- User voting on model performance
- ELO-based ranking system with confidence intervals
- Comprehensive leaderboard with filtering and sorting
- Session management (multiple battles per session)

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│          Next.js Frontend (Client)              │
│                                                   │
│  ┌─────────────────────────────────────────┐   │
│  │  Battle UI                               │   │
│  │  - Side-by-side MuJoCo viewers           │   │
│  │  │  Model A (Left)  │  Model B (Right)  │   │
│  │  - Voting interface (after execution)    │   │
│  │  - Model reveal after vote               │   │
│  └──────────────┬──────────────────────────┘   │
│                 │                                │
│  ┌──────────────▼──────────────────────────┐   │
│  │  Session Sidebar (ChatGPT-like)         │   │
│  │  - Battle history                        │   │
│  │  - Create new battle                     │   │
│  └──────────────┬──────────────────────────┘   │
│                 │                                │
│  ┌──────────────▼──────────────────────────┐   │
│  │  Leaderboard Page                        │   │
│  │  - ELO rankings with confidence intervals│   │
│  │  - Sorting and filtering                 │   │
│  └──────────────────────────────────────────┘   │
└─────────────────┬───────────────────────────────┘
                  │ HTTP/REST
    ┌─────────────▼─────────────┐
    │   FastAPI Backend (API)    │
    │                             │
    │  ┌─────────────────────┐   │
    │  │  API Routes          │   │
    │  │  - /sessions         │   │
    │  │  - /battles          │   │
    │  │  - /votes            │   │
    │  │  - /models           │   │
    │  │  - /leaderboard      │   │
    │  └──────────┬───────────┘   │
    │             │                │
    │  ┌──────────▼───────────┐   │
    │  │  Services             │   │
    │  │  - VLA execution      │   │
    │  │  - Model selection    │   │
    │  │  - Vote processing    │   │
    │  └──────────┬───────────┘   │
    │             │                │
    │  ┌──────────▼───────────┐   │
    │  │  Repositories         │   │
    │  │  - SessionRepository  │   │
    │  │  - BattleRepository   │   │
    │  │  - VoteRepository     │   │
    │  └──────────┬───────────┘   │
    └─────────────┼───────────────┘
                  │
    ┌─────────────▼─────────────┐
    │   PostgreSQL Database      │
    │                             │
    │  Tables:                    │
    │  - sessions                 │
    │  - battles                  │
    │  - votes                    │
    │  - models                   │
    │  - model_stats              │
    │  - worker_status            │
    │                             │
    │  (No FK constraints)        │
    └─────────────┬───────────────┘
                  │
    ┌─────────────▼─────────────┐
    │   Worker (Python)          │
    │                             │
    │  ┌─────────────────────┐   │
    │  │  APScheduler         │   │
    │  │  (Hourly cron job)   │   │
    │  └──────────┬───────────┘   │
    │             │                │
    │  ┌──────────▼───────────┐   │
    │  │  Vote Aggregation     │   │
    │  │  - ELO calculation    │   │
    │  │  - Bradley-Terry CI   │   │
    │  │  - Update model_stats │   │
    │  └──────────────────────┘   │
    └───────────────────────────────┘
```

---

## Design Decisions

### 1. Session-Based Architecture
**Decision:** Battle arena with session management (following lmarena-clone pattern)

**Hierarchy:**
```
User Session
  └── Battle 1 (Task: Pick and Place)
      ├── Model A execution (hidden)
      ├── Model B execution (hidden)
      └── Vote → Model reveal
  └── Battle 2 (Task: Open Drawer)
      ├── Model C execution (hidden)
      ├── Model D execution (hidden)
      └── Vote → Model reveal
  └── ... (multiple battles per session)
```

**Key Features:**
- Anonymous user IDs (localStorage UUID)
- Session title = first task/instruction
- `last_active_at` timestamp for session sorting
- Battle history in sidebar (ChatGPT-like)

### 2. No Foreign Keys (ADR-001)
**Decision:** Application-level relationships only

**Rationale:**
- Simplifies testing (no fixture dependency order)
- Easier development (no cascade deletion complexity)
- Same pattern as lmarena-clone

**Example:**
```python
# SQLModel (no FK)
class Battle(SQLModel, table=True):
    id: int
    session_id: int  # No FK constraint
    model_a_id: str
    model_b_id: str
    # ...

# Application-level relationship
async def get_session_battles(session_id: int):
    battles = await battle_repo.find_by_session_id(session_id)
    return battles
```

### 3. VLA Execution Strategy
**Decision:** Server-side execution (Phase 2)

**Options Considered:**
- **Option A:** Client-side (browser ONNX)
  - ❌ Inconsistent performance across devices
  - ❌ Limited model size
  - ❌ Hard to ensure fair comparison
- **Option B:** Server-side execution ✅
  - ✅ Consistent environment
  - ✅ Supports larger models
  - ✅ Fair comparison
  - ✅ Can use GPU acceleration

**Implementation:**
- Backend loads VLA models on startup
- Async execution with timeout (30-60 seconds)
- Record full episode logs (JSONB)
- Return observation videos + action sequences

### 4. Model Selection Algorithm
**Decision:** Uniform random selection (same as lmarena-clone)

**Algorithm:**
```python
def select_models_for_battle():
    active_models = get_active_models()
    model_a, model_b = random.sample(active_models, 2)

    # Random left/right position to prevent bias
    if random.random() < 0.5:
        left_model, right_model = model_a, model_b
    else:
        left_model, right_model = model_b, model_a

    return left_model, right_model
```

**Future Enhancement:**
- Weighted selection based on vote count (prioritize under-tested models)

### 5. Vote Options
**Decision:** 4-option voting (same as lmarena-clone)

**Vote Types:**
- `left_better`: Left model performed better (1.0, 0.0)
- `right_better`: Right model performed better (0.0, 1.0)
- `tie`: Both models performed equally well (0.5, 0.5)
- `both_bad`: Both models failed the task (0.25, 0.25)

**Scoring:**
- ELO calculation based on pairwise comparison
- Bradley-Terry model for confidence intervals

### 6. Task Configuration
**Decision:** Pre-defined tasks with configurable parameters

**Task Schema:**
```yaml
tasks:
  - id: pick-and-place
    name: Pick and Place
    description: Pick up an object and place it at a target location
    environment: franka-kitchen
    instruction_template: "Pick up the {object} and place it on the {target}"
    objects: [red cube, blue cylinder, green sphere]
    targets: [left plate, right plate, shelf]
    success_criteria:
      - object_near_target: 0.05  # meters
      - object_stable: true
    max_steps: 200
    timeout: 30  # seconds

  - id: drawer-opening
    name: Drawer Opening
    description: Open a drawer and retrieve an object
    environment: franka-cabinet
    instruction_template: "Open the {drawer} and take out the object inside"
    drawers: [top drawer, middle drawer, bottom drawer]
    success_criteria:
      - drawer_open_angle: 0.5  # radians
      - object_grasped: true
    max_steps: 300
    timeout: 45
```

### 7. Episode Recording
**Decision:** Record full episodes in JSONB

**Episode Data:**
```json
{
  "task_id": "pick-and-place",
  "instruction": "Pick up the red cube and place it on the left plate",
  "model_id": "openvla-v1",
  "steps": [
    {
      "step": 0,
      "observation": {
        "rgb": "base64_encoded_image",
        "proprioception": [0.0, 0.5, 1.2, ...]
      },
      "action": [0.1, 0.2, -0.3, ...],
      "reward": 0.0,
      "timestamp": 1234567890.123
    },
    // ... more steps
  ],
  "success": true,
  "total_reward": 10.5,
  "execution_time": 12.3,
  "metadata": {
    "max_steps": 200,
    "timeout": 30,
    "termination_reason": "success"
  }
}
```

### 8. ELO Calculation
**Decision:** Same as lmarena-clone (Bradley-Terry + Confidence Intervals)

**Algorithm:**
- Initial ELO: 1500
- K-factor: 32
- Bradley-Terry model for pairwise comparison
- 95% confidence intervals using normal approximation
- Minimum 5 votes to appear on leaderboard

**Worker Schedule:**
- Hourly ELO recalculation
- Update `model_stats` table
- Track vote counts and win rates

---

## Phase Breakdown

### Phase 2.1: Backend Infrastructure (Week 1-2)
**Goal:** Set up backend API and database

**Tasks:**
- [ ] Create FastAPI project structure (following lmarena-clone)
- [ ] Set up uv workspace (backend, worker, shared)
- [ ] Configure PostgreSQL database
- [ ] Define SQLModel models
  - [ ] Session
  - [ ] Battle
  - [ ] Vote
  - [ ] Model
  - [ ] ModelStats
  - [ ] WorkerStatus
- [ ] Create Alembic migrations
- [ ] Set up Docker Compose
- [ ] Configure environment variables (.env)

**Acceptance Criteria:**
- ✅ Backend skeleton created
- ✅ Database schema designed (no FKs)
- ✅ Migrations work
- ✅ Docker Compose starts PostgreSQL
- ✅ uv workspace configured

**Files to Create:**
```
backend/
  src/llmbattler_backend/
    main.py
    config.py
    api/
      __init__.py
    services/
      __init__.py
    repositories/
      __init__.py
  tests/
    conftest.py
  pyproject.toml
  alembic/
    versions/

worker/
  src/llmbattler_worker/
    main.py
  pyproject.toml

shared/
  src/llmbattler_shared/
    models.py
    schemas.py
    config.py
  pyproject.toml

docker-compose.yml
.env.example
```

---

### Phase 2.2: Battle System API (Week 2-3)
**Goal:** Implement battle creation and voting APIs

**Tasks:**
- [ ] Task configuration system (YAML)
- [ ] Model management API
  - [ ] List active models
  - [ ] Get model details
- [ ] Session API
  - [ ] Create session
  - [ ] Get session details
  - [ ] List user sessions
- [ ] Battle API
  - [ ] Create battle (with model selection)
  - [ ] Execute VLA policies (parallel)
  - [ ] Get battle details
  - [ ] List session battles
- [ ] Voting API
  - [ ] Submit vote
  - [ ] Reveal models after vote
- [ ] VLA execution service
  - [ ] Load models on startup
  - [ ] Execute policy with timeout
  - [ ] Record episode logs
  - [ ] Generate observation videos (optional)

**Acceptance Criteria:**
- ✅ Task configuration loads from YAML
- ✅ Models can be listed
- ✅ Sessions can be created
- ✅ Battles execute VLA policies correctly
- ✅ Voting works and reveals models
- ✅ Episodes recorded in JSONB

**API Endpoints:**
```
GET  /api/models                    # List models
GET  /api/models/{model_id}         # Get model details
POST /api/sessions                  # Create session
GET  /api/sessions/{session_id}     # Get session
GET  /api/sessions/user/{user_id}   # List user sessions
POST /api/battles                   # Create battle (executes VLA)
GET  /api/battles/{battle_id}       # Get battle
GET  /api/battles/session/{sid}     # List session battles
POST /api/votes                     # Submit vote
GET  /api/tasks                     # List available tasks
```

---

### Phase 2.3: Leaderboard System (Week 3-4)
**Goal:** Implement ELO calculation and leaderboard

**Tasks:**
- [ ] Worker service setup (APScheduler)
- [ ] Vote aggregation logic
- [ ] ELO calculation algorithm
- [ ] Bradley-Terry confidence intervals
- [ ] Update model_stats table
- [ ] Leaderboard API
  - [ ] Get rankings
  - [ ] Filter by model type/organization
  - [ ] Sort by ELO, votes, win rate
  - [ ] Search by model name
- [ ] Worker status tracking
- [ ] Manual aggregation endpoint (for testing)

**Acceptance Criteria:**
- ✅ Worker runs hourly aggregation
- ✅ ELO scores calculated correctly
- ✅ Confidence intervals computed
- ✅ Leaderboard API returns rankings
- ✅ Filtering and sorting work
- ✅ Minimum vote threshold enforced

**API Endpoints:**
```
GET /api/leaderboard              # Get rankings
GET /api/leaderboard/stats        # Overall statistics
GET /api/worker/status            # Worker status
POST /api/worker/aggregate        # Manual aggregation (admin)
```

**Worker Logic:**
```python
# worker/src/llmbattler_worker/main.py
from apscheduler.schedulers.blocking import BlockingScheduler

def aggregate_votes():
    # 1. Get all new votes since last aggregation
    # 2. Calculate pairwise ELO updates
    # 3. Compute confidence intervals
    # 4. Update model_stats table
    # 5. Update worker_status
    pass

scheduler = BlockingScheduler()
scheduler.add_job(aggregate_votes, 'cron', hour='*')  # Hourly
scheduler.start()
```

---

### Phase 2.4: Frontend Integration (Week 4-5)
**Goal:** Build battle UI and leaderboard page

**Tasks:**
- [ ] Battle page UI
  - [ ] Side-by-side MuJoCo viewers
  - [ ] Task selection
  - [ ] Instruction input
  - [ ] Execution loading state
  - [ ] Voting interface
  - [ ] Model reveal after vote
- [ ] Session sidebar (ChatGPT-like)
  - [ ] Battle history
  - [ ] Create new battle button
  - [ ] Session title display
- [ ] Leaderboard page
  - [ ] ELO rankings table
  - [ ] Confidence intervals display
  - [ ] Filtering by organization
  - [ ] Sorting controls
  - [ ] Search functionality
- [ ] API client integration
- [ ] Anonymous user management (localStorage UUID)
- [ ] Error handling and loading states

**Acceptance Criteria:**
- ✅ Battle page works end-to-end
- ✅ Side-by-side execution visible
- ✅ Voting interface intuitive
- ✅ Session sidebar functional
- ✅ Leaderboard displays correctly
- ✅ Responsive design

**Pages:**
```
app/
  battle/
    page.tsx              # Battle page
  leaderboard/
    page.tsx              # Leaderboard page

components/
  battle/
    battle-card.tsx       # Side-by-side viewers
    voting-panel.tsx      # Vote buttons
    model-reveal.tsx      # After vote
  sidebar/
    session-list.tsx      # Session history
  leaderboard/
    leaderboard-table.tsx # Rankings table

lib/
  apiClient.ts            # API client
  storage.ts              # localStorage utilities
```

---

### Phase 2.5: Testing & Deployment (Week 5-6)
**Goal:** Test and deploy the system

**Tasks:**
- [ ] Backend testing
  - [ ] Unit tests (pytest)
  - [ ] API integration tests
  - [ ] ELO calculation tests
  - [ ] Vote aggregation tests
- [ ] Frontend testing
  - [ ] Component tests
  - [ ] Playwright MCP verification
  - [ ] End-to-end tests
- [ ] Performance testing
  - [ ] VLA execution time
  - [ ] API response time
  - [ ] Database query optimization
- [ ] Production deployment
  - [ ] Docker Compose production profile
  - [ ] Environment configuration
  - [ ] Monitoring setup
- [ ] Documentation
  - [ ] API documentation (FastAPI auto-docs)
  - [ ] User guide
  - [ ] Deployment guide

**Acceptance Criteria:**
- ✅ All tests pass
- ✅ Performance meets targets
- ✅ Production deployment successful
- ✅ Documentation complete

---

## Database Schema

### Tables (No Foreign Keys)

**sessions:**
```sql
CREATE TABLE sessions (
    id SERIAL PRIMARY KEY,
    user_id TEXT NOT NULL,
    title TEXT,
    last_active_at TIMESTAMP,
    created_at TIMESTAMP DEFAULT NOW()
);
```

**battles:**
```sql
CREATE TABLE battles (
    id SERIAL PRIMARY KEY,
    session_id INT NOT NULL,
    task_id TEXT NOT NULL,
    instruction TEXT NOT NULL,
    model_a_id TEXT NOT NULL,
    model_b_id TEXT NOT NULL,
    left_model_id TEXT NOT NULL,
    right_model_id TEXT NOT NULL,
    episode_a JSONB,
    episode_b JSONB,
    status TEXT DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT NOW()
);
```

**votes:**
```sql
CREATE TABLE votes (
    id SERIAL PRIMARY KEY,
    battle_id INT NOT NULL,
    winner TEXT NOT NULL,  -- 'left_better', 'right_better', 'tie', 'both_bad'
    created_at TIMESTAMP DEFAULT NOW()
);
```

**models:**
```sql
CREATE TABLE models (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    description TEXT,
    organization TEXT,
    license TEXT,
    model_path TEXT,
    status TEXT DEFAULT 'active',
    created_at TIMESTAMP DEFAULT NOW()
);
```

**model_stats:**
```sql
CREATE TABLE model_stats (
    id SERIAL PRIMARY KEY,
    model_id TEXT NOT NULL,
    elo_score FLOAT DEFAULT 1500,
    elo_ci_lower FLOAT,
    elo_ci_upper FLOAT,
    vote_count INT DEFAULT 0,
    win_count INT DEFAULT 0,
    loss_count INT DEFAULT 0,
    tie_count INT DEFAULT 0,
    updated_at TIMESTAMP DEFAULT NOW()
);
```

**worker_status:**
```sql
CREATE TABLE worker_status (
    id SERIAL PRIMARY KEY,
    last_run TIMESTAMP,
    status TEXT,
    votes_processed INT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

---

## Success Metrics

**Phase 2 MVP Success:**
1. ✅ Battle system works end-to-end
2. ✅ ELO rankings calculated correctly
3. ✅ Leaderboard displays with CI
4. ✅ At least 3 VLA models integrated
5. ✅ Session management functional
6. ✅ No critical bugs

**Performance Targets:**
- VLA execution: < 60 seconds per episode
- API response time: < 500ms (95th percentile)
- Leaderboard load time: < 1 second
- Database queries: < 100ms

**User Experience Goals:**
- Intuitive battle interface
- Clear visualization of VLA execution
- Fair and transparent ranking system

---

## Future Enhancements (Post-Phase 2)

### Advanced Features
- **Task Builder**: Create custom tasks with web UI
- **Multi-step Tasks**: Complex task sequences
- **Real Robot Integration**: Real-world robot execution
- **Video Playback**: Slow-motion replay of episodes
- **Heatmaps**: Attention visualization for VLA models
- **Comments**: User comments on battles

### Community Features
- **User Authentication**: Replace anonymous UUIDs
- **User Profiles**: Track user voting history
- **Public Battles**: Share battle links
- **Discussion**: Comments and discussions

### Performance & Scale
- **Caching**: Redis for leaderboard caching
- **GPU Acceleration**: Faster VLA execution
- **Distributed Execution**: Multiple worker nodes
- **Read Replicas**: Scale leaderboard queries

---

## References

### lmarena-clone (Primary Reference)
- Battle system architecture
- ELO calculation algorithm
- Session management
- Worker implementation
- Database schema (no FKs)

**Key Files to Study:**
```
lmarena-clone/
  backend/src/llmbattler_backend/
    api/battles.py          # Battle API
    api/sessions.py         # Session API
    api/leaderboard.py      # Leaderboard API
    services/battle_service.py
    repositories/
  worker/src/llmbattler_worker/
    aggregators/elo_calculator.py
  shared/src/llmbattler_shared/
    models.py               # SQLModel schemas
```

### VLA Resources
- [OpenVLA](https://openvla.github.io/)
- [RT-2 Paper](https://arxiv.org/abs/2307.15818)
- [Octo Model](https://octo-models.github.io/)

---

## Notes

### Open Questions
1. **VLA Model Access**: Which VLA models to use?
   - Option A: Public models (OpenVLA, RT-1 reproductions)
   - Option B: Fine-tuned models
   - **Decision TBD**

2. **Episode Visualization**: How to display episodes?
   - Option A: Animated GIF
   - Option B: Video (MP4)
   - Option C: Step-by-step images
   - **Decision**: Start with step-by-step images

3. **Success Criteria**: How to determine success?
   - Option A: Manual labeling
   - Option B: Automatic detection (physics-based)
   - **Decision**: Start with automatic (physics-based)

### Risk Mitigation
- **VLA Execution Time**: Use timeout (60s), optimize models
- **Database Load**: Index properly, use connection pooling
- **Model Availability**: Start with 2-3 models, expand gradually

---

**Created:** 2025-11-01
**Last Updated:** 2025-11-01
**Status:** Ready for Implementation (after Phase 1)
