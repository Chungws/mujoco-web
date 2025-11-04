# ADR-002: Database Schema Design for VLA Arena

**Status:** Accepted
**Date:** 2025-01-04
**Decision Makers:** Development Team
**Related:** Phase 2 (Model Comparison), ADR-001 (Server-Side Execution)

---

## Context

### Problem Statement

VLA Arena requires a database schema to support:

1. **Battle Sessions**: Users select robot+scene, system assigns 2 anonymous VLA models
2. **Multi-turn Interaction**: Users can provide multiple instructions in one battle
3. **Episode Storage**: Store execution data (actions, states) for replay
4. **Voting & ELO**: Track votes and calculate segmented ELO rankings
5. **Leaderboard**: Display rankings by robot type (e.g., "Best for Franka robot")

### Key Requirements

- **No Foreign Keys**: Application-level relationships only (lmarena-clone pattern)
- **Denormalized Vote**: Worker aggregates ELO without JOINs
- **Segmented ELO**: Robot-specific and global rankings
- **Episode Replay**: Support step-by-step replay in browser (MuJoCo WASM)
- **Large Episode Data**: ~27KB per episode (50 states × 500 bytes)

### Key Questions

1. How to handle large episode data (actions, states)?
2. Should we store video or state-based replay?
3. How to structure ELO stats (global vs. segmented)?
4. MongoDB vs. PostgreSQL JSONB for episode data?

---

## Decision

We adopt a **PostgreSQL + MongoDB Hybrid Architecture**:
- **PostgreSQL**: Metadata, votes, ELO stats (relational queries)
- **MongoDB**: Episode execution data (large JSON documents)

### Core Design Principles

#### 1. Session-Battle Separation (1:1 for now, N later)

**Session** = User's session container
- Stores: robot_id, scene_id, user_id (optional)
- Future: Can have multiple battles (different instructions)

**Battle** = One battle between two models
- Stores: left/right model IDs, seq_in_session
- Relationship: session_id (application-level FK)

**Current MVP**: 1 Session = 1 Battle
**Future**: 1 Session = N Battles (try different instructions)

---

#### 2. Multi-turn Support

**Turn** = One user instruction
- User provides: "Pick up the red cube"
- System generates: 2 Episodes (left/right models)

**Battle Structure:**
```
Battle
├── Turn 1: "Pick up cube" → Episode (left), Episode (right)
├── Turn 2: "Place on table" → Episode (left), Episode (right)
└── Vote
```

**Episode** = One model execution (up to max_steps, configurable)
- Stores in MongoDB: actions[], states[]
- Size: ~13KB per episode (max 50 steps)

---

#### 3. No Video Storage (State-based Replay)

**Decision:** Store MuJoCo states, not video

**Rationale:**
- Video: 5MB, fast preview, not interactive
- State: 27KB, step-by-step replay in browser (MuJoCo WASM)
- User can scrub through steps, rotate camera, inspect joint angles

**Replay Flow:**
```
Server: Execute episode → Save states[0..50] to MongoDB
Client: Load states → MuJoCo WASM.set_state(states[i]) → Render
```

---

#### 4. Segmented ELO (Robot-specific + Global)

**Problem:** VLA models may perform differently on different robots
- OpenVLA on Franka: 1900 ELO
- OpenVLA on UR5: 1700 ELO
- Global average: 1800 (misleading!)

**Solution:** Two stats tables

**ModelStatsByRobot** (segmented)
```
PK: (model_id, robot_id)
- openvla-7b, franka → ELO 1900
- openvla-7b, ur5 → ELO 1700
```

**ModelStatsTotal** (global)
```
PK: model_id
- openvla-7b → ELO 1880 (calculated from all votes)
```

**Calculation:** Worker filters votes by robot_id and calculates separate ELO

---

#### 5. Denormalized Vote (Worker Performance)

**Vote Table:**
```python
Vote:
- vote_id, battle_id
- robot_id, scene_id  # Denormalized from Session
- left_model_id, right_model_id  # Denormalized from Battle
- vote: "left_better" | "right_better" | "tie" | "both_bad"
- processing_status: "pending" | "processed" | "failed"
```

**Rationale:**
- Worker reads Votes table only (no JOINs with Battle or Session)
- Filters by robot_id → calculate robot-specific ELO
- lmarena-clone pattern: denormalize for aggregation performance

---

## Database Schema

### PostgreSQL Tables

#### Session
```python
sessions:
- id: int (PK, auto-increment)
- session_id: str (unique, indexed)
- robot_id: str  # "franka", "ur5", "widowx"
- scene_id: str  # "table", "kitchen", "warehouse"
- user_id: str (optional, indexed)  # Anonymous UUID from localStorage
- created_at: timestamp
- last_active_at: timestamp (indexed)
```

---

#### Battle
```python
battles:
- id: int (PK, auto-increment)
- battle_id: str (unique, indexed)
- session_id: str (indexed)  # Application-level FK
- seq_in_session: int (indexed)  # Order in session (1, 2, 3...)
- left_model_id: str  # "openvla-7b"
- right_model_id: str  # "rt-1"
- status: str  # "ongoing", "voted", "abandoned"
- created_at: timestamp (indexed)
- updated_at: timestamp
```

---

#### Turn
```python
turns:
- id: int (PK, auto-increment)
- turn_id: str (unique, indexed)
- session_id: str (indexed)
- battle_id: str (indexed)
- battle_seq_in_session: int (indexed)
- seq: int (indexed)  # Turn order in battle (1, 2, 3...)
- instruction: str  # User's natural language instruction
- created_at: timestamp (indexed)
```

---

#### Vote
```python
votes:
- id: int (PK, auto-increment)
- vote_id: str (unique, indexed)
- battle_id: str (unique, indexed)  # 1:1 relationship
- session_id: str (indexed)  # For analytics
- robot_id: str (indexed)  # Denormalized for worker
- scene_id: str (indexed)  # Denormalized for worker
- left_model_id: str  # Denormalized
- right_model_id: str  # Denormalized
- vote: str  # "left_better", "right_better", "tie", "both_bad"
- processing_status: str (indexed)  # "pending", "processed", "failed"
- processed_at: timestamp (nullable)
- error_message: str (nullable)
- voted_at: timestamp (indexed)
```

---

#### ModelStatsByRobot
```python
model_stats_by_robot:
- id: int (PK, auto-increment)
- model_id: str (indexed)
- robot_id: str (indexed)
- elo_score: int (indexed)
- elo_ci: float  # 95% confidence interval
- vote_count: int (indexed)
- win_count: int
- loss_count: int
- tie_count: int
- win_rate: float
- updated_at: timestamp

# Composite unique constraint: (model_id, robot_id)
```

---

#### ModelStatsTotal
```python
model_stats_total:
- id: int (PK, auto-increment)
- model_id: str (unique, indexed)
- elo_score: int (indexed)
- elo_ci: float
- vote_count: int (indexed)
- win_count: int
- loss_count: int
- tie_count: int
- win_rate: float
- organization: str  # "OpenAI", "Google", "Meta"
- license: str  # "proprietary", "open-source"
- updated_at: timestamp
```

---

#### WorkerStatus
```python
worker_status:
- id: int (PK, auto-increment)
- worker_name: str (unique)  # "elo_aggregator"
- last_run_at: timestamp
- status: str  # "success", "failed", "running"
- votes_processed: int
- error_message: str (nullable, max 1000 chars)
```

---

### MongoDB Collections

#### episodes
```javascript
{
  // Identifiers
  episode_id: "ep_abc123",
  session_id: "sess_def456",
  battle_id: "battle_ghi789",
  turn_id: "turn_jkl012",

  // Sequence tracking
  battle_seq_in_session: 1,
  turn_seq: 1,
  seq_in_turn: 0,  // 0=left, 1=right

  // Model info
  side: "left",  // "left" or "right"
  model_id: "openvla-7b",

  // Execution data (variable length, up to EPISODE_MAX_STEPS)
  actions: [
    [0.1, 0.2, -0.3, 0.0, 0.5, 0.1, 0.0, 1.0],  // step 0, 8-dim action
    [0.15, 0.25, -0.25, 0.05, 0.45, 0.12, 0.0, 1.0],  // step 1
    // ... variable length (early termination possible)
  ],

  states: [
    {
      qpos: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04],  // joint positions
      qvel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  // joint velocities
      time: 0.0
    },
    {
      qpos: [...],
      qvel: [...],
      time: 0.1
    },
    // ... variable length
  ],

  // Metrics (small summary)
  metrics: {
    success: true,
    total_steps: 35,  // Actual steps executed
    max_steps: 50,  // Configured maximum
    terminated_early: false,
    final_distance_to_goal: 0.05,
    collision_count: 0,
    gripper_opened_at_step: 25
  },

  // Metadata
  duration_ms: 5120,  // Execution time
  created_at: ISODate("2025-01-04T12:34:56.789Z")
}
```

**Size estimate (max episode: 50 steps):**
- qpos: 9 floats × 8 bytes = 72 bytes
- qvel: 9 floats × 8 bytes = 72 bytes
- time: 8 bytes
- **state total**: ~152 bytes (with overhead → ~200 bytes)
- **50 states (max)**: 10 KB
- **50 actions (max)** (8-dim): 1.6 KB
- **metrics**: < 1 KB
- **Total**: ~13 KB per episode (maximum, actual size varies)
- **Example**: 20-step episode = ~5 KB

**Indexes:**
```javascript
db.episodes.createIndex({ episode_id: 1 }, { unique: true })
db.episodes.createIndex({ battle_id: 1, side: 1 })
db.episodes.createIndex({ turn_id: 1 })
db.episodes.createIndex({ session_id: 1 })
db.episodes.createIndex({ model_id: 1, created_at: -1 })
```

---

### YAML Configuration

**config/models.yaml** (not in database for MVP)
```yaml
models:
  - id: openvla-7b
    name: OpenVLA 7B
    version: "1.0"
    organization: OpenAI
    license: open-source
    compatible_robots:
      - franka
      - ur5
    action_dim: 7
    inference_latency_ms: 200  # For simulation
    config:
      checkpoint: ./models/openvla-7b.pt

  - id: rt-1
    name: RT-1
    version: "1.0"
    organization: Google
    license: proprietary
    compatible_robots:
      - franka
      - widowx
    action_dim: 7
    inference_latency_ms: 100
```

**Future:** Move to database with Admin UI for dynamic management

---

## Consequences

### Positive

✅ **Clear Separation**
- PostgreSQL: Fast relational queries (votes, stats, sessions)
- MongoDB: Optimized for large JSON documents (episodes)

✅ **Efficient ELO Calculation**
- Denormalized Vote table → Worker reads 1 table
- Robot-specific ELO → Accurate rankings per robot type

✅ **Interactive Replay**
- State-based replay (27KB) vs. Video (5MB)
- Users can scrub, rotate, inspect in browser (MuJoCo WASM)

✅ **No Foreign Keys**
- Application-level relationships
- Horizontal scaling friendly
- lmarena-clone pattern consistency

✅ **Future-proof**
- Session can have multiple Battles (instruction variations)
- Battle can have multiple Turns (multi-step tasks)
- Easy to add scene-specific ELO later

### Negative

⚠️ **Two Database Systems**
- Increased operational complexity (PostgreSQL + MongoDB)
- Need to manage two backup strategies
- Episode queries require MongoDB client

⚠️ **Large Episode Storage**
- Average ~8KB per episode (assuming mixed task complexity)
- 8KB × 1M episodes = 8 GB (manageable)
- Need MongoDB scaling plan for 10M+ episodes

⚠️ **No Video Preview**
- Users must wait for WASM to load (~10MB, first time only)
- Mobile may be slower than video playback
- Mitigated: Can add low-res video thumbnails later

### Mitigation Strategies

**For operational complexity:**
- Docker Compose handles both databases in dev/prod
- Use MongoDB Atlas for managed solution in production
- Single backup script for both databases

**For storage scaling:**
- MongoDB sharding when needed (10M+ episodes)
- Archive old episodes to cold storage (S3 Glacier)
- TTL index: auto-delete episodes older than 1 year

**For mobile performance:**
- Lazy-load WASM (only when user clicks "Replay")
- Cache WASM in browser (loads once per device)
- Consider adding video thumbnails in Phase 3

---

## Alternatives Considered

### Alternative 1: PostgreSQL JSONB Only (No MongoDB)

**Approach:**
```python
episodes (PostgreSQL):
- episode_id
- execution_data: JSONB  # 13KB JSON in PostgreSQL
```

**Rejected because:**
- PostgreSQL JSONB not optimized for 13KB+ documents
- Slower than MongoDB for document retrieval
- Inflates PostgreSQL database size
- Backup/restore slower
- No horizontal scaling for episode data

---

### Alternative 2: MinIO S3 for Episodes (Previous Design)

**Approach:**
```python
episodes (PostgreSQL):
- episode_id
- execution_data_url: str  # s3://vlaarena/ep_123.json
```

**Rejected because:**
- S3 optimized for static files, not structured data
- Cannot query episode data (e.g., "find episodes with collision")
- No indexing on model_id or turn_id
- Extra network hop for episode retrieval
- MongoDB better for JSON document storage

---

### Alternative 3: Video Only (No States)

**Approach:**
- Store video files (5MB each)
- No state-based replay

**Rejected because:**
- Larger storage (5MB vs 13KB = 385× larger)
- Not interactive (can't scrub to specific step, rotate camera)
- Not debuggable (can't inspect joint angles)
- User research shows "step-by-step analysis" is valuable

---

### Alternative 4: Global ELO Only (No Segmentation)

**Approach:**
```python
ModelStatsTotal only:
- model_id
- elo_score  # Average across all robots
```

**Rejected because:**
- VLA models have robot-specific performance
- Users want "Best for Franka" leaderboard
- Misleading: OpenVLA may excel on Franka but underperform on UR5
- Segmented ELO provides more useful information

---

## Implementation Notes

### Phase 2 Scope

**Implement:**
- PostgreSQL models (Session, Battle, Turn, Vote, ModelStats)
- MongoDB episodes collection
- Alembic migrations for PostgreSQL
- Worker: ELO aggregation (robot-specific + global)
- No video storage

**Defer to Phase 3:**
- Scene-specific ELO (if needed)
- Model database table + Admin UI
- Video thumbnails for mobile
- Episode analytics dashboard

---

### Migration Strategy

**From lmarena-clone:**
1. Keep Session, Battle, Vote, WorkerStatus (identical structure)
2. Replace Message table → Episode collection (MongoDB)
3. Add robot_id, scene_id to Session, Vote
4. Split ModelStats → ModelStatsByRobot + ModelStatsTotal
5. Add Turn table (lmarena has it, keep it)

**Database Setup:**
```bash
# PostgreSQL
docker compose up -d postgres
cd backend && alembic upgrade head

# MongoDB
docker compose up -d mongodb
# No migrations needed (schema-less)
```

---

### Query Patterns

**1. Get Battle with Episodes**
```python
# PostgreSQL
battle = session.get(Battle, battle_id)

# MongoDB
episodes = db.episodes.find({"battle_id": battle_id}).sort("side", 1)
# Result: [left_episode, right_episode]
```

**2. Worker ELO Calculation**
```python
# Get pending votes (no JOIN needed)
votes = session.exec(
    select(Vote)
    .where(Vote.processing_status == "pending")
    .where(Vote.robot_id == "franka")
).all()

# Calculate ELO for each model
for vote in votes:
    update_elo(vote.left_model_id, vote.right_model_id, vote.vote)
```

**3. Leaderboard (Franka)**
```python
stats = session.exec(
    select(ModelStatsByRobot)
    .where(ModelStatsByRobot.robot_id == "franka")
    .order_by(ModelStatsByRobot.elo_score.desc())
).all()
```

---

## Related Decisions

- **ADR-001**: Server-Side Execution (why episodes are generated on server)
- **ADR-003** (future): Model Configuration Management (YAML vs DB)
- **ADR-004** (future): Episode Archival Strategy

---

## References

- [lmarena-clone models.py](../../lmarena-clone/shared/src/llmbattler_shared/models.py)
- [lmarena-clone ADR-001: No Foreign Keys](../../lmarena-clone/WORKSPACE/ARCHITECTURE/ADR_001-No_Foreign_Keys.md)
- [FEATURES/002_MODEL_COMPARISON.md](../FEATURES/002_MODEL_COMPARISON.md)
- [MongoDB Best Practices](https://www.mongodb.com/docs/manual/core/data-modeling-introduction/)

---

**Last Updated:** 2025-01-04
**Status:** Accepted and ready for implementation
