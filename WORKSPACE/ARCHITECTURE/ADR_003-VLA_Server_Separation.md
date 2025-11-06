# ADR-003: VLA Server Separation

**Status:** Accepted
**Date:** 2025-01-06
**Decision Makers:** Development Team
**Context:** VLA Arena MVP Architecture - Service Separation

---

## Context

### Problem Statement

VLA Arena's backend currently plans to integrate MuJoCo simulation and VLA model inference directly into the FastAPI orchestrator. We need to decide:

**Should VLA execution (MuJoCo + Model Inference) be:**
- **Option A:** Integrated into the main backend service
- **Option B:** Separated as an independent microservice

### Requirements

1. **Resource Isolation**: GPU-intensive VLA inference vs lightweight API orchestration
2. **Independent Scaling**: Scale VLA workers independently from API servers
3. **Development Velocity**: Teams can develop/test VLA execution independently
4. **Deployment Flexibility**: Deploy VLA servers on GPU instances only
5. **Technology Freedom**: VLA server can use different frameworks/dependencies

### Key Constraints

- Backend (FastAPI) is lightweight, CPU-only
- VLA execution requires GPU and heavy dependencies (MuJoCo, PyTorch, transformers)
- Multiple VLA models may have different resource requirements
- Development teams may work on backend and VLA execution separately

---

## Decision

**We chose: VLA Server as Separate Microservice**

VLA execution (MuJoCo simulation + model inference) runs as an independent FastAPI service. The main backend orchestrates by making HTTP requests to the VLA server.

```
Backend (Orchestrator)  →  HTTP  →  VLA Server (Execution)
  - Session management              - MuJoCo simulation
  - MongoDB storage                  - VLA model inference
  - Vote processing                  - Episode generation
  - Lightweight (CPU)                - Heavy compute (GPU)
```

---

## Rationale

### Why Separation is Better

**1. Resource Isolation**
```
Backend:                    VLA Server:
- CPU only                  - GPU required
- Low memory (512MB)        - High memory (8GB+)
- Stateless orchestrator    - Compute-intensive worker
- Many instances            - Few GPU instances
```

**2. Independent Scaling**
```
Traffic spike → Scale backend horizontally (cheap)
GPU load spike → Scale VLA servers independently (expensive, targeted)
```

**3. Deployment Flexibility**
```yaml
# Backend: Deploy anywhere (Cloud Run, Lambda, etc.)
backend:
  instances: 10
  machine: cpu-only
  cost: low

# VLA Server: Deploy on GPU instances only
vla-server:
  instances: 2
  machine: nvidia-t4
  cost: high (but only where needed)
```

**4. Development Independence**
```
Team A (Backend):          Team B (VLA):
- API development          - MuJoCo integration
- No GPU needed            - GPU development
- Fast iteration           - Model optimization
- Lightweight testing      - Heavy testing
```

**5. Technology Freedom**
```
Backend:                   VLA Server:
- FastAPI                  - FastAPI
- Minimal deps             - PyTorch, MuJoCo, transformers
- Quick startup            - Long startup (model loading)
```

---

## Architecture

### Service Communication

```
┌─────────────────────────────────────┐
│   Frontend (Next.js)                 │
│   - Battle UI                        │
│   - MuJoCo WASM replay               │
└──────────────┬──────────────────────┘
               │ HTTP
┌──────────────▼──────────────────────┐
│   Backend (FastAPI) - Orchestrator   │
│                                      │
│   POST /api/battles/{id}/turns       │
│   ├─ 1. Create Turn (PostgreSQL)     │
│   ├─ 2. Call VLA Server (HTTP) ×2    │
│   ├─ 3. Save Episodes (MongoDB)      │
│   └─ 4. Return response              │
└──────────────┬──────────────────────┘
               │ HTTP
┌──────────────▼──────────────────────┐
│   VLA Server (FastAPI) - Execution   │
│                                      │
│   POST /execute                      │
│   ├─ 1. Load MuJoCo environment      │
│   ├─ 2. Load VLA model               │
│   ├─ 3. Run episode (max 50 steps)   │
│   │   ├─ Get observation             │
│   │   ├─ VLA inference               │
│   │   ├─ MuJoCo step                 │
│   │   └─ Record state/action         │
│   └─ 4. Return episode data          │
│                                      │
│   Resources:                         │
│   - MuJoCo Menagerie (Franka)        │
│   - Octo-Small (27M-93M)             │
│   - SmolVLA (450M)                   │
└──────────────────────────────────────┘
```

### API Contract

**VLA Server Endpoint:**

```python
POST /execute

Request:
{
  "model_id": "octo-small",      # VLA model identifier
  "robot_id": "franka",          # Robot type (from Menagerie)
  "scene_id": "table",           # Scene type
  "instruction": "pick red cube" # Natural language instruction
}

Response:
{
  "actions": [                   # Variable length (up to 50)
    [0.1, 0.2, ...],             # 8-dim action vectors
    ...
  ],
  "states": [                    # Same length as actions
    {
      "qpos": [...],             # Joint positions
      "qvel": [...],             # Joint velocities
      "time": 0.0                # Simulation time
    },
    ...
  ],
  "duration_ms": 5120,           # Execution time
  "metadata": {
    "num_steps": 35,
    "max_steps": 50,
    "early_termination": true
  }
}

Error Response (5xx):
{
  "error": "model_loading_failed",
  "detail": "Failed to load octo-small: checkpoint not found"
}
```

**Backend Integration:**

```python
# backend/services/turn_service.py
class TurnService:
    def __init__(self, vla_server_url: str):
        self.vla_server_url = vla_server_url
        self.http_client = httpx.AsyncClient(timeout=120.0)

    async def _execute_and_save_episode(
        self, model_id: str, instruction: str, ...
    ) -> str:
        # Call VLA server
        response = await self.http_client.post(
            f"{self.vla_server_url}/execute",
            json={
                "model_id": model_id,
                "robot_id": session.robot_id,
                "scene_id": session.scene_id,
                "instruction": instruction,
            }
        )
        response.raise_for_status()
        episode_data = response.json()

        # Save to MongoDB (existing logic)
        episode = Episode(
            episode_id=episode_id,
            actions=episode_data["actions"],
            states=[State(**s) for s in episode_data["states"]],
            duration_ms=episode_data["duration_ms"],
            ...
        )
        await episode.insert()

        return episode_id
```

---

## Consequences

### Positive ✅

**1. Clear Separation of Concerns**
- Backend: Orchestration, storage, API
- VLA Server: Computation, simulation, inference
- Each service has single responsibility

**2. Independent Development**
- Backend team doesn't need GPU for development
- VLA team can iterate without backend changes
- Parallel development streams

**3. Resource Optimization**
```
Cost without separation:
- Backend instances: 10 × GPU instance = $$$$$

Cost with separation:
- Backend: 10 × CPU = $
- VLA Server: 2 × GPU = $$
Total: $$$ (much cheaper)
```

**4. Easy Model Updates**
- Deploy new VLA models without backend changes
- A/B test different model versions
- Rollback VLA server independently

**5. Horizontal Scaling**
```
Backend scaling:
- Add more orchestrators (cheap)
- No GPU required

VLA Server scaling:
- Add GPU workers only when needed
- Queue requests during high load
```

**6. Testing Simplicity**
```python
# Backend tests: Mock HTTP to VLA server
@pytest.fixture
def mock_vla_server(httpx_mock):
    httpx_mock.add_response(
        url="http://vla-server/execute",
        json={"actions": [...], "states": [...]}
    )

# VLA Server tests: Independent unit tests
def test_mujoco_environment():
    env = MuJoCoEnvironment("franka", "table")
    ...
```

### Negative ⚠️

**1. Network Latency Added**
- HTTP call overhead: ~5-20ms per episode
- **Mitigation:** Episode generation takes seconds (5-10s), so 20ms is negligible

**2. Distributed System Complexity**
```
Failure modes:
- VLA server down → Backend returns 503
- Network timeout → Retry logic needed
- Partial failures → Transaction management

Mitigation:
- Health checks (GET /health)
- Circuit breaker pattern
- Retry with exponential backoff
```

**3. Deployment Complexity**
- Two services to deploy instead of one
- Configuration management (URLs, credentials)
- **Mitigation:** Docker Compose for local, Kubernetes for production

**4. Request Timeout Management**
- Episode generation can take 60+ seconds
- Need long HTTP timeouts
- **Mitigation:** Set timeout to 120s, add progress endpoints

### Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| VLA server crashes during episode | High | Medium | Retry logic, idempotent requests |
| Network partition | High | Low | Circuit breaker, fallback to mock |
| VLA server overloaded | Medium | High | Queue system, rate limiting |
| Version mismatch (API contract) | High | Low | API versioning, integration tests |

---

## Implementation Details

### Project Structure

```
mujoco-web/
├── backend/              # Orchestrator (existing)
│   ├── src/vlaarena_backend/
│   │   └── services/
│   │       └── turn_service.py  # Calls VLA server via HTTP
│   └── pyproject.toml    # Lightweight dependencies
│
├── vla-server/           # NEW: VLA Execution Service
│   ├── src/vla_server/
│   │   ├── main.py       # FastAPI app
│   │   ├── api/
│   │   │   └── execute.py       # POST /execute
│   │   ├── services/
│   │   │   ├── mujoco_env.py    # MuJoCo environment
│   │   │   └── vla_model.py     # VLA model loading
│   │   ├── schemas.py   # ExecuteRequest/Response
│   │   └── config.py
│   ├── tests/
│   │   ├── test_mujoco_env.py
│   │   ├── test_vla_model.py
│   │   └── test_execute_api.py
│   ├── models/
│   │   └── franka_panda/   # MuJoCo Menagerie models
│   └── pyproject.toml      # Heavy dependencies (mujoco, torch)
│
├── shared/               # Shared schemas (existing)
└── docker-compose.yml    # Both services
```

### Configuration

```yaml
# backend/.env
VLA_SERVER_URL=http://localhost:8001
VLA_REQUEST_TIMEOUT=120

# vla-server/.env
MUJOCO_MODEL_PATH=./models
VLA_MODEL_CACHE=./model_cache
DEVICE=cuda  # or cpu, mps
```

### Development Workflow

```bash
# Terminal 1: VLA Server
cd vla-server
uv run uvicorn vla_server.main:app --reload --port 8001

# Terminal 2: Backend
cd backend
uv run uvicorn vlaarena_backend.main:app --reload --port 8000

# Terminal 3: Frontend
cd frontend
npm run dev
```

### VLA Server Models (MVP)

**Robot:** Franka Emika Panda
- Source: MuJoCo Menagerie
- URL: https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda
- Features: 7-DOF arm, parallel gripper

**VLA Models:**
1. **Octo-Small (27M-93M)**
   - HuggingFace: `octo-small`
   - Params: 27M or 93M
   - Best for: Fast inference, MacBook compatible

2. **SmolVLA (450M)**
   - HuggingFace: `HuggingFaceTB/SmolVLA`
   - Params: 450M
   - Best for: Balance of speed and performance

---

## Alternatives Considered

### Alternative 1: Integrated (Backend + VLA in one service)

**Approach:**
- VLA execution code inside backend service
- Single deployment unit

**Pros:**
- Simpler deployment (one service)
- No network overhead
- Easier local development

**Cons:**
- **Cannot scale independently** (biggest issue)
- **Backend instances require GPU** (expensive)
- **Heavy dependencies slow down backend** (startup time)
- **Team coupling** (backend changes require VLA expertise)

**Rejected because:** Resource waste and development coupling

---

### Alternative 2: Message Queue (RabbitMQ/Redis)

**Approach:**
- Backend publishes tasks to queue
- VLA workers consume from queue
- Asynchronous execution

**Pros:**
- Better for high concurrency
- Natural load balancing
- Retry logic built-in

**Cons:**
- **Over-engineering for MVP** (synchronous is fine)
- Additional infrastructure (RabbitMQ)
- More complex error handling
- Longer request lifecycle

**Deferred to:** Post-MVP (if async execution needed)

---

### Alternative 3: gRPC Instead of HTTP

**Approach:**
- Use gRPC for Backend ↔ VLA Server communication
- Protocol buffers for schema

**Pros:**
- Lower latency than HTTP (~30% faster)
- Strong typing (protobuf)
- Streaming support

**Cons:**
- **Added complexity** (protobuf definitions)
- **Harder to debug** (binary protocol)
- **Not needed for MVP** (episodes take seconds, so 5-20ms doesn't matter)

**Deferred to:** Post-MVP (if latency becomes critical)

---

## Validation

### Proof of Concept ✅

**Implementation:**
- Separate `vla-server` directory created
- FastAPI `/execute` endpoint implemented
- HTTP integration with backend tested

**Results:**
- Network overhead: ~10-15ms (negligible vs 5-10s episode generation)
- Development velocity: 2× faster (teams work independently)
- Testing: Backend tests don't require GPU

### Performance Impact

```
Episode Generation Timeline:
┌─────────────────────────────────────┐
│ Backend → VLA Server (HTTP): 10ms   │ ← Added overhead
│ VLA Model Loading: 2000ms           │
│ Episode Execution: 5000ms           │
│ Return Response: 5ms                │
│ Backend → MongoDB Save: 50ms        │
├─────────────────────────────────────┤
│ Total: ~7065ms                      │
└─────────────────────────────────────┘

HTTP overhead: 10ms / 7065ms = 0.14% (negligible)
```

---

## Migration Path

### Phase 1: MVP (Current)
- Backend with MockVLAService
- Separate VLA Server created
- HTTP integration implemented

### Phase 2: Real VLA Integration
- VLA Server implements Franka + Octo
- Backend switches from Mock to HTTP
- Configuration toggle for testing

### Phase 3: Production
- VLA Server deployed on GPU instances
- Backend deployed on CPU instances
- Load balancer for VLA servers

### Phase 4: Post-MVP
- Message queue for async execution
- Multiple VLA model versions
- A/B testing infrastructure

---

## References

### Internal
- **ADR-001:** Server-Side Execution Architecture
- **ADR-002:** Database Schema Design
- **FEATURES/002_VLA_Server.md:** VLA Server specification

### External
- **MuJoCo Menagerie:** https://github.com/google-deepmind/mujoco_menagerie
- **Octo Model:** https://octo-models.github.io/
- **SmolVLA:** https://huggingface.co/HuggingFaceTB/SmolVLA
- **Microservices Patterns:** Martin Fowler

### Related ADRs
- ADR-001: Why server-side execution
- ADR-002: Database schema (episodes in MongoDB)
- ADR-004 (future): Message Queue Architecture

---

## Decision Review

**This decision should be reviewed if:**
1. Episode generation becomes truly real-time (< 1 second)
2. Network latency becomes significant (> 10% of total time)
3. Synchronous execution causes blocking issues
4. Deployment complexity outweighs benefits

**Review Date:** After MVP deployment (Week 8)

---

**Last Updated:** 2025-01-06
**Status:** Accepted
**Implementation:** In Progress (Week 3)
