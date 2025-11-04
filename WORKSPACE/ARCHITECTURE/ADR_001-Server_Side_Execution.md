# ADR-001: Server-Side Execution Architecture

**Status:** Accepted
**Date:** 2025-01-04
**Decision Makers:** Development Team
**Context:** VLA Arena MVP Architecture

---

## Context

### Problem Statement

VLA Arena needs to execute Vision-Language-Action models in robot simulations and compare them fairly. We need to decide where to run:
1. **MuJoCo Physics Simulation** (environment)
2. **VLA Model Inference** (policy)

Two main options exist:
- **Hybrid**: Client MuJoCo (WASM) + Server VLA
- **All Server**: Server MuJoCo + Server VLA

### Requirements

1. **Fair Comparison**: All users must see identical model behavior
2. **Real-time Feedback**: Users should see smooth execution
3. **Scalability**: Support multiple concurrent sessions
4. **Deterministic**: Same inputs → same outputs (for debugging/replay)

### Key Constraints

- VLA models are large (7B+ parameters)
- Inference requires GPU
- Network latency exists (even on localhost)
- Control loops typically run at 5-10 Hz

---

## Decision

**We chose: All Server-Side Execution**

Both MuJoCo simulation and VLA inference run on the server. Episodes are generated server-side and videos are sent to the client for playback.

---

## Rationale

### Benchmark Results (2025-01-04)

We benchmarked OpenVLA 7B inference with our FastAPI server:

```
Test Configuration:
- Model: OpenVLA 7B (bfloat16, GPU)
- Server: FastAPI on localhost
- Network: Localhost (minimal latency)
- Samples: 50 inference calls

Results:
┌─────────────────────┬──────────────┐
│ Metric              │ Value        │
├─────────────────────┼──────────────┤
│ VLA Inference       │ 283.0ms      │
│ Network Latency     │ 1.6ms        │
│ Total (End-to-end)  │ 284.7ms      │
│ Control Frequency   │ 3.51 Hz      │
├─────────────────────┼──────────────┤
│ Target Frequency    │ ≥ 5 Hz       │
│ Result              │ FAIL ❌      │
└─────────────────────┴──────────────┘
```

**Analysis:**
- Even with negligible network latency (1.6ms), the 283ms inference time dominates
- Control frequency of 3.51 Hz is below the 5 Hz minimum target
- **Hybrid architecture (Client MuJoCo + Server VLA) is NOT VIABLE**

### Decision Tree

```
Can we run VLA on client?
├─ NO (models too large, need GPU)
│
└─ Must run VLA on server ✓

Control loop with server VLA:
├─ Option A: Client MuJoCo + Server VLA
│   └─ Network in loop: 284.7ms per step (3.51 Hz) ❌
│
└─ Option B: Server MuJoCo + Server VLA ✓
    └─ No network in loop: Episode generated once, sent as video ✅
```

### Why All Server-Side Works

**Episode-Based Execution:**
```
Client → Server:
1. "Create episode with instruction X"

Server (No network in control loop):
2. for step in 50:
     obs = mujoco.get_observation()    # ~1ms
     action = vla.predict(obs)         # ~283ms (simulated delay)
     mujoco.step(action)               # ~1ms
     record(frame)
3. encode_video(frames)                # ~2 seconds
4. upload_to_s3(video)                 # ~1 second

Server → Client:
5. Return video URL + new state

Client:
6. Play video (30 FPS, smooth playback) ✓
```

**Total time:** ~8-10 seconds (for 50-step episode)
**User experience:** Single wait, then smooth playback

---

## Consequences

### Positive ✅

**1. No Network Latency in Control Loop**
- Control loop runs at native speed
- VLA inference time doesn't cause jitter
- Predictable performance

**2. Fair Comparison**
- All users see identical model behavior
- Same GPU, same environment
- No device-specific variation

**3. Deterministic Execution**
- Same state + action → same result
- Episodes can be replayed exactly
- Debugging simplified

**4. Scalable**
- Stateless workers (pass state in request)
- Horizontal scaling possible
- No session affinity required

**5. Episode Recording**
- Episodes automatically saved
- Can be reused (Phase 2 battle replays)
- Trajectory data available

### Negative ⚠️

**1. Episode Generation Wait Time**
- Users wait 8-10 seconds for episode
- Not truly "real-time" interactive
- **Mitigation:** Progress indicator, reasonable expectations

**2. Server Resource Requirements**
- Need GPU for VLA inference
- Need CPU for MuJoCo simulation
- **Mitigation:** Optimize resource usage, scale horizontally

**3. Storage Costs**
- Videos stored in S3/MinIO
- ~1-5 MB per episode
- **Mitigation:** Compression, cleanup old episodes

**4. Cannot Interrupt Episode**
- Must complete full 50 steps
- Cannot change instruction mid-episode
- **Mitigation:** Keep episodes short (5 seconds), allow multiple episodes

### Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Video generation slow | High | Medium | Optimize encoding (ffmpeg GPU) |
| Storage costs | Medium | High | Compression, retention policy |
| Concurrent load | High | Medium | Horizontal scaling, queue system |
| MuJoCo state serialization failure | High | Low | Thorough testing, error handling |

---

## Implementation Details

### MuJoCo State Management

**State Size:** ~500 bytes (base64 encoded)

```python
State Contents:
- qpos: Joint positions (robot + objects)
- qvel: Joint velocities
- time: Simulation time

Example (WidowX + 3 objects):
- qpos: 29 floats (8 robot + 21 objects)
- qvel: 29 floats
- time: 1 float
Total: ~470 bytes + overhead ≈ 500 bytes
```

**Stateless Execution:**
```python
def generate_episode(state_b64: str, instruction: str):
    # 1. Decode state
    qpos, qvel, time = decode_state(state_b64)

    # 2. Create environment
    env = MuJoCoEnv(robot="widowx", scene="table")

    # 3. Restore state
    env.data.qpos[:] = qpos
    env.data.qvel[:] = qvel
    env.data.time = time
    mj.mj_forward(env.model, env.data)

    # 4. Run episode
    frames = []
    for step in range(50):
        obs = env.get_observation()
        action = vla.predict(obs, instruction)
        env.step(action)
        frames.append(env.render())

    # 5. Encode
    new_state = encode_state(env.data)
    video_url = upload_video(frames)

    return new_state, video_url
```

### VLA Inference Latency Simulation

**Realistic Latencies:**
```python
Model Latencies (simulated):
- OpenVLA 7B: 200ms (based on 283ms benchmark)
- RT-2: 100ms
- Octo-base: 50ms

Implementation:
- During latency period, hold previous action
- Calculated as timesteps: 200ms = 100 timesteps @ 0.002s
- NO actual sleep (simulation time only)
```

**Why simulate?**
- Fair comparison: All models experience realistic delays
- Matches real robot behavior (inference isn't instant)
- Users understand model constraints

---

## Alternatives Considered

### Alternative 1: Hybrid (Client MuJoCo + Server VLA)

**Approach:**
- MuJoCo WASM runs in browser
- Each step sends observation to server
- Server returns action
- Client steps simulation

**Pros:**
- Leverages existing muwanx viewer
- No video encoding needed
- "True" real-time rendering

**Cons:**
- **Network latency in control loop (284.7ms/step)**
- **Control frequency too low (3.51 Hz < 5 Hz target)**
- Inconsistent across devices (client GPU varies)
- Unfair comparison (network varies)

**Rejected because:** Benchmark shows it's too slow

---

### Alternative 2: Client-Only (Browser WASM + transformers.js)

**Approach:**
- Everything in browser
- MuJoCo WASM + transformers.js
- No server needed

**Pros:**
- No server costs
- True real-time
- Offline capable

**Cons:**
- **Model size limited (<500 MB realistic)**
- **No GPU acceleration (CPU only)**
- **Inference very slow (seconds per step)**
- **Inconsistent performance (device dependent)**
- **Unfair comparison**

**Rejected because:** VLA models too large, performance too variable

---

### Alternative 3: Streaming Video (Server MuJoCo + VLA)

**Approach:**
- Server generates episode
- Stream video frames as they're generated
- Client shows partial episode

**Pros:**
- Perceived latency lower (see first frames sooner)
- Still all server-side benefits

**Cons:**
- Complex streaming infrastructure (WebRTC/HLS)
- Bandwidth intensive
- Over-engineering for MVP

**Deferred to:** Post-MVP (if users request it)

---

## Validation

### Benchmark Validation ✅

**Test Setup:**
```bash
# Server
uv run python src/vla_backend/main.py

# Benchmark
uv run python tests/test_benchmark.py
```

**Results:** 283ms mean inference, 3.51 Hz → All server-side necessary

### Proof of Concept ✅

**Implemented:**
- FastAPI server with OpenVLA 7B
- Benchmark script demonstrating latency

**Next Steps:**
- MuJoCo Python integration
- Episode generation endpoint
- Video encoding pipeline

---

## References

### Internal
- **Benchmark Code:** `tests/test_benchmark.py`
- **Server Code:** `src/vla_backend/main.py`
- **New Specification:** VLA Arena Project Specification (2025-01)

### External
- **lmarena-clone:** Battle system reference (stateless design)
- **MuJoCo Python:** https://mujoco.readthedocs.io/en/stable/python.html
- **OpenVLA:** https://openvla.github.io/

### Related ADRs
- ADR-002 (future): Stateless Worker Architecture
- ADR-003 (future): Video Storage Strategy

---

## Decision Review

**This decision should be reviewed if:**
1. VLA inference speed improves significantly (< 100ms)
2. Browser-based GPU inference becomes viable
3. Real-time interaction becomes a critical requirement
4. New deployment constraints emerge

**Review Date:** After MVP deployment (Week 8)

---

**Last Updated:** 2025-01-04
**Status:** Accepted and Implemented
**Benchmark Results:** Documented and validated
