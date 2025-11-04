# VLA Arena - Features

This directory contains feature specifications for the VLA Arena platform.

---

## 📋 Active Features

### 001_MVP.md - VLA Arena MVP

**Status:** Week 2 - Backend Foundation (In Progress)
**Timeline:** Week 1-7 (7 weeks total)
**Priority:** HIGH

**Description:**
Complete VLA model comparison platform with blind A/B testing, server-side episode generation, browser-based replay, voting, and robot-specific ELO leaderboard.

**MVP Scope:**
- 1 robot (WidowX)
- 1 scene (Table pick-and-place)
- 2 VLA models (OpenVLA 7B, Octo-base)
- Session management + Multi-turn battles
- State-based episode replay (MuJoCo WASM)
- Voting system (A/B/Tie/Both Bad)
- Robot-specific + Global ELO rankings
- Hourly worker aggregation

**Key Decisions:**
- All server-side execution (MuJoCo + VLA inference) - [ADR-001](../ARCHITECTURE/ADR_001-Server_Side_Execution.md)
- PostgreSQL + MongoDB hybrid - [ADR-002](../ARCHITECTURE/ADR_002-Database_Schema.md)
- State-based replay (not video) for interactive debugging
- Robot-specific ELO for fair comparisons

**See:** [001_MVP.md](./001_MVP.md) for complete specification

---

## 🎯 Post-MVP Features (Planned)

These features are planned for post-MVP releases:

### Phase 2: Content Expansion
- **Priority:** High
- **Features:**
  - Additional robots (Franka, UR5)
  - Additional scenes (Kitchen, Warehouse)
  - 5-10 more VLA models (RT-2, etc.)
  - Scene-specific ELO rankings

### Phase 3: Advanced Visualization
- **Priority:** Medium
- **Features:**
  - Video thumbnail generation
  - Real-time execution streaming
  - Trajectory overlay (MuJoCo Marker API)
  - Multi-step task sequences
  - Attention heatmap visualization

### Phase 4: Infrastructure
- **Priority:** Medium
- **Features:**
  - Ray Serve for environment workers
  - vLLM/TGI for VLA serving
  - Redis caching for leaderboard
  - Prometheus + Grafana monitoring

### Phase 5: Community
- **Priority:** Low
- **Features:**
  - User authentication
  - Model upload interface
  - Public battle links
  - Comments and discussions
  - Custom robot/scene upload
  - Public API

---

## 📝 Feature Request Process

1. Check if feature is in Post-MVP plan above
2. Create GitHub issue with:
   - Clear description
   - Use case / motivation
   - Priority (High/Medium/Low)
   - Estimated complexity
3. Discuss in team meetings
4. Add to roadmap if approved

---

## 🔗 Related Documents

- **[../ROADMAP.md](../ROADMAP.md)** - Week-by-week development timeline
- **[../00_PROJECT.md](../00_PROJECT.md)** - Project overview and architecture
- **[../ARCHITECTURE/](../ARCHITECTURE/)** - Architecture Decision Records (ADR-001, ADR-002)

---

**Last Updated:** 2025-01-04
**Current Focus:** MVP Development (Week 2-7)
**Current Week:** Week 2 - Backend Foundation
