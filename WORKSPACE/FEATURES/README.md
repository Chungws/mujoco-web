# VLA Arena - Features

This directory contains feature specifications for the VLA Arena platform.

---

## 📋 Current Features

### MVP: VLA Arena (In Progress)

**Status:** Week 2 - Backend Foundation
**Target:** Week 7 completion
**Priority:** HIGH

**Description:**
Complete VLA model comparison platform with blind A/B testing, episode generation, video playback, voting, and ELO-based leaderboard.

**Key Components:**
- Session management (random model assignment)
- Episode generation (50-step server-side execution)
- Side-by-side video playback
- Voting system (A/B/Tie)
- ELO leaderboard with confidence intervals
- Hourly aggregation worker

**Scope:**
- 1 robot (WidowX)
- 1 scene (Table)
- 2 VLA models (OpenVLA 7B, Octo-base)

**See:** [../ROADMAP.md](../ROADMAP.md) for detailed breakdown

---

## 🗂️ Archived Specifications

The following documents represent earlier planning phases before the MVP specification was finalized. They are kept for historical reference.

### 001_VLA_VISUALIZATION.md (Archived)

**Original Plan:** Phase 1 - Visualization only
**Status:** Superseded by integrated MVP approach
**Date:** 2025-11-01

**Why Archived:**
- Original plan separated visualization (Phase 1) from comparison (Phase 2)
- New specification integrates both into unified MVP
- Benchmark results (283ms VLA inference) confirmed all server-side architecture
- lmarena-clone structure adopted for faster development

**Useful Sections:**
- Plugin architecture concepts (adapted to MVP)
- MuJoCo integration details
- Technical specifications (still relevant)

### 002_MODEL_COMPARISON.md (Archived)

**Original Plan:** Phase 2 - Battle system
**Status:** Integrated into MVP
**Date:** 2025-11-01

**Why Archived:**
- Battle system now part of MVP (not separate phase)
- Database schema refined based on lmarena-clone
- ELO system directly adopted from reference project

**Useful Sections:**
- Battle flow (now called "sessions" and "episodes")
- ELO calculation details
- Leaderboard requirements

---

## 🎯 Post-MVP Features (Planned)

These features are planned for post-MVP releases:

### Expansion: More Content
- **Priority:** High
- **Features:**
  - Additional robots (Franka, UR5)
  - Additional scenes (Kitchen, Warehouse)
  - More VLA models (RT-2, etc.)

### Advanced Visualization
- **Priority:** Medium
- **Features:**
  - Trajectory overlay (MuJoCo Marker API)
  - Step-by-step replay with timeline
  - Synchronized video playback
  - Downloadable videos

### Infrastructure Improvements
- **Priority:** Medium
- **Features:**
  - Ray Serve for environment workers
  - vLLM for VLA serving (auto-batching)
  - Redis caching
  - Monitoring (Prometheus + Grafana)

### Community Features
- **Priority:** Low
- **Features:**
  - User authentication
  - Model upload interface
  - Custom robot/scene upload
  - Public API
  - Comments and discussions

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

- **[../ROADMAP.md](../ROADMAP.md)** - Development timeline and progress
- **[../00_PROJECT.md](../00_PROJECT.md)** - Project overview and architecture
- **[../ARCHITECTURE/](../ARCHITECTURE/)** - Architecture Decision Records

---

**Last Updated:** 2025-01-04
**Current Focus:** MVP Development (Week 2-7)
