# Integration Tests

Integration tests for VLA Arena multi-service architecture.

## Overview

Integration tests verify the full end-to-end flow:
1. **Multi-Service Tests** (`test_multi_service.py`) - VLA service communication
2. **Backend Integration Tests** (`test_backend_vla_integration.py`) - Backend → VLA Server → MongoDB

## Prerequisites

### 1. Infrastructure Services

```bash
# Start PostgreSQL + MongoDB
docker compose up -d
```

### 2. VLA Services

**Mock Service (Port 8001):**
```bash
cd vla-servers/mock
uv sync
uv run uvicorn mock_service.server:app --port 8001 --reload
```

**Octo-Small Service (Port 8002) - Optional:**
```bash
cd vla-servers/octo-small
uv sync
uv run uvicorn octo_service.server:app --port 8002 --reload
```

### 3. Backend Service (Port 8000)

```bash
cd backend
uv run uvicorn vlaarena_backend.main:app --port 8000 --reload
```

## Running Integration Tests

### Run All Integration Tests

```bash
# From project root
uv run pytest tests/integration/ --run-integration -v
```

### Run Specific Test File

```bash
# Multi-service tests only
uv run pytest tests/integration/test_multi_service.py --run-integration -v

# Backend integration tests only
uv run pytest tests/integration/test_backend_vla_integration.py --run-integration -v
```

### Run Specific Test Class

```bash
# Test Backend → VLA Server → MongoDB flow
uv run pytest tests/integration/test_backend_vla_integration.py::TestBackendVLAIntegration --run-integration -v
```

### Run with Performance Metrics

```bash
# Show performance logs
uv run pytest tests/integration/test_backend_vla_integration.py::TestBackendVLAIntegration::test_episode_generation_performance --run-integration -v -s
```

## Test Coverage

### Multi-Service Tests (18 tests)

**File:** `test_multi_service.py`

- **Health Checks** (3 tests)
  - Mock service health
  - Octo-Small service health
  - Concurrent health checks

- **Service Info** (3 tests)
  - Mock service info
  - Octo-Small service info
  - Concurrent info requests

- **Predictions** (4 tests)
  - Mock service prediction
  - Octo-Small service prediction
  - Concurrent predictions
  - Different models produce different actions

- **Error Scenarios** (3 tests)
  - Missing instruction
  - Missing XML
  - Invalid XML format

- **Performance** (3 tests)
  - Mock service response time (< 1s)
  - Octo-Small response time (< 5s)
  - Concurrent predictions (< 30s)

- **Communication Patterns** (2 tests)
  - Round-robin load distribution
  - Failover scenario

### Backend Integration Tests (6 tests)

**File:** `test_backend_vla_integration.py`

- **Basic Flow**
  - Session creation via Backend API
  - Turn creation with Mock VLA
  - Multi-turn battle flow

- **MongoDB Verification**
  - Episode data structure
  - Episode storage validation
  - Multi-episode storage

- **Performance**
  - Episode generation < 60s (ROADMAP requirement)

- **Error Handling**
  - VLA server down scenario

- **Direct VLA Access** (debugging)
  - Mock VLA health check
  - Mock VLA predict endpoint

## Expected Results

### When Services Are Running

```bash
$ uv run pytest tests/integration/ --run-integration -v

tests/integration/test_multi_service.py::TestMultiServiceHealthChecks::test_mock_service_health PASSED
tests/integration/test_multi_service.py::TestMultiServiceHealthChecks::test_octo_small_service_health PASSED
...
tests/integration/test_backend_vla_integration.py::TestBackendVLAIntegration::test_turn_creation_with_mock_vla PASSED
tests/integration/test_backend_vla_integration.py::TestBackendVLAIntegration::test_episode_generation_performance PASSED

========================= 24 passed in 45.2s =========================
```

### When Services Are NOT Running

```bash
$ uv run pytest tests/integration/ --run-integration -v

tests/integration/test_multi_service.py::TestMultiServiceHealthChecks::test_mock_service_health FAILED
...
E   httpx.ConnectError: All connection attempts failed

========================= 24 failed in 5.2s ==========================
```

**Note:** This is expected! Integration tests require running services.

### When --run-integration Is Omitted

```bash
$ uv run pytest tests/integration/ -v

tests/integration/test_multi_service.py::TestMultiServiceHealthChecks::test_mock_service_health SKIPPED
...
========================= 24 skipped in 0.02s =========================
```

## Troubleshooting

### Issue: "All connection attempts failed"

**Cause:** VLA services are not running

**Solution:**
```bash
# Terminal 1: Mock VLA Service
cd vla-servers/mock
uv run uvicorn mock_service.server:app --port 8001

# Terminal 2: Octo-Small VLA Service (optional)
cd vla-servers/octo-small
uv run uvicorn octo_service.server:app --port 8002

# Terminal 3: Backend Service
cd backend
uv run uvicorn vlaarena_backend.main:app --port 8000
```

### Issue: "MongoDB connection failed"

**Cause:** MongoDB is not running

**Solution:**
```bash
docker compose up -d mongodb
```

### Issue: "PostgreSQL connection failed"

**Cause:** PostgreSQL is not running

**Solution:**
```bash
docker compose up -d postgres
```

### Issue: Tests pass but no performance metrics shown

**Cause:** Need `-s` flag to show print output

**Solution:**
```bash
uv run pytest tests/integration/ --run-integration -v -s
```

## CI/CD Integration

### Skip Integration Tests in CI (Default)

```yaml
# .github/workflows/test.yml
- name: Run unit tests
  run: uv run pytest  # Integration tests skipped by default
```

### Run Integration Tests in CI

```yaml
# .github/workflows/integration.yml
- name: Start services
  run: docker compose up -d

- name: Start VLA services
  run: |
    cd vla-servers/mock && uv run uvicorn mock_service.server:app --port 8001 &
    cd backend && uv run uvicorn vlaarena_backend.main:app --port 8000 &
    sleep 10  # Wait for services to start

- name: Run integration tests
  run: uv run pytest tests/integration/ --run-integration -v
```

## Performance Benchmarks

From `test_episode_generation_performance`:

| Metric | Target | Typical |
|--------|--------|---------|
| Total episode generation (2 models) | < 60s | ~15-30s |
| Mock VLA single prediction | < 1s | ~0.1s |
| Octo-Small single prediction | < 5s | ~1-3s |
| Concurrent predictions (10 total) | < 30s | ~10-15s |

## Adding New Integration Tests

### 1. Create Test Class

```python
@pytest.mark.asyncio
class TestMyIntegration:
    """Test my integration scenario"""

    async def test_my_scenario(self):
        async with httpx.AsyncClient(timeout=30.0) as client:
            # Your test code
            pass
```

### 2. Mark Test File

```python
# At top of file
pytestmark = pytest.mark.integration
```

### 3. Run Test

```bash
uv run pytest tests/integration/test_my_file.py --run-integration -v
```

## References

- **Multi-Service Architecture:** `WORKSPACE/FEATURES/002_VLA_Server.md`
- **Backend Integration:** `WORKSPACE/FEATURES/001_MVP.md`
- **ROADMAP:** `WORKSPACE/ROADMAP.md`
- **ADR-003:** VLA Server Separation

---

**Last Updated:** 2025-11-10
**Status:** Multi-Service Integration Testing Phase
**Total Integration Tests:** 24 (18 multi-service + 6 backend integration)
