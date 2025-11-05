# VLA Arena Worker

Data aggregation worker for VLA Arena. Runs hourly cron job to calculate ELO ratings from battle votes.

## Quick Start

### Prerequisites
- Python 3.11+
- uv package manager
- PostgreSQL 16+
- MongoDB

### Development Setup

```bash
# From project root
uv sync

# Run worker manually (one-time execution)
cd worker
uv run python -m vlaarena_worker.main

# Or from workspace root
uv run --package vlaarena-worker python -m vlaarena_worker.main
```

### Environment Variables

Copy `.env.example` to `.env` and configure:

```bash
cp .env.example .env
```

Key settings:
- `POSTGRES_URI`: PostgreSQL connection string
- `MONGODB_URI`: MongoDB connection string
- `WORKER_INTERVAL_HOURS`: How often to run aggregation (default: 1)
- `WORKER_TIMEZONE`: Timezone for scheduler (default: UTC)

### Running Tests

```bash
cd worker
uv run --with pytest --with pytest-asyncio pytest -s
```

### Code Quality

```bash
# Linting (includes import sorting)
uvx ruff check

# Formatting
uvx ruff format --check
```

## How It Works

### Scheduling
- Runs hourly at :00 (e.g., 00:00, 01:00, 02:00, ...)
- Configurable interval via `WORKER_INTERVAL_HOURS`
- Uses APScheduler with AsyncIOScheduler
- Timezone configurable via `WORKER_TIMEZONE`

### Aggregation Process

1. **Read New Votes** (MongoDB)
   - Query votes since last run timestamp
   - Retrieve battle documents for model positions
   - Filter out already processed votes

2. **Calculate ELO Ratings**
   - Use standard ELO formula (K-factor = 32, Initial = 1500)
   - Handle vote types: `left_better`, `right_better`, `tie`, `both_bad`
   - Convert votes to scores using model positions

3. **Calculate Confidence Intervals**
   - Bradley-Terry Model: `CI = 1.96 * (400 / sqrt(n))`
   - 95% confidence interval
   - Default CI = 200.0 for models with 0 votes

4. **Update PostgreSQL**
   - Upsert `model_stats` table with new ELO scores
   - Update vote counts, win/loss/tie counts, win rates
   - Store confidence intervals

5. **Update Worker Status**
   - Record last run timestamp in `worker_status` table
   - Log votes processed and status

## Project Structure

```
worker/
├── src/
│   └── vlaarena_worker/
│       ├── __init__.py
│       ├── main.py           # Worker entry point
│       └── aggregators/      # ELO aggregation logic
├── tests/                    # pytest tests
├── pyproject.toml
├── .env.example
└── README.md
```

## ELO Calculation

### Formula

```python
expected_a = 1 / (1 + 10 ** ((rating_b - rating_a) / 400))
new_rating_a = rating_a + k * (result - expected_a)
```

### Vote to Score Mapping

| Vote | Model A Position | Score for A | Score for B |
|------|-----------------|-------------|-------------|
| `left_better` | left | 1.0 | 0.0 |
| `left_better` | right | 0.0 | 1.0 |
| `right_better` | left | 0.0 | 1.0 |
| `right_better` | right | 1.0 | 0.0 |
| `tie` | any | 0.5 | 0.5 |
| `both_bad` | any | 0.25 | 0.25 |

### Constants

- **Initial ELO**: 1500
- **K-Factor**: 32
- **Minimum Votes**: 5 (for leaderboard display)

## Robot-Specific ELO

**Key Feature:** VLA Arena calculates separate ELO rankings per robot.

**Tables:**
- `ModelStatsByRobot` - ELO per (model_id, robot_id)
- `ModelStatsTotal` - Global ELO across all robots

**Example:**
```
OpenVLA 7B:
- WidowX ELO: 1650 (50 votes)
- Franka ELO: 1580 (30 votes)
- Global ELO: 1620 (80 votes)
```

## Related Documentation

- [MVP Feature Spec](../WORKSPACE/FEATURES/001_MVP.md) - Worker requirements
- [SQLModel No Foreign Keys](./.claude/skills/sqlmodel-no-foreign-keys/SKILL.md) - Database modeling
- [Backend TDD Workflow](./.claude/skills/backend-tdd-workflow/SKILL.md) - Testing guide
- [shared/src/vlaarena_shared/models.py](../shared/src/vlaarena_shared/models.py) - PostgreSQL models

---

**Last Updated:** 2025-11-05
