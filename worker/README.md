# llmbattler-worker

Data aggregation worker for LLM Battle Arena. Runs hourly cron job to calculate ELO ratings from battle votes.

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
uv run python -m llmbattler_worker.main

# Or from workspace root
uv run --package llmbattler-worker python -m llmbattler_worker.main
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
# Linting
uvx ruff check src tests

# Formatting
uvx ruff format --check src tests

# Import sorting
uvx isort --check --profile black src tests
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
│   └── llmbattler_worker/
│       ├── __init__.py
│       ├── main.py           # Worker entry point
│       └── aggregators/      # Aggregation logic
├── tests/                    # pytest tests
├── Dockerfile
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

## TODO

- [ ] Implement MongoDB vote reader
- [ ] Implement ELO calculation algorithm
- [ ] Implement PostgreSQL writer
- [ ] Add worker status tracking
- [ ] Add comprehensive tests
- [ ] Add error handling and retries
- [ ] Add metrics/monitoring

## Related Documentation

- [WORKSPACE/CONVENTIONS/backend/](../WORKSPACE/CONVENTIONS/backend/) - Python coding standards
- [WORKSPACE/FEATURES/002_LEADERBOARD_MVP.md](../WORKSPACE/FEATURES/002_LEADERBOARD_MVP.md) - Leaderboard spec
- [shared/src/llmbattler_shared/models.py](../shared/src/llmbattler_shared/models.py) - PostgreSQL models
