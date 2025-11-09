"""
Pytest configuration for octo-small tests
"""

import numpy as np
import pytest


def pytest_addoption(parser):
    """Add --run-integration option to pytest"""
    parser.addoption(
        "--run-integration",
        action="store_true",
        default=False,
        help="Run integration tests that require running services",
    )


def pytest_collection_modifyitems(config, items):
    """Skip integration tests unless --run-integration is specified"""
    if config.getoption("--run-integration"):
        # When --run-integration is specified, run integration tests
        return

    skip_integration = pytest.mark.skip(reason="Need --run-integration option to run")
    for item in items:
        if "integration" in item.keywords:
            item.add_marker(skip_integration)


@pytest.fixture
def sample_observation():
    """Sample MuJoCo observation"""
    return {
        "image": np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8),
        "qpos": np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float32),
        "qvel": np.array([0.0] * 7, dtype=np.float32),
    }


@pytest.fixture
def sample_instruction():
    """Sample language instruction"""
    return "pick up the red cube"


@pytest.fixture
def mock_octo_action():
    """
    Mock Octo model output
    Shape: (batch=1, pred_horizon=4, action_dim=7)
    """
    import jax.numpy as jnp

    # Generate 4 actions (action chunking)
    actions = np.random.uniform(-1.0, 1.0, (1, 4, 7)).astype(np.float32)
    return jnp.array(actions)
