"""
Pytest configuration for root-level tests
"""

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
