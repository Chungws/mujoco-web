"""
Pytest configuration for root-level tests
"""


def pytest_addoption(parser):
    """Add --run-integration option to pytest"""
    parser.addoption(
        "--run-integration",
        action="store_true",
        default=False,
        help="Run integration tests that require running services",
    )
