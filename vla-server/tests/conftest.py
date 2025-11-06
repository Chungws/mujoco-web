"""Pytest configuration and fixtures"""

import pytest


@pytest.fixture
def robot_id():
    """Default robot ID for testing"""
    return "franka"


@pytest.fixture
def scene_id():
    """Default scene ID for testing"""
    return "table"
