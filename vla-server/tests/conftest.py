"""
Pytest configuration and fixtures for VLA Server tests
"""

import pytest


@pytest.fixture
def sample_instruction():
    """Sample instruction for testing"""
    return "Pick up the red cube"


@pytest.fixture
def sample_execute_request():
    """Sample execute request data"""
    return {
        "model_id": "octo-small",
        "robot_id": "franka",
        "scene_id": "table",
        "instruction": "Pick up the red cube",
    }
