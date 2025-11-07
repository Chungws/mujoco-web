"""
Tests for Mock VLA Adapter
"""

import numpy as np
import pytest

from vla_mock.adapters.mock_adapter import MockVLAAdapter


@pytest.fixture
def adapter():
    """Fixture to provide mock adapter"""
    adapter = MockVLAAdapter()
    adapter.load_model(model_id="mock-test", device="cpu", cache_dir="/tmp")
    return adapter


@pytest.fixture
def obs():
    """Fixture to provide sample observation"""
    return {
        "image": np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8),
        "qpos": np.zeros(7, dtype=np.float32),
        "qvel": np.zeros(7, dtype=np.float32),
    }


class TestMockVLAAdapterInit:
    """Test adapter initialization"""

    def test_init_creates_unloaded_adapter(self):
        """Test initialization creates unloaded adapter"""
        adapter = MockVLAAdapter()
        assert not adapter.model_loaded
        assert adapter.model_id is None
        assert adapter.device is None


class TestMockVLAAdapterLoadModel:
    """Test model loading"""

    def test_load_model_sets_attributes(self):
        """Test load_model sets attributes"""
        adapter = MockVLAAdapter()
        adapter.load_model(model_id="mock-test", device="cpu", cache_dir="/tmp")

        assert adapter.model_loaded
        assert adapter.model_id == "mock-test"
        assert adapter.device == "cpu"

    def test_load_model_with_empty_id_raises_error(self):
        """Test load_model with empty model_id raises error"""
        adapter = MockVLAAdapter()
        with pytest.raises(ValueError, match="model_id cannot be empty"):
            adapter.load_model(model_id="", device="cpu", cache_dir="/tmp")


class TestMockVLAAdapterPreprocessObservation:
    """Test observation preprocessing"""

    def test_preprocess_observation_returns_same_obs(self, adapter, obs):
        """Test preprocess_observation returns same observation"""
        processed = adapter.preprocess_observation(obs)

        assert "image" in processed
        assert "qpos" in processed
        assert "qvel" in processed
        np.testing.assert_array_equal(processed["image"], obs["image"])

    def test_preprocess_observation_missing_keys_raises_error(self, adapter):
        """Test preprocess_observation with missing keys raises error"""
        invalid_obs = {"image": np.zeros((224, 224, 3))}

        with pytest.raises(ValueError, match="Missing required keys"):
            adapter.preprocess_observation(invalid_obs)

    def test_preprocess_observation_invalid_image_shape_raises_error(self, adapter):
        """Test preprocess_observation with invalid image shape raises error"""
        invalid_obs = {
            "image": np.zeros((224, 224)),  # Missing color channel
            "qpos": np.zeros(7),
            "qvel": np.zeros(7),
        }

        with pytest.raises(ValueError, match="image must be"):
            adapter.preprocess_observation(invalid_obs)


class TestMockVLAAdapterPreprocessInstruction:
    """Test instruction preprocessing"""

    def test_preprocess_instruction_returns_same_instruction(self, adapter):
        """Test preprocess_instruction returns same instruction"""
        instruction = "Pick up the red cube"
        processed = adapter.preprocess_instruction(instruction)

        assert processed == instruction

    def test_preprocess_instruction_empty_raises_error(self, adapter):
        """Test preprocess_instruction with empty instruction raises error"""
        with pytest.raises(ValueError, match="instruction cannot be empty"):
            adapter.preprocess_instruction("")


class TestMockVLAAdapterPredict:
    """Test prediction"""

    def test_predict_returns_8dim_action(self, adapter, obs):
        """Test predict returns 8-dim action"""
        processed_obs = adapter.preprocess_observation(obs)
        instruction = "Pick up the red cube"

        action = adapter.predict(processed_obs, instruction)

        assert isinstance(action, np.ndarray)
        assert action.shape == (8,)
        assert action.dtype == np.float32

    def test_predict_is_deterministic(self, adapter, obs):
        """Test predict returns same action for same instruction"""
        processed_obs = adapter.preprocess_observation(obs)
        instruction = "Pick up the red cube"

        action1 = adapter.predict(processed_obs, instruction)
        action2 = adapter.predict(processed_obs, instruction)

        np.testing.assert_array_equal(action1, action2)

    def test_predict_before_load_raises_error(self, obs):
        """Test predict before load_model raises error"""
        adapter = MockVLAAdapter()
        processed_obs = adapter.preprocess_observation(obs)

        with pytest.raises(RuntimeError, match="Model not loaded"):
            adapter.predict(processed_obs, "Pick up the red cube")


class TestMockVLAAdapterPostprocessAction:
    """Test action postprocessing"""

    def test_postprocess_action_converts_to_list(self, adapter):
        """Test postprocess_action converts numpy array to list"""
        raw_action = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
        action = adapter.postprocess_action(raw_action)

        assert isinstance(action, list)
        assert len(action) == 8
        assert action == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

    def test_postprocess_action_wrong_shape_raises_error(self, adapter):
        """Test postprocess_action with wrong shape raises error"""
        raw_action = np.array([0.1, 0.2, 0.3])  # Only 3-dim

        with pytest.raises(ValueError, match="must be \\(8,\\)"):
            adapter.postprocess_action(raw_action)
