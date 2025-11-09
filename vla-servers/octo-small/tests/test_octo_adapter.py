"""
Tests for OctoSmallAdapter

Tests adapter interface implementation without loading actual model.
"""

import numpy as np
import pytest
from octo_service.adapters.octo_small_adapter import OctoSmallAdapter


class TestOctoSmallAdapterInterface:
    """Test adapter interface implementation"""

    def test_adapter_initialization(self):
        """Test adapter can be instantiated"""
        adapter = OctoSmallAdapter()

        assert adapter.model is None
        assert adapter.model_id is None
        assert adapter.device is None

    def test_preprocess_observation_valid(self, sample_observation):
        """Test observation preprocessing with valid input"""
        adapter = OctoSmallAdapter()
        processed = adapter.preprocess_observation(sample_observation)

        # Check keys
        assert "image_primary" in processed
        assert "timestep_pad_mask" in processed

        # Check shapes
        image = processed["image_primary"]
        assert image.shape == (1, 1, 256, 256, 3)  # [batch, time, H, W, C]
        assert image.dtype == np.float32
        assert image.min() >= 0 and image.max() <= 1  # Normalized

        mask = processed["timestep_pad_mask"]
        assert mask.shape == (1, 1)
        assert mask.dtype == bool
        assert mask[0, 0]  # Valid observation

    def test_preprocess_observation_missing_keys(self):
        """Test preprocessing fails with missing keys"""
        adapter = OctoSmallAdapter()

        with pytest.raises(ValueError, match="Missing required keys"):
            adapter.preprocess_observation({"image": np.zeros((224, 224, 3))})

    def test_preprocess_observation_invalid_image_shape(self):
        """Test preprocessing fails with invalid image shape"""
        adapter = OctoSmallAdapter()

        obs = {
            "image": np.zeros((224, 224), dtype=np.uint8),  # Missing channel
            "qpos": np.zeros(7),
            "qvel": np.zeros(7),
        }

        with pytest.raises(ValueError, match="image must be"):
            adapter.preprocess_observation(obs)

    def test_preprocess_observation_non_array(self):
        """Test preprocessing fails with non-array image"""
        adapter = OctoSmallAdapter()

        obs = {
            "image": [[1, 2, 3]],  # List instead of array
            "qpos": np.zeros(7),
            "qvel": np.zeros(7),
        }

        with pytest.raises(ValueError, match="image must be numpy array"):
            adapter.preprocess_observation(obs)

    def test_preprocess_instruction_valid(self, sample_instruction):
        """Test instruction preprocessing with valid input"""
        adapter = OctoSmallAdapter()
        processed = adapter.preprocess_instruction(sample_instruction)

        assert isinstance(processed, dict)
        assert "text" in processed
        assert processed["text"] == sample_instruction

    def test_preprocess_instruction_empty(self):
        """Test preprocessing fails with empty instruction"""
        adapter = OctoSmallAdapter()

        with pytest.raises(ValueError, match="instruction cannot be empty"):
            adapter.preprocess_instruction("")

        with pytest.raises(ValueError, match="instruction cannot be empty"):
            adapter.preprocess_instruction("   ")

    def test_postprocess_action_valid(self, mock_octo_action):
        """Test action postprocessing with valid input"""
        adapter = OctoSmallAdapter()
        action = adapter.postprocess_action(mock_octo_action)

        # Check format
        assert isinstance(action, list)
        assert len(action) == 8  # 7 joints + gripper

        # Check values
        for val in action:
            assert isinstance(val, float)
            assert -10.0 <= val <= 10.0  # Reasonable range

        # Check gripper is padded (last dimension)
        assert action[7] == 0.0

    def test_postprocess_action_invalid_shape(self):
        """Test postprocessing fails with invalid shape"""
        import jax.numpy as jnp

        adapter = OctoSmallAdapter()

        # Wrong dimensions
        with pytest.raises(ValueError, match="Expected 3D action"):
            adapter.postprocess_action(jnp.array([1.0, 2.0, 3.0]))

        # Wrong batch size
        with pytest.raises(ValueError, match="Expected batch_size=1"):
            invalid_action = jnp.zeros((2, 4, 7))  # batch=2
            adapter.postprocess_action(invalid_action)

    def test_postprocess_action_no_shape_attribute(self):
        """Test postprocessing fails with non-array input"""
        adapter = OctoSmallAdapter()

        with pytest.raises(ValueError, match="must have shape attribute"):
            adapter.postprocess_action([1, 2, 3])

    def test_predict_without_model_loaded(self, sample_observation, sample_instruction):
        """Test prediction fails if model not loaded"""
        adapter = OctoSmallAdapter()

        processed_obs = adapter.preprocess_observation(sample_observation)
        processed_instruction = adapter.preprocess_instruction(sample_instruction)

        with pytest.raises(RuntimeError, match="Model not loaded"):
            adapter.predict(processed_obs, processed_instruction)


class TestOctoSmallAdapterIntegration:
    """
    Integration tests with actual model loading

    Note: These tests are skipped by default to avoid downloading model.
    Run with: pytest --run-integration
    """

    @pytest.mark.integration
    def test_load_model_success(self):
        """Test model loading from HuggingFace"""
        adapter = OctoSmallAdapter()

        # This will download ~100MB model
        adapter.load_model(model_id="octo-small", device="cpu", cache_dir="/tmp/octo_test_cache")

        assert adapter.model is not None
        assert adapter.model_id == "octo-small"
        assert adapter.device == "cpu"

    def test_load_model_empty_id(self):
        """Test load_model fails with empty model_id"""
        adapter = OctoSmallAdapter()

        with pytest.raises(ValueError, match="model_id cannot be empty"):
            adapter.load_model(model_id="", device="cpu", cache_dir="/tmp")

    @pytest.mark.integration
    def test_full_inference_pipeline(self, sample_observation, sample_instruction):
        """Test full inference pipeline (load → preprocess → predict → postprocess)"""
        adapter = OctoSmallAdapter()

        # Load model
        adapter.load_model(model_id="octo-small", device="cpu", cache_dir="/tmp/octo_test_cache")

        # Preprocess
        processed_obs = adapter.preprocess_observation(sample_observation)
        processed_instruction = adapter.preprocess_instruction(sample_instruction)

        # Predict
        raw_action = adapter.predict(processed_obs, processed_instruction)

        # Check raw action shape
        assert raw_action.shape == (1, 4, 7)  # batch, pred_horizon, action_dim

        # Postprocess
        action = adapter.postprocess_action(raw_action)

        # Validate output
        assert len(action) == 8
        assert all(isinstance(v, float) for v in action)
        assert action[7] == 0.0  # Gripper padded
