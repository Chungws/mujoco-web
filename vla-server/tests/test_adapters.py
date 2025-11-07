"""
Tests for VLA adapters

Tests:
- Factory function (get_adapter)
- MockVLAAdapter implementation
- OctoSmallAdapter implementation
- Base adapter interface
- Error handling

Total: 30 tests (20 Mock + 10 OctoSmall)
"""

import numpy as np
import pytest
from vla_server.adapters import (
    MockVLAAdapter,
    OctoSmallAdapter,
    VLAModelAdapter,
    get_adapter,
)

# ============================================================================
# Factory Tests (5 tests)
# ============================================================================


def test_get_adapter_returns_adapter_instance():
    """Test that get_adapter returns VLAModelAdapter instance"""
    adapter = get_adapter("mock")
    assert isinstance(adapter, VLAModelAdapter)


def test_get_adapter_returns_mock_adapter():
    """Test that get_adapter('mock') returns MockVLAAdapter"""
    adapter = get_adapter("mock")
    assert isinstance(adapter, MockVLAAdapter)


def test_get_adapter_unknown_model_raises_error():
    """Test that get_adapter raises ValueError for unknown model"""
    with pytest.raises(ValueError, match="Unknown model_id"):
        get_adapter("unknown-model")


def test_get_adapter_error_message_includes_available_models():
    """Test that error message includes available models"""
    with pytest.raises(ValueError, match="Available models"):
        get_adapter("invalid")


def test_get_adapter_returns_new_instance_each_time():
    """Test that get_adapter returns new instance each call"""
    adapter1 = get_adapter("mock")
    adapter2 = get_adapter("mock")
    assert adapter1 is not adapter2


def test_get_adapter_returns_octo_small_adapter():
    """Test that get_adapter('octo-small') returns OctoSmallAdapter"""
    adapter = get_adapter("octo-small")
    assert isinstance(adapter, OctoSmallAdapter)
    assert isinstance(adapter, VLAModelAdapter)


# ============================================================================
# MockVLAAdapter - Load Model Tests (3 tests)
# ============================================================================


def test_mock_adapter_load_model_success():
    """Test that load_model sets attributes correctly"""
    adapter = MockVLAAdapter()
    adapter.load_model("mock", "cpu", "./cache")

    assert adapter.model_loaded is True
    assert adapter.model_id == "mock"
    assert adapter.device == "cpu"


def test_mock_adapter_load_model_empty_model_id_raises_error():
    """Test that load_model raises ValueError for empty model_id"""
    adapter = MockVLAAdapter()
    with pytest.raises(ValueError, match="model_id cannot be empty"):
        adapter.load_model("", "cpu", "./cache")


def test_mock_adapter_initial_state():
    """Test that MockVLAAdapter initializes with correct state"""
    adapter = MockVLAAdapter()
    assert adapter.model_loaded is False
    assert adapter.model_id is None
    assert adapter.device is None


# ============================================================================
# MockVLAAdapter - Preprocess Observation Tests (5 tests)
# ============================================================================


def test_mock_adapter_preprocess_observation_success():
    """Test that preprocess_observation returns correct format"""
    adapter = MockVLAAdapter()
    obs = {
        "image": np.zeros((224, 224, 3), dtype=np.uint8),
        "qpos": np.array([0.1, 0.2, 0.3]),
        "qvel": np.array([0.0, 0.0, 0.0]),
    }

    result = adapter.preprocess_observation(obs)

    assert "image" in result
    assert "qpos" in result
    assert "qvel" in result
    assert np.array_equal(result["image"], obs["image"])
    assert np.array_equal(result["qpos"], obs["qpos"])
    assert np.array_equal(result["qvel"], obs["qvel"])


def test_mock_adapter_preprocess_observation_missing_keys_raises_error():
    """Test that preprocess_observation raises ValueError for missing keys"""
    adapter = MockVLAAdapter()
    obs = {"image": np.zeros((224, 224, 3), dtype=np.uint8)}

    with pytest.raises(ValueError, match="Missing required keys"):
        adapter.preprocess_observation(obs)


def test_mock_adapter_preprocess_observation_invalid_image_type_raises_error():
    """Test that preprocess_observation raises ValueError for non-array image"""
    adapter = MockVLAAdapter()
    obs = {
        "image": [[0, 0, 0]],  # List instead of numpy array
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }

    with pytest.raises(ValueError, match="image must be numpy array"):
        adapter.preprocess_observation(obs)


def test_mock_adapter_preprocess_observation_invalid_image_shape_raises_error():
    """Test that preprocess_observation raises ValueError for invalid image shape"""
    adapter = MockVLAAdapter()
    obs = {
        "image": np.zeros((224, 224), dtype=np.uint8),  # Missing channel dimension
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }

    with pytest.raises(ValueError, match="image must be"):
        adapter.preprocess_observation(obs)


def test_mock_adapter_preprocess_observation_different_image_sizes():
    """Test that preprocess_observation works with different image sizes"""
    adapter = MockVLAAdapter()

    # Test with 256x256 image
    obs = {
        "image": np.zeros((256, 256, 3), dtype=np.uint8),
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }
    result = adapter.preprocess_observation(obs)
    assert result["image"].shape == (256, 256, 3)


# ============================================================================
# MockVLAAdapter - Preprocess Instruction Tests (2 tests)
# ============================================================================


def test_mock_adapter_preprocess_instruction_success():
    """Test that preprocess_instruction returns instruction as-is"""
    adapter = MockVLAAdapter()
    instruction = "Pick up the red cube"
    result = adapter.preprocess_instruction(instruction)
    assert result == instruction


def test_mock_adapter_preprocess_instruction_empty_raises_error():
    """Test that preprocess_instruction raises ValueError for empty instruction"""
    adapter = MockVLAAdapter()

    with pytest.raises(ValueError, match="instruction cannot be empty"):
        adapter.preprocess_instruction("")

    with pytest.raises(ValueError, match="instruction cannot be empty"):
        adapter.preprocess_instruction("   ")


# ============================================================================
# MockVLAAdapter - Predict Tests (3 tests)
# ============================================================================


def test_mock_adapter_predict_success():
    """Test that predict returns 8-dim action"""
    adapter = MockVLAAdapter()
    adapter.load_model("mock", "cpu", "./cache")

    obs = {
        "image": np.zeros((224, 224, 3), dtype=np.uint8),
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }
    instruction = "Pick up the cube"

    action = adapter.predict(obs, instruction)

    assert isinstance(action, np.ndarray)
    assert action.shape == (8,)
    assert action.dtype == np.float32


def test_mock_adapter_predict_deterministic():
    """Test that predict returns same action for same instruction"""
    adapter = MockVLAAdapter()
    adapter.load_model("mock", "cpu", "./cache")

    obs = {
        "image": np.zeros((224, 224, 3), dtype=np.uint8),
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }
    instruction = "Pick up the cube"

    action1 = adapter.predict(obs, instruction)
    action2 = adapter.predict(obs, instruction)

    assert np.array_equal(action1, action2)


def test_mock_adapter_predict_not_loaded_raises_error():
    """Test that predict raises RuntimeError if model not loaded"""
    adapter = MockVLAAdapter()

    obs = {"image": np.zeros((224, 224, 3)), "qpos": np.array([0.1]), "qvel": np.array([0.0])}
    instruction = "Pick up the cube"

    with pytest.raises(RuntimeError, match="Model not loaded"):
        adapter.predict(obs, instruction)


# ============================================================================
# MockVLAAdapter - Postprocess Action Tests (2 tests)
# ============================================================================


def test_mock_adapter_postprocess_action_success():
    """Test that postprocess_action converts numpy array to list"""
    adapter = MockVLAAdapter()
    raw_action = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8], dtype=np.float32)

    result = adapter.postprocess_action(raw_action)

    assert isinstance(result, list)
    assert len(result) == 8
    assert all(isinstance(x, float) for x in result)


def test_mock_adapter_postprocess_action_invalid_shape_raises_error():
    """Test that postprocess_action raises ValueError for invalid shape"""
    adapter = MockVLAAdapter()

    # Test with wrong dimension
    raw_action = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    with pytest.raises(ValueError, match="raw_action must be"):
        adapter.postprocess_action(raw_action)

    # Test with non-numpy array
    with pytest.raises(ValueError, match="raw_action must be numpy array"):
        adapter.postprocess_action([0.1, 0.2, 0.3])


# ============================================================================
# OctoSmallAdapter Tests (10 tests)
# ============================================================================


def test_octo_small_adapter_initial_state():
    """Test that OctoSmallAdapter initializes with correct state"""
    adapter = OctoSmallAdapter()
    assert adapter.model is None
    assert adapter.processor is None
    assert adapter.device is None
    assert adapter.model_loaded is False


def test_octo_small_adapter_load_model_wrong_model_id_raises_error():
    """Test that load_model raises ValueError for wrong model_id"""
    adapter = OctoSmallAdapter()
    with pytest.raises(ValueError, match="Expected model_id='octo-small'"):
        adapter.load_model("wrong-model", "cpu", "./cache")


def test_octo_small_adapter_preprocess_observation_resize_image():
    """Test that preprocess_observation resizes images to 224x224"""
    adapter = OctoSmallAdapter()

    # Test with 256x256 image (needs resize)
    obs = {
        "image": np.zeros((256, 256, 3), dtype=np.uint8),
        "qpos": np.array([0.1, 0.2]),
        "qvel": np.array([0.0, 0.0]),
    }

    result = adapter.preprocess_observation(obs)

    assert result["image"].shape == (224, 224, 3)
    assert np.array_equal(result["qpos"], obs["qpos"])
    assert np.array_equal(result["qvel"], obs["qvel"])


def test_octo_small_adapter_preprocess_observation_keeps_224x224():
    """Test that preprocess_observation keeps 224x224 images as-is"""
    adapter = OctoSmallAdapter()

    # Test with already correct size
    obs = {
        "image": np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8),
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }

    result = adapter.preprocess_observation(obs)

    assert result["image"].shape == (224, 224, 3)
    # Should be the same array since no resize needed
    assert np.array_equal(result["image"], obs["image"])


def test_octo_small_adapter_preprocess_observation_missing_keys_raises_error():
    """Test that preprocess_observation raises ValueError for missing keys"""
    adapter = OctoSmallAdapter()
    obs = {"image": np.zeros((224, 224, 3), dtype=np.uint8)}

    with pytest.raises(ValueError, match="Missing required keys"):
        adapter.preprocess_observation(obs)


def test_octo_small_adapter_preprocess_observation_invalid_image_raises_error():
    """Test that preprocess_observation validates image format"""
    adapter = OctoSmallAdapter()

    # Test with non-numpy array
    obs = {
        "image": [[0, 0, 0]],  # List instead of numpy array
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }
    with pytest.raises(ValueError, match="image must be numpy array"):
        adapter.preprocess_observation(obs)

    # Test with wrong shape
    obs = {
        "image": np.zeros((224, 224), dtype=np.uint8),  # Missing channel dimension
        "qpos": np.array([0.1]),
        "qvel": np.array([0.0]),
    }
    with pytest.raises(ValueError, match="image must be"):
        adapter.preprocess_observation(obs)


def test_octo_small_adapter_preprocess_instruction_success():
    """Test that preprocess_instruction returns instruction as-is"""
    adapter = OctoSmallAdapter()
    instruction = "Pick up the red cube"
    result = adapter.preprocess_instruction(instruction)
    assert result == instruction


def test_octo_small_adapter_preprocess_instruction_empty_raises_error():
    """Test that preprocess_instruction raises ValueError for empty instruction"""
    adapter = OctoSmallAdapter()

    with pytest.raises(ValueError, match="instruction cannot be empty"):
        adapter.preprocess_instruction("")

    with pytest.raises(ValueError, match="instruction cannot be empty"):
        adapter.preprocess_instruction("   ")


def test_octo_small_adapter_postprocess_action_7dim_to_8dim():
    """Test that postprocess_action pads 7-dim action to 8-dim"""
    pytest.importorskip("torch")
    import torch

    adapter = OctoSmallAdapter()

    # Test with 7-dim action (no gripper)
    raw_action = torch.tensor([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=torch.float32)
    result = adapter.postprocess_action(raw_action)

    assert isinstance(result, list)
    assert len(result) == 8
    assert result[-1] == 0.0  # Gripper should be padded with 0.0
    assert all(isinstance(x, float) for x in result)


def test_octo_small_adapter_postprocess_action_invalid_type_raises_error():
    """Test that postprocess_action raises ValueError for non-tensor input"""
    pytest.importorskip("torch")

    adapter = OctoSmallAdapter()

    # Test with numpy array instead of tensor
    raw_action = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)
    with pytest.raises(ValueError, match=r"raw_action must be torch\.Tensor"):
        adapter.postprocess_action(raw_action)
