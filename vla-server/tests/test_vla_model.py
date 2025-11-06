"""
Tests for VLA model loading and inference
Following TDD: Red → Green → Refactor

Test Coverage:
1. Model loading (Octo-Small, SmolVLA)
2. Device management (CPU, CUDA, MPS)
3. Inference functionality
4. Error handling
"""

import numpy as np
import pytest

from vla_server.services.vla_model import VLAModelManager


class TestVLAModelManagerInitialization:
    """Test suite for VLA model initialization"""

    @pytest.mark.skip(reason="Requires downloading large model - run manually")
    def test_load_octo_small(self):
        """
        Test loading Octo-Small model

        Arrange: Create model manager
        Act: Load octo-small
        Assert: Model loaded successfully
        """
        # Arrange & Act
        manager = VLAModelManager(model_id="octo-small", device="cpu")

        # Assert
        assert manager.model is not None
        assert manager.device == "cpu"

    @pytest.mark.skip(reason="Requires downloading large model - run manually")
    def test_load_smolvla(self):
        """
        Test loading SmolVLA model

        Arrange: Create model manager
        Act: Load smolvla
        Assert: Model loaded successfully
        """
        # Arrange & Act
        manager = VLAModelManager(model_id="smolvla", device="cpu")

        # Assert
        assert manager.model is not None
        assert manager.device == "cpu"

    def test_invalid_model_id_raises_error(self):
        """
        Test that invalid model ID raises ValueError

        Arrange: Prepare invalid model ID
        Act: Attempt to create model manager
        Assert: Raises ValueError
        """
        # Arrange & Act & Assert
        with pytest.raises(ValueError, match="Unknown model"):
            VLAModelManager(model_id="invalid_model", device="cpu")

    def test_model_hub_ids_mapping(self):
        """
        Test that model hub IDs are properly mapped

        Arrange: Check MODEL_HUB_IDS
        Act: Access class variable
        Assert: Contains expected models
        """
        # Arrange & Act
        hub_ids = VLAModelManager.MODEL_HUB_IDS

        # Assert
        assert "octo-small" in hub_ids
        assert "smolvla" in hub_ids
        assert hub_ids["octo-small"] == "octo-models/octo-small"
        assert hub_ids["smolvla"] == "HuggingFaceTB/SmolVLA"


class TestVLAModelManagerDevice:
    """Test suite for device management"""

    def test_device_auto_selects_cpu_when_no_gpu(self):
        """
        Test that auto device selection works

        Arrange: Use auto device selection
        Act: Get device
        Assert: Selects appropriate device
        """
        # Arrange & Act
        manager = VLAModelManager.__new__(VLAModelManager)
        device = manager._get_device("auto")

        # Assert
        assert device in ["cpu", "cuda", "mps"]

    def test_device_cuda_when_specified(self):
        """
        Test explicit CUDA device specification

        Arrange: Specify cuda device
        Act: Get device
        Assert: Returns cuda
        """
        # Arrange & Act
        manager = VLAModelManager.__new__(VLAModelManager)
        device = manager._get_device("cuda")

        # Assert
        assert device == "cuda"

    def test_device_cpu_when_specified(self):
        """
        Test explicit CPU device specification

        Arrange: Specify cpu device
        Act: Get device
        Assert: Returns cpu
        """
        # Arrange & Act
        manager = VLAModelManager.__new__(VLAModelManager)
        device = manager._get_device("cpu")

        # Assert
        assert device == "cpu"

    def test_device_mps_when_specified(self):
        """
        Test explicit MPS device specification

        Arrange: Specify mps device
        Act: Get device
        Assert: Returns mps
        """
        # Arrange & Act
        manager = VLAModelManager.__new__(VLAModelManager)
        device = manager._get_device("mps")

        # Assert
        assert device == "mps"


class TestVLAModelManagerInference:
    """Test suite for inference functionality"""

    @pytest.mark.skip(reason="Requires model loading - integration test")
    def test_inference_returns_action(self):
        """
        Test VLA inference returns valid action

        Arrange: Load model, create observation
        Act: Run inference
        Assert: Returns 8-dim action
        """
        # Arrange
        manager = VLAModelManager(model_id="octo-small", device="cpu")
        obs = {"image": np.zeros((224, 224, 3), dtype=np.uint8), "qpos": [0.0] * 7}
        instruction = "Pick up the cube"

        # Act
        action = manager.predict(obs, instruction)

        # Assert
        assert len(action) == 8
        assert all(isinstance(a, float) for a in action)

    @pytest.mark.skip(reason="Requires model loading - integration test")
    def test_inference_with_different_instructions(self):
        """
        Test inference with different instructions

        Arrange: Load model
        Act: Run inference with multiple instructions
        Assert: Returns valid actions for each
        """
        # Arrange
        manager = VLAModelManager(model_id="octo-small", device="cpu")
        obs = {"image": np.zeros((224, 224, 3), dtype=np.uint8), "qpos": [0.0] * 7}
        instructions = ["Pick up the red cube", "Move to the left", "Open gripper"]

        # Act & Assert
        for instruction in instructions:
            action = manager.predict(obs, instruction)
            assert len(action) == 8
            assert all(isinstance(a, float) for a in action)

    @pytest.mark.skip(reason="Requires model loading - integration test")
    def test_inference_action_bounds(self):
        """
        Test that inference returns actions within reasonable bounds

        Arrange: Load model, create observation
        Act: Run inference
        Assert: Actions are within [-1, 1] or reasonable range
        """
        # Arrange
        manager = VLAModelManager(model_id="octo-small", device="cpu")
        obs = {"image": np.zeros((224, 224, 3), dtype=np.uint8), "qpos": [0.0] * 7}
        instruction = "Pick up the cube"

        # Act
        action = manager.predict(obs, instruction)

        # Assert - actions should be bounded (model-specific)
        assert all(-10 < a < 10 for a in action)  # Reasonable bounds


class TestVLAModelManagerErrorHandling:
    """Test suite for error handling"""

    def test_create_dummy_model_for_testing(self):
        """
        Test creating a dummy model manager for testing purposes

        Arrange: Use __new__ to create instance without init
        Act: Set attributes manually
        Assert: Can create test instance
        """
        # Arrange & Act
        manager = VLAModelManager.__new__(VLAModelManager)
        manager.model_id = "test-model"
        manager.device = "cpu"
        manager.model = None
        manager.processor = None

        # Assert
        assert manager.model_id == "test-model"
        assert manager.device == "cpu"
