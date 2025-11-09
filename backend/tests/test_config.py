"""
Tests for config module - YAML loading and model endpoint configuration
"""

import os
from unittest.mock import mock_open, patch

import pytest
import yaml
from vlaarena_backend.config import get_model_endpoints, load_models_config


class TestLoadModelsConfig:
    """Test YAML configuration loading"""

    def test_load_models_config_success(self):
        """Test successful config loading from default path"""
        config = load_models_config()

        # Verify structure
        assert "models" in config
        assert isinstance(config["models"], list)
        assert len(config["models"]) > 0

        # Verify model fields
        for model in config["models"]:
            assert "id" in model
            assert "base_url" in model
            assert "status" in model

    def test_load_models_config_custom_path(self):
        """Test loading config from custom path"""
        # Create temp config file
        custom_config = {
            "models": [
                {"id": "custom-model", "base_url": "http://localhost:9999", "status": "active"}
            ]
        }

        with (
            patch.dict(os.environ, {"MODELS_CONFIG_PATH": "custom/path/models.yaml"}),
            patch("builtins.open", mock_open(read_data=yaml.dump(custom_config))),
        ):
            config = load_models_config()

            assert len(config["models"]) == 1
            assert config["models"][0]["id"] == "custom-model"

    def test_load_models_config_file_not_found(self):
        """Test error handling when config file doesn't exist"""
        with (
            patch.dict(os.environ, {"MODELS_CONFIG_PATH": "nonexistent/path.yaml"}, clear=False),
            patch("pathlib.Path.exists", return_value=False),
            pytest.raises(FileNotFoundError),
        ):
            load_models_config()

    def test_load_models_config_invalid_yaml(self):
        """Test error handling for invalid YAML"""
        invalid_yaml = "invalid: yaml: content: ["

        with (
            patch("builtins.open", mock_open(read_data=invalid_yaml)),
            pytest.raises(yaml.YAMLError),
        ):
            load_models_config()


class TestGetModelEndpoints:
    """Test model endpoint extraction"""

    def test_get_model_endpoints_active_only(self):
        """Test that only active models are included"""
        mock_config = {
            "models": [
                {"id": "model1", "base_url": "http://localhost:8001", "status": "active"},
                {"id": "model2", "base_url": "http://localhost:8002", "status": "inactive"},
                {"id": "model3", "base_url": "http://localhost:8003", "status": "active"},
            ]
        }

        with patch("vlaarena_backend.config.load_models_config", return_value=mock_config):
            endpoints = get_model_endpoints()

            assert len(endpoints) == 2
            assert "model1" in endpoints
            assert "model3" in endpoints
            assert "model2" not in endpoints
            assert endpoints["model1"] == "http://localhost:8001"
            assert endpoints["model3"] == "http://localhost:8003"

    def test_get_model_endpoints_empty_when_no_active(self):
        """Test empty dict when no active models"""
        mock_config = {
            "models": [
                {"id": "model1", "base_url": "http://localhost:8001", "status": "inactive"},
            ]
        }

        with patch("vlaarena_backend.config.load_models_config", return_value=mock_config):
            endpoints = get_model_endpoints()

            assert len(endpoints) == 0
            assert endpoints == {}

    def test_get_model_endpoints_no_models_key(self):
        """Test handling of config without models key"""
        mock_config = {"other_key": "value"}

        with patch("vlaarena_backend.config.load_models_config", return_value=mock_config):
            endpoints = get_model_endpoints()

            assert endpoints == {}

    def test_get_model_endpoints_returns_dict(self):
        """Test return type is dictionary"""
        mock_config = {
            "models": [
                {"id": "test", "base_url": "http://localhost:8000", "status": "active"},
            ]
        }

        with patch("vlaarena_backend.config.load_models_config", return_value=mock_config):
            endpoints = get_model_endpoints()

            assert isinstance(endpoints, dict)
            assert "test" in endpoints
