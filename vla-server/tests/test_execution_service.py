"""
Tests for ExecutionService
Following TDD: Red → Green → Refactor

Test Coverage:
1. Service initialization
2. Environment caching
3. Model caching
4. Episode execution
5. Integration with MuJoCo and VLA
6. Error handling
"""

from unittest.mock import Mock, patch

import numpy as np
import pytest

from vla_server.schemas.execute import ExecuteRequest, ExecuteResponse
from vla_server.services.execution_service import ExecutionService


class TestExecutionServiceInitialization:
    """Test suite for ExecutionService initialization"""

    def test_service_initializes_empty_caches(self):
        """
        Test that service initializes with empty caches

        Arrange: Create execution service
        Act: Check cache attributes
        Assert: Caches are empty dicts
        """
        # Arrange & Act
        service = ExecutionService()

        # Assert
        assert hasattr(service, "envs")
        assert hasattr(service, "models")
        assert service.envs == {}
        assert service.models == {}


class TestExecutionServiceCaching:
    """Test suite for environment and model caching"""

    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    def test_environment_cached_on_first_use(self, mock_env_class):
        """
        Test that environment is cached on first use

        Arrange: Create service, mock environment
        Act: Get environment twice
        Assert: Constructor called once, same instance returned
        """
        # Arrange
        service = ExecutionService()
        mock_env = Mock()
        mock_env_class.return_value = mock_env

        # Act
        env1 = service._get_environment("franka", "table")
        env2 = service._get_environment("franka", "table")

        # Assert
        assert mock_env_class.call_count == 1
        assert env1 is env2

    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    def test_different_environments_cached_separately(self, mock_env_class):
        """
        Test that different robot/scene combinations are cached separately

        Arrange: Create service
        Act: Get different environments
        Assert: Constructor called for each unique combination
        """
        # Arrange
        service = ExecutionService()
        mock_env_class.return_value = Mock()

        # Act
        service._get_environment("franka", "table")
        service._get_environment("franka", "kitchen")
        service._get_environment("widowx", "table")

        # Assert
        assert mock_env_class.call_count == 3
        assert len(service.envs) == 3

    @patch("vla_server.services.execution_service.VLAModelManager")
    def test_model_cached_on_first_use(self, mock_model_class):
        """
        Test that model is cached on first use

        Arrange: Create service, mock model
        Act: Get model twice
        Assert: Constructor called once, same instance returned
        """
        # Arrange
        service = ExecutionService()
        mock_model = Mock()
        mock_model_class.return_value = mock_model

        # Act
        model1 = service._get_model("octo-small")
        model2 = service._get_model("octo-small")

        # Assert
        assert mock_model_class.call_count == 1
        assert model1 is model2

    @patch("vla_server.services.execution_service.VLAModelManager")
    def test_different_models_cached_separately(self, mock_model_class):
        """
        Test that different models are cached separately

        Arrange: Create service
        Act: Get different models
        Assert: Constructor called for each model
        """
        # Arrange
        service = ExecutionService()
        mock_model_class.return_value = Mock()

        # Act
        service._get_model("octo-small")
        service._get_model("smolvla")

        # Assert
        assert mock_model_class.call_count == 2
        assert len(service.models) == 2


class TestExecutionServiceExecution:
    """Test suite for episode execution"""

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_returns_response(self, mock_env_class, mock_model_class):
        """
        Test that execute returns valid response

        Arrange: Create service, mock env and model
        Act: Execute request
        Assert: Returns ExecuteResponse
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env.get_observation.return_value = {
            "image": np.zeros((224, 224, 3)),
            "qpos": [0.0] * 7,
            "qvel": [0.0] * 7,
        }
        mock_env.get_state.return_value = {"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}
        mock_env.reset.return_value = mock_env.get_observation.return_value
        mock_env.step.return_value = mock_env.get_observation.return_value
        mock_env_class.return_value = mock_env

        # Mock model
        mock_model = Mock()
        mock_model.predict.return_value = [0.0] * 8
        mock_model_class.return_value = mock_model

        # Create request
        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act
        response = await service.execute(request)

        # Assert
        assert isinstance(response, ExecuteResponse)
        assert len(response.actions) > 0
        assert len(response.states) == len(response.actions)
        assert response.duration_ms > 0

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_runs_max_steps(self, mock_env_class, mock_model_class):
        """
        Test that execution runs for max steps if no early termination

        Arrange: Create service, mock components
        Act: Execute request
        Assert: Returns max_steps actions
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env.get_observation.return_value = {
            "image": np.zeros((224, 224, 3)),
            "qpos": [0.0] * 7,
            "qvel": [0.0] * 7,
        }
        mock_env.get_state.return_value = {"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}
        mock_env.reset.return_value = mock_env.get_observation.return_value
        mock_env.step.return_value = mock_env.get_observation.return_value
        mock_env_class.return_value = mock_env

        # Mock model
        mock_model = Mock()
        mock_model.predict.return_value = [0.0] * 8
        mock_model_class.return_value = mock_model

        # Create request
        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act
        response = await service.execute(request)

        # Assert
        assert len(response.actions) == 50  # max_steps from config
        assert response.metadata["num_steps"] == 50
        assert response.metadata["early_termination"] is False

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_resets_environment(self, mock_env_class, mock_model_class):
        """
        Test that execution resets environment before running

        Arrange: Create service, mock components
        Act: Execute request
        Assert: Environment reset called
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env.get_observation.return_value = {
            "image": np.zeros((224, 224, 3)),
            "qpos": [0.0] * 7,
            "qvel": [0.0] * 7,
        }
        mock_env.get_state.return_value = {"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}
        mock_env.reset.return_value = mock_env.get_observation.return_value
        mock_env.step.return_value = mock_env.get_observation.return_value
        mock_env_class.return_value = mock_env

        # Mock model
        mock_model = Mock()
        mock_model.predict.return_value = [0.0] * 8
        mock_model_class.return_value = mock_model

        # Create request
        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act
        await service.execute(request)

        # Assert
        mock_env.reset.assert_called_once()

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_calls_model_predict(self, mock_env_class, mock_model_class):
        """
        Test that execution calls model predict for each step

        Arrange: Create service, mock components
        Act: Execute request
        Assert: Model predict called for each step
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env.get_observation.return_value = {
            "image": np.zeros((224, 224, 3)),
            "qpos": [0.0] * 7,
            "qvel": [0.0] * 7,
        }
        mock_env.get_state.return_value = {"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}
        mock_env.reset.return_value = mock_env.get_observation.return_value
        mock_env.step.return_value = mock_env.get_observation.return_value
        mock_env_class.return_value = mock_env

        # Mock model
        mock_model = Mock()
        mock_model.predict.return_value = [0.0] * 8
        mock_model_class.return_value = mock_model

        # Create request
        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act
        await service.execute(request)

        # Assert
        assert mock_model.predict.call_count == 50  # max_steps

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_steps_environment(self, mock_env_class, mock_model_class):
        """
        Test that execution steps environment for each action

        Arrange: Create service, mock components
        Act: Execute request
        Assert: Environment step called for each action
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env.get_observation.return_value = {
            "image": np.zeros((224, 224, 3)),
            "qpos": [0.0] * 7,
            "qvel": [0.0] * 7,
        }
        mock_env.get_state.return_value = {"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}
        mock_env.reset.return_value = mock_env.get_observation.return_value
        mock_env.step.return_value = mock_env.get_observation.return_value
        mock_env_class.return_value = mock_env

        # Mock model
        mock_model = Mock()
        mock_model.predict.return_value = [0.0] * 8
        mock_model_class.return_value = mock_model

        # Create request
        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act
        await service.execute(request)

        # Assert
        assert mock_env.step.call_count == 50  # max_steps

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_records_states(self, mock_env_class, mock_model_class):
        """
        Test that execution records states for each step

        Arrange: Create service, mock components
        Act: Execute request
        Assert: States recorded match actions
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env.get_observation.return_value = {
            "image": np.zeros((224, 224, 3)),
            "qpos": [0.0] * 7,
            "qvel": [0.0] * 7,
        }
        mock_env.get_state.return_value = {"qpos": [0.0] * 7, "qvel": [0.0] * 7, "time": 0.0}
        mock_env.reset.return_value = mock_env.get_observation.return_value
        mock_env.step.return_value = mock_env.get_observation.return_value
        mock_env_class.return_value = mock_env

        # Mock model
        mock_model = Mock()
        mock_model.predict.return_value = [0.0] * 8
        mock_model_class.return_value = mock_model

        # Create request
        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act
        response = await service.execute(request)

        # Assert
        assert len(response.states) == len(response.actions)
        for state in response.states:
            assert "qpos" in state.model_dump()
            assert "qvel" in state.model_dump()
            assert "time" in state.model_dump()


class TestExecutionServiceErrorHandling:
    """Test suite for error handling"""

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_propagates_environment_error(self, mock_env_class):
        """
        Test that environment errors are propagated

        Arrange: Create service, mock env to raise error
        Act: Execute request
        Assert: Error propagated
        """
        # Arrange
        service = ExecutionService()
        mock_env_class.side_effect = FileNotFoundError("Robot not found")

        request = ExecuteRequest(
            model_id="octo-small",
            robot_id="invalid",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act & Assert
        with pytest.raises(FileNotFoundError):
            await service.execute(request)

    @pytest.mark.asyncio
    @patch("vla_server.services.execution_service.VLAModelManager")
    @patch("vla_server.services.execution_service.MuJoCoEnvironment")
    async def test_execute_propagates_model_error(self, mock_env_class, mock_model_class):
        """
        Test that model errors are propagated

        Arrange: Create service, mock model to raise error
        Act: Execute request
        Assert: Error propagated
        """
        # Arrange
        service = ExecutionService()

        # Mock environment
        mock_env = Mock()
        mock_env_class.return_value = mock_env

        # Mock model to raise error
        mock_model_class.side_effect = ValueError("Invalid model")

        request = ExecuteRequest(
            model_id="invalid",
            robot_id="franka",
            scene_id="table",
            instruction="Pick up the cube",
        )

        # Act & Assert
        with pytest.raises(ValueError):
            await service.execute(request)
