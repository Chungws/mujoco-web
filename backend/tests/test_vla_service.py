"""
Tests for VLA execution service (Mock implementation)

Following TDD workflow:
1. Red: Write failing tests
2. Green: Minimal implementation
3. Refactor: Improve code quality
"""

from vlaarena_backend.services.vla_service import MockVLAService


class TestMockVLAService:
    """Test suite for MockVLAService"""

    def test_generate_episode_returns_episode_data(self):
        """
        Test that generate_episode returns episode data with correct structure

        Arrange: Create service with model config
        Act: Generate episode
        Assert: Returns dict with actions, states, metrics
        """
        # Arrange
        service = MockVLAService()
        model_id = "openvla-7b"
        instruction = "Pick up the red cube"
        robot_id = "widowx"
        scene_id = "table"

        # Act
        episode_data = service.generate_episode(
            model_id=model_id,
            instruction=instruction,
            robot_id=robot_id,
            scene_id=scene_id,
        )

        # Assert
        assert "actions" in episode_data
        assert "states" in episode_data
        assert "metrics" in episode_data
        assert "duration_ms" in episode_data

    def test_generate_episode_actions_format(self):
        """
        Test that actions are 8-dimensional vectors

        Arrange: Create service
        Act: Generate episode
        Assert: Each action is 8-dim list of floats
        """
        # Arrange
        service = MockVLAService()

        # Act
        episode_data = service.generate_episode(
            model_id="openvla-7b",
            instruction="Pick up the red cube",
            robot_id="widowx",
            scene_id="table",
        )

        # Assert
        actions = episode_data["actions"]
        assert isinstance(actions, list)
        assert len(actions) > 0
        assert len(actions) <= 50  # Max 50 steps

        for action in actions:
            assert isinstance(action, list)
            assert len(action) == 8  # 8-dimensional action space
            assert all(isinstance(a, float) for a in action)

    def test_generate_episode_states_format(self):
        """
        Test that states have qpos, qvel, time

        Arrange: Create service
        Act: Generate episode
        Assert: Each state has qpos, qvel, time
        """
        # Arrange
        service = MockVLAService()

        # Act
        episode_data = service.generate_episode(
            model_id="openvla-7b",
            instruction="Pick up the red cube",
            robot_id="widowx",
            scene_id="table",
        )

        # Assert
        states = episode_data["states"]
        assert isinstance(states, list)
        assert len(states) > 0
        assert len(states) <= 50

        for state in states:
            assert "qpos" in state
            assert "qvel" in state
            assert "time" in state
            assert isinstance(state["qpos"], list)
            assert isinstance(state["qvel"], list)
            assert isinstance(state["time"], float)

    def test_generate_episode_metrics_format(self):
        """
        Test that metrics have required fields

        Arrange: Create service
        Act: Generate episode
        Assert: Metrics contain success, total_steps, max_steps
        """
        # Arrange
        service = MockVLAService()

        # Act
        episode_data = service.generate_episode(
            model_id="openvla-7b",
            instruction="Pick up the red cube",
            robot_id="widowx",
            scene_id="table",
        )

        # Assert
        metrics = episode_data["metrics"]
        assert "success" in metrics
        assert "total_steps" in metrics
        assert "max_steps" in metrics
        assert isinstance(metrics["success"], bool)
        assert isinstance(metrics["total_steps"], int)
        assert isinstance(metrics["max_steps"], int)
        assert metrics["total_steps"] <= metrics["max_steps"]

    def test_generate_episode_variable_length(self):
        """
        Test that episodes can have variable length (not always 50 steps)

        Arrange: Create service, generate multiple episodes
        Act: Check episode lengths
        Assert: Episodes have different lengths (simulating early termination)
        """
        # Arrange
        service = MockVLAService()
        lengths = []

        # Act - generate multiple episodes
        for i in range(10):
            episode_data = service.generate_episode(
                model_id="openvla-7b",
                instruction=f"Pick up cube {i}",
                robot_id="widowx",
                scene_id="table",
            )
            lengths.append(len(episode_data["actions"]))

        # Assert - should have some variation in lengths
        assert len(set(lengths)) > 1, "All episodes have same length, should vary"
        assert all(length <= 50 for length in lengths)
        assert all(length > 0 for length in lengths)

    def test_generate_episode_different_models_different_results(self):
        """
        Test that different models produce different episodes

        Arrange: Create service
        Act: Generate episodes with different model IDs
        Assert: Episodes are different
        """
        # Arrange
        service = MockVLAService()
        instruction = "Pick up the red cube"

        # Act
        episode_1 = service.generate_episode(
            model_id="openvla-7b",
            instruction=instruction,
            robot_id="widowx",
            scene_id="table",
        )
        episode_2 = service.generate_episode(
            model_id="octo-base",
            instruction=instruction,
            robot_id="widowx",
            scene_id="table",
        )

        # Assert - different models should produce different actions
        # (at least some actions should differ)
        actions_1 = episode_1["actions"]
        actions_2 = episode_2["actions"]

        # Check that at least one action differs
        different = False
        for a1, a2 in zip(actions_1, actions_2):
            if a1 != a2:
                different = True
                break

        assert different, "Different models should produce different actions"

    def test_generate_episode_duration_ms_positive(self):
        """
        Test that duration_ms is positive

        Arrange: Create service
        Act: Generate episode
        Assert: duration_ms > 0
        """
        # Arrange
        service = MockVLAService()

        # Act
        episode_data = service.generate_episode(
            model_id="openvla-7b",
            instruction="Pick up the red cube",
            robot_id="widowx",
            scene_id="table",
        )

        # Assert
        assert episode_data["duration_ms"] > 0
        assert isinstance(episode_data["duration_ms"], int)

    def test_generate_episode_states_match_actions_count(self):
        """
        Test that number of states matches number of actions

        Arrange: Create service
        Act: Generate episode
        Assert: len(states) == len(actions)
        """
        # Arrange
        service = MockVLAService()

        # Act
        episode_data = service.generate_episode(
            model_id="openvla-7b",
            instruction="Pick up the red cube",
            robot_id="widowx",
            scene_id="table",
        )

        # Assert
        assert len(episode_data["states"]) == len(episode_data["actions"])

    def test_generate_episode_metrics_total_steps_matches_actions(self):
        """
        Test that metrics.total_steps matches actual action count

        Arrange: Create service
        Act: Generate episode
        Assert: metrics.total_steps == len(actions)
        """
        # Arrange
        service = MockVLAService()

        # Act
        episode_data = service.generate_episode(
            model_id="openvla-7b",
            instruction="Pick up the red cube",
            robot_id="widowx",
            scene_id="table",
        )

        # Assert
        assert episode_data["metrics"]["total_steps"] == len(episode_data["actions"])
