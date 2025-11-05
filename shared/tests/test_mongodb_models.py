"""
Tests for MongoDB models (Episode, State, Metrics)

Following TDD principles with AAA pattern:
- Arrange: Setup test data
- Act: Execute operation
- Assert: Verify result
"""

from datetime import UTC, datetime

import pytest
from motor.motor_asyncio import AsyncIOMotorDatabase
from vlaarena_shared.mongodb_models import Episode, Metrics, State


@pytest.mark.asyncio
async def test_create_episode(mongodb_database: AsyncIOMotorDatabase):
    """Test creating an Episode document"""
    # Arrange
    episode_data = Episode(
        episode_id="ep_test_001",
        session_id="sess_test_001",
        battle_id="battle_test_001",
        turn_id="turn_test_001",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=[[0.1, 0.2, -0.3, 0.0, 0.5, 0.1, 0.0, 1.0]],
        states=[
            State(
                qpos=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04],
                qvel=[0.0] * 9,
                time=0.0,
            )
        ],
        metrics=Metrics(success=True, total_steps=1, max_steps=50),
        duration_ms=100,
    )

    # Act
    await episode_data.insert()

    # Assert
    assert episode_data.id is not None
    assert episode_data.episode_id == "ep_test_001"


@pytest.mark.asyncio
async def test_read_episode(mongodb_database: AsyncIOMotorDatabase):
    """Test reading an Episode document"""
    # Arrange
    episode = Episode(
        episode_id="ep_test_002",
        session_id="sess_test_002",
        battle_id="battle_test_002",
        turn_id="turn_test_002",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="right",
        model_id="octo-base",
        actions=[[0.15, 0.25, -0.25, 0.05, 0.45, 0.12, 0.0, 1.0]],
        states=[
            State(
                qpos=[0.01, -0.78, 0.01, -2.35, 0.01, 1.57, 0.79, 0.045, 0.045],
                qvel=[0.1] * 9,
                time=0.1,
            )
        ],
        metrics=Metrics(success=False, total_steps=1, max_steps=50),
        duration_ms=150,
    )
    await episode.insert()

    # Act
    retrieved = await Episode.find_one(Episode.episode_id == "ep_test_002")

    # Assert
    assert retrieved is not None
    assert retrieved.episode_id == "ep_test_002"
    assert retrieved.model_id == "octo-base"
    assert retrieved.side == "right"
    assert retrieved.metrics.success is False


@pytest.mark.asyncio
async def test_update_episode(mongodb_database: AsyncIOMotorDatabase):
    """Test updating an Episode document"""
    # Arrange
    episode = Episode(
        episode_id="ep_test_003",
        session_id="sess_test_003",
        battle_id="battle_test_003",
        turn_id="turn_test_003",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=[[0.1] * 8],
        states=[State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0)],
        metrics=Metrics(success=False, total_steps=1, max_steps=50),
        duration_ms=100,
    )
    await episode.insert()

    # Act
    episode.metrics.success = True
    episode.duration_ms = 200
    await episode.save()

    # Assert
    updated = await Episode.find_one(Episode.episode_id == "ep_test_003")
    assert updated.metrics.success is True
    assert updated.duration_ms == 200


@pytest.mark.asyncio
async def test_delete_episode(mongodb_database: AsyncIOMotorDatabase):
    """Test deleting an Episode document"""
    # Arrange
    episode = Episode(
        episode_id="ep_test_004",
        session_id="sess_test_004",
        battle_id="battle_test_004",
        turn_id="turn_test_004",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=[[0.1] * 8],
        states=[State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0)],
        metrics=Metrics(success=True, total_steps=1, max_steps=50),
        duration_ms=100,
    )
    await episode.insert()

    # Act
    await episode.delete()

    # Assert
    deleted = await Episode.find_one(Episode.episode_id == "ep_test_004")
    assert deleted is None


@pytest.mark.asyncio
async def test_query_by_battle_id(mongodb_database: AsyncIOMotorDatabase):
    """Test querying episodes by battle_id"""
    # Arrange
    battle_id = "battle_test_005"
    episode1 = Episode(
        episode_id="ep_test_005_left",
        session_id="sess_test_005",
        battle_id=battle_id,
        turn_id="turn_test_005",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=[[0.1] * 8],
        states=[State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0)],
        metrics=Metrics(success=True, total_steps=1, max_steps=50),
        duration_ms=100,
    )
    episode2 = Episode(
        episode_id="ep_test_005_right",
        session_id="sess_test_005",
        battle_id=battle_id,
        turn_id="turn_test_005",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=1,
        side="right",
        model_id="octo-base",
        actions=[[0.2] * 8],
        states=[State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0)],
        metrics=Metrics(success=False, total_steps=1, max_steps=50),
        duration_ms=150,
    )
    await episode1.insert()
    await episode2.insert()

    # Act
    episodes = await Episode.find(Episode.battle_id == battle_id).to_list()

    # Assert
    assert len(episodes) == 2
    episode_ids = {ep.episode_id for ep in episodes}
    assert "ep_test_005_left" in episode_ids
    assert "ep_test_005_right" in episode_ids


@pytest.mark.asyncio
async def test_episode_with_multiple_states(mongodb_database: AsyncIOMotorDatabase):
    """Test episode with multiple states and actions"""
    # Arrange
    states = [
        State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0),
        State(qpos=[0.01] * 9, qvel=[0.1] * 9, time=0.1),
        State(qpos=[0.02] * 9, qvel=[0.2] * 9, time=0.2),
    ]
    actions = [[0.1] * 8, [0.15] * 8, [0.2] * 8]

    episode = Episode(
        episode_id="ep_test_006",
        session_id="sess_test_006",
        battle_id="battle_test_006",
        turn_id="turn_test_006",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=actions,
        states=states,
        metrics=Metrics(success=True, total_steps=3, max_steps=50),
        duration_ms=300,
    )

    # Act
    await episode.insert()
    retrieved = await Episode.find_one(Episode.episode_id == "ep_test_006")

    # Assert
    assert len(retrieved.states) == 3
    assert len(retrieved.actions) == 3
    assert retrieved.states[0].time == 0.0
    assert retrieved.states[1].time == 0.1
    assert retrieved.states[2].time == 0.2
    assert retrieved.metrics.total_steps == 3


@pytest.mark.asyncio
async def test_episode_metrics_fields(mongodb_database: AsyncIOMotorDatabase):
    """Test all metrics fields"""
    # Arrange
    metrics = Metrics(
        success=True,
        total_steps=35,
        max_steps=50,
        terminated_early=False,
        final_distance_to_goal=0.05,
        collision_count=0,
        gripper_opened_at_step=25,
    )

    episode = Episode(
        episode_id="ep_test_007",
        session_id="sess_test_007",
        battle_id="battle_test_007",
        turn_id="turn_test_007",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=[[0.1] * 8],
        states=[State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0)],
        metrics=metrics,
        duration_ms=3500,
    )

    # Act
    await episode.insert()
    retrieved = await Episode.find_one(Episode.episode_id == "ep_test_007")

    # Assert
    assert retrieved.metrics.success is True
    assert retrieved.metrics.total_steps == 35
    assert retrieved.metrics.max_steps == 50
    assert retrieved.metrics.terminated_early is False
    assert retrieved.metrics.final_distance_to_goal == 0.05
    assert retrieved.metrics.collision_count == 0
    assert retrieved.metrics.gripper_opened_at_step == 25


@pytest.mark.asyncio
async def test_created_at_timestamp(mongodb_database: AsyncIOMotorDatabase):
    """Test created_at timestamp is set automatically"""
    # Arrange
    before = datetime.now(UTC)
    episode = Episode(
        episode_id="ep_test_008",
        session_id="sess_test_008",
        battle_id="battle_test_008",
        turn_id="turn_test_008",
        battle_seq_in_session=1,
        turn_seq=1,
        seq_in_turn=0,
        side="left",
        model_id="openvla-7b",
        actions=[[0.1] * 8],
        states=[State(qpos=[0.0] * 9, qvel=[0.0] * 9, time=0.0)],
        metrics=Metrics(success=True, total_steps=1, max_steps=50),
        duration_ms=100,
    )

    # Act
    await episode.insert()
    after = datetime.now(UTC)

    # Assert
    assert episode.created_at is not None
    assert before <= episode.created_at <= after
