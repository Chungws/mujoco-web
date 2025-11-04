"""
Tests for battle API endpoints
"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, patch

from fastapi.testclient import TestClient

from vlaarena_shared.models import Battle


def test_add_follow_up_message_success(client: TestClient):
    """
    Test adding follow-up message to existing battle

    Scenario:
    1. User has existing battle with 1 turn (user message + 2 assistant responses)
    2. User submits follow-up prompt
    3. System retrieves session-wide conversation history from Turn/Message tables
    4. System calls LLM APIs with full message history (OpenAI chat format)
    5. System creates new Turn and Message records
    6. Returns anonymous responses with message_count
    """
    # Arrange: Create session with a battle that has one turn

    battle_id = "battle_xyz789"
    follow_up_prompt = "What about its population?"

    # Create session, battle, turn, and messages via API endpoint
    # (This simulates a real session with existing battle)
    create_response = client.post(
        "/api/sessions",
        json={"prompt": "What is the capital of France?", "user_id": "user_test123"},
    )
    assert create_response.status_code == 201
    session_data = create_response.json()
    session_data["session_id"]
    battle_id = session_data["battle_id"]

    # Act: Add follow-up message (uses MockLLMClient automatically from conftest)
    response = client.post(f"/api/battles/{battle_id}/messages", json={"prompt": follow_up_prompt})

    # Assert
    assert response.status_code == 201
    data = response.json()

    # Check response structure (FollowUpResponse schema)
    assert "battle_id" in data
    assert data["battle_id"] == battle_id
    assert "message_id" in data
    assert data["message_id"] == "msg_2"  # Second user message
    assert "responses" in data
    assert len(data["responses"]) == 2
    assert "message_count" in data
    assert data["message_count"] == 2  # 2 user messages total (initial + follow-up)
    assert "max_messages" in data
    assert data["max_messages"] == 6

    # Check response format (MockLLMClient provides generic responses)
    left_response = data["responses"][0]
    right_response = data["responses"][1]

    assert left_response["position"] == "left"
    assert "text" in left_response
    assert left_response["latency_ms"] > 0

    assert right_response["position"] == "right"
    assert "text" in right_response
    assert right_response["latency_ms"] > 0


def test_add_follow_up_message_battle_not_found(client: TestClient):
    """
    Test adding follow-up message to non-existent battle fails

    Scenario:
    1. User provides invalid battle_id
    2. Returns 404 not found error
    """
    # Arrange
    battle_id = "battle_nonexistent"
    prompt = "Follow-up question"

    with patch(
        "vlaarena_backend.services.session_service.BattleRepository"
    ) as mock_battle_repo_class:
        # Mock battle not found
        mock_battle_repo = AsyncMock()
        mock_battle_repo.get_by_battle_id.return_value = None
        mock_battle_repo_class.return_value = mock_battle_repo

        # Act
        response = client.post(f"/api/battles/{battle_id}/messages", json={"prompt": prompt})

    # Assert
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()


def test_add_follow_up_message_battle_already_voted(client: TestClient):
    """
    Test adding follow-up message to voted battle fails

    Scenario:
    1. User tries to add message to battle that has been voted
    2. Returns 400 bad request error
    """
    # Arrange
    battle_id = "battle_voted123"
    prompt = "Follow-up question"

    mock_battle = Battle(
        id=1,
        battle_id=battle_id,
        session_id="session_abc123",
        left_model_id="gpt-4o-mini",
        right_model_id="llama-3-1-8b",
        conversation=[],
        status="voted",  # Battle already voted
        created_at=datetime.now(UTC),
        updated_at=datetime.now(UTC),
    )

    with patch(
        "vlaarena_backend.services.session_service.BattleRepository"
    ) as mock_battle_repo_class:
        # Mock battle found but voted
        mock_battle_repo = AsyncMock()
        mock_battle_repo.get_by_battle_id.return_value = mock_battle
        mock_battle_repo_class.return_value = mock_battle_repo

        # Act
        response = client.post(f"/api/battles/{battle_id}/messages", json={"prompt": prompt})

    # Assert
    assert response.status_code == 400
    assert (
        "voted" in response.json()["detail"].lower()
        or "cannot add" in response.json()["detail"].lower()
    )


def test_vote_on_battle_success(client: TestClient):
    """
    Test voting on battle successfully

    Scenario:
    1. User submits vote on ongoing battle
    2. System creates vote record with denormalized model IDs
    3. System updates battle.status to 'voted'
    4. System updates session.last_active_at
    5. Returns vote confirmation with revealed model identities
    """
    # Arrange
    battle_id = "battle_abc123"
    vote_choice = "left_better"

    mock_battle = Battle(
        id=1,
        battle_id=battle_id,
        session_id="session_xyz789",
        left_model_id="gpt-4o-mini",
        right_model_id="llama-3-1-8b",
        conversation=[
            {
                "role": "user",
                "content": "What is Python?",
                "timestamp": "2025-10-21T10:00:00Z",
            },
            {
                "role": "assistant",
                "model_id": "gpt-4o-mini",
                "position": "left",
                "content": "Python is a programming language.",
                "latency_ms": 250,
                "timestamp": "2025-10-21T10:00:01Z",
            },
            {
                "role": "assistant",
                "model_id": "llama-3-1-8b",
                "position": "right",
                "content": "Python is a high-level language.",
                "latency_ms": 280,
                "timestamp": "2025-10-21T10:00:01Z",
            },
        ],
        status="ongoing",
        created_at=datetime.now(UTC),
        updated_at=datetime.now(UTC),
    )

    with (
        patch(
            "vlaarena_backend.services.session_service.BattleRepository"
        ) as mock_battle_repo_class,
        patch(
            "vlaarena_backend.services.session_service.SessionRepository"
        ) as mock_session_repo_class,
        patch("vlaarena_backend.services.session_service.VoteRepository") as mock_vote_repo_class,
    ):
        # Mock repositories
        mock_battle_repo = AsyncMock()
        mock_session_repo = AsyncMock()
        mock_vote_repo = AsyncMock()

        mock_battle_repo.get_by_battle_id.return_value = mock_battle
        mock_battle_repo.update_status.return_value = None
        mock_session_repo.update_last_active_at.return_value = None
        mock_vote_repo.create.return_value = None

        mock_battle_repo_class.return_value = mock_battle_repo
        mock_session_repo_class.return_value = mock_session_repo
        mock_vote_repo_class.return_value = mock_vote_repo

        # Act
        response = client.post(
            f"/api/battles/{battle_id}/vote",
            json={"vote": vote_choice},
        )

    # Assert
    assert response.status_code == 200
    data = response.json()
    assert data["battle_id"] == battle_id
    assert data["vote"] == vote_choice
    assert data["revealed_models"]["left"] == "gpt-4o-mini"
    assert data["revealed_models"]["right"] == "llama-3-1-8b"


def test_vote_on_battle_not_found(client: TestClient):
    """
    Test voting on non-existent battle fails

    Scenario:
    1. User submits vote on non-existent battle
    2. Returns 404 not found error
    """
    # Arrange
    battle_id = "battle_nonexistent"
    vote_choice = "left_better"

    with patch(
        "vlaarena_backend.services.session_service.BattleRepository"
    ) as mock_battle_repo_class:
        # Mock battle not found
        mock_battle_repo = AsyncMock()
        mock_battle_repo.get_by_battle_id.return_value = None
        mock_battle_repo_class.return_value = mock_battle_repo

        # Act
        response = client.post(
            f"/api/battles/{battle_id}/vote",
            json={"vote": vote_choice},
        )

    # Assert
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()


def test_vote_on_battle_already_voted(client: TestClient):
    """
    Test voting on already-voted battle fails

    Scenario:
    1. User tries to vote on battle that has already been voted
    2. Returns 400 bad request error
    """
    # Arrange
    battle_id = "battle_already_voted"
    vote_choice = "left_better"

    mock_battle = Battle(
        id=1,
        battle_id=battle_id,
        session_id="session_xyz789",
        left_model_id="gpt-4o-mini",
        right_model_id="llama-3-1-8b",
        conversation=[],
        status="voted",  # Already voted
        created_at=datetime.now(UTC),
        updated_at=datetime.now(UTC),
    )

    with patch(
        "vlaarena_backend.services.session_service.BattleRepository"
    ) as mock_battle_repo_class:
        # Mock battle found but already voted
        mock_battle_repo = AsyncMock()
        mock_battle_repo.get_by_battle_id.return_value = mock_battle
        mock_battle_repo_class.return_value = mock_battle_repo

        # Act
        response = client.post(
            f"/api/battles/{battle_id}/vote",
            json={"vote": vote_choice},
        )

    # Assert
    assert response.status_code == 400
    detail = response.json()["detail"].lower()
    assert ("already" in detail and "voted" in detail) or "been voted" in detail
