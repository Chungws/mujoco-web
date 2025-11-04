"""
Tests for session and battle API endpoints
"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, Mock, patch

from fastapi.testclient import TestClient

from vlaarena_backend.services.llm_client import LLMResponse


def test_create_session_success(client: TestClient):
    """
    Test successful session creation with first battle

    Scenario:
    1. User submits initial prompt
    2. System creates session
    3. System selects 2 random models
    4. System calls both LLMs in parallel
    5. System creates battle with conversation
    6. Returns anonymous responses
    """
    # Arrange
    prompt = "What is the capital of France?"

    # Mock model configs
    from vlaarena_backend.services.model_service import ModelConfig

    mock_model_a = ModelConfig(
        {
            "id": "llama-3-1-8b",
            "name": "Llama 3.1 8B",
            "model": "llama3.1:8b",
            "base_url": "http://localhost:11434/v1",
            "api_key_env": None,
            "organization": "Meta",
            "license": "open-source",
            "status": "active",
        }
    )

    mock_model_b = ModelConfig(
        {
            "id": "qwen-2-5-7b",
            "name": "Qwen 2.5 7B",
            "model": "qwen2.5:7b",
            "base_url": "http://localhost:11434/v1",
            "api_key_env": None,
            "organization": "Alibaba",
            "license": "open-source",
            "status": "active",
        }
    )

    # Mock LLM responses
    mock_left_response = LLMResponse(
        content="The capital of France is Paris.",
        latency_ms=250,
        model_id="llama-3-1-8b",
    )
    mock_right_response = LLMResponse(
        content="Paris is the capital city of France.",
        latency_ms=300,
        model_id="qwen-2-5-7b",
    )

    with (
        patch(
            "vlaarena_backend.services.session_service.get_model_service"
        ) as mock_get_model_service,
        patch("vlaarena_backend.services.session_service.get_llm_client") as mock_get_client,
    ):
        # Mock model service (sync, not async)
        mock_model_service = Mock()
        mock_model_service.select_models_for_battle.return_value = (mock_model_a, mock_model_b)
        mock_get_model_service.return_value = mock_model_service

        # Mock LLM client
        mock_client = AsyncMock()
        mock_client.chat_completion.side_effect = [
            mock_left_response,
            mock_right_response,
        ]
        mock_get_client.return_value = mock_client

        # Act
        response = client.post("/api/sessions", json={"prompt": prompt})

    # Assert
    assert response.status_code == 201
    data = response.json()

    # Check response structure
    assert "session_id" in data
    assert "battle_id" in data
    assert "message_id" in data
    assert data["message_id"] == "msg_1"  # First message
    assert "responses" in data
    assert len(data["responses"]) == 2

    # Check response format
    left_response = data["responses"][0]
    right_response = data["responses"][1]

    assert left_response["position"] == "left"
    assert "text" in left_response
    assert "latency_ms" in left_response
    assert len(left_response["text"]) > 0
    assert left_response["latency_ms"] == 250  # Mock value

    assert right_response["position"] == "right"
    assert "text" in right_response
    assert "latency_ms" in right_response
    assert len(right_response["text"]) > 0
    assert right_response["latency_ms"] == 300  # Mock value

    # Verify responses match mock data
    assert left_response["text"] == "The capital of France is Paris."
    assert right_response["text"] == "Paris is the capital city of France."


def test_create_session_empty_prompt(client: TestClient):
    """
    Test session creation with empty prompt fails

    Scenario:
    1. User submits empty prompt
    2. Returns 422 validation error
    """
    # Arrange
    prompt = ""

    # Act
    response = client.post("/api/sessions", json={"prompt": prompt})

    # Assert
    assert response.status_code == 422


def test_create_session_too_long_prompt(client: TestClient):
    """
    Test session creation with excessively long prompt fails

    Scenario:
    1. User submits prompt > 10000 characters
    2. Returns 422 validation error
    """
    # Arrange
    prompt = "A" * 10001

    # Act
    response = client.post("/api/sessions", json={"prompt": prompt})

    # Assert
    assert response.status_code == 422


def test_create_new_battle_in_session_success(client: TestClient):
    """
    Test creating a new battle in existing session

    Scenario:
    1. User has existing session with one battle
    2. User submits new prompt for second battle
    3. System selects 2 NEW random models (different from first battle)
    4. System calls both LLMs in parallel (using MockLLMClient)
    5. System creates new battle in same session with session-wide history
    6. System updates session.last_active_at
    7. Returns anonymous responses
    """
    # Arrange: Create first session/battle via API
    create_response = client.post(
        "/api/sessions",
        json={"prompt": "What is the capital of France?", "user_id": "user_test123"},
    )
    assert create_response.status_code == 201
    first_data = create_response.json()
    session_id = first_data["session_id"]

    # Act: Create second battle in the same session
    prompt = "Tell me about Python programming"
    response = client.post(f"/api/sessions/{session_id}/battles", json={"prompt": prompt})

    # Assert
    assert response.status_code == 201
    data = response.json()

    # Check response structure (BattleResponse schema)
    assert "battle_id" in data
    assert data["battle_id"] != first_data["battle_id"]  # Different battle
    assert "message_id" in data
    assert data["message_id"] == "msg_1"  # First message of new battle
    assert "responses" in data
    assert len(data["responses"]) == 2

    # Check response format (MockLLMClient provides generic responses)
    left_response = data["responses"][0]
    right_response = data["responses"][1]

    assert left_response["position"] == "left"
    assert "text" in left_response
    assert left_response["latency_ms"] > 0

    assert right_response["position"] == "right"
    assert "text" in right_response
    assert right_response["latency_ms"] > 0


def test_create_new_battle_session_not_found(client: TestClient):
    """
    Test creating battle with non-existent session ID fails

    Scenario:
    1. User provides invalid session_id
    2. Returns 404 not found error
    """
    # Arrange
    session_id = "session_nonexistent"
    prompt = "Test prompt"

    with patch(
        "vlaarena_backend.services.session_service.SessionRepository"
    ) as mock_session_repo_class:
        # Mock session not found
        mock_session_repo = AsyncMock()
        mock_session_repo.get_by_session_id.return_value = None
        mock_session_repo_class.return_value = mock_session_repo

        # Act
        response = client.post(f"/api/sessions/{session_id}/battles", json={"prompt": prompt})

    # Assert
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()


def test_get_sessions_empty_list(client: TestClient):
    """
    Test GET /api/sessions with no sessions for user

    Scenario:
    1. User requests session list
    2. No sessions exist for this user_id
    3. Returns empty list with total=0
    """
    # Arrange
    user_id = "user_abc123"

    with patch(
        "vlaarena_backend.services.session_service.SessionRepository"
    ) as mock_session_repo_class:
        # Mock empty session list
        mock_session_repo = AsyncMock()
        mock_session_repo.get_by_user_id.return_value = []
        mock_session_repo.count_by_user_id.return_value = 0
        mock_session_repo_class.return_value = mock_session_repo

        # Act
        response = client.get(f"/api/sessions?user_id={user_id}")

    # Assert
    assert response.status_code == 200
    data = response.json()

    assert "sessions" in data
    assert "total" in data
    assert data["sessions"] == []
    assert data["total"] == 0


def test_get_sessions_with_multiple_sessions(client: TestClient):
    """
    Test GET /api/sessions with multiple sessions

    Scenario:
    1. User has 3 sessions
    2. Sessions are ordered by last_active_at DESC
    3. Returns all sessions with correct structure
    """
    # Arrange
    user_id = "user_abc123"

    from vlaarena_shared.models import Session

    mock_sessions = [
        Session(
            id=1,
            session_id="session_001",
            title="What is Python?",
            user_id=user_id,
            created_at=datetime(2025, 1, 20, 10, 0, 0, tzinfo=UTC),
            last_active_at=datetime(2025, 1, 20, 15, 0, 0, tzinfo=UTC),
        ),
        Session(
            id=2,
            session_id="session_002",
            title="How to learn AI?",
            user_id=user_id,
            created_at=datetime(2025, 1, 19, 10, 0, 0, tzinfo=UTC),
            last_active_at=datetime(2025, 1, 20, 14, 0, 0, tzinfo=UTC),
        ),
        Session(
            id=3,
            session_id="session_003",
            title="Best practices for REST APIs?",
            user_id=user_id,
            created_at=datetime(2025, 1, 18, 10, 0, 0, tzinfo=UTC),
            last_active_at=datetime(2025, 1, 19, 12, 0, 0, tzinfo=UTC),
        ),
    ]

    with patch(
        "vlaarena_backend.services.session_service.SessionRepository"
    ) as mock_session_repo_class:
        # Mock session list
        mock_session_repo = AsyncMock()
        mock_session_repo.get_by_user_id.return_value = mock_sessions
        mock_session_repo.count_by_user_id.return_value = 3
        mock_session_repo_class.return_value = mock_session_repo

        # Act
        response = client.get(f"/api/sessions?user_id={user_id}&limit=50&offset=0")

    # Assert
    assert response.status_code == 200
    data = response.json()

    assert "sessions" in data
    assert "total" in data
    assert len(data["sessions"]) == 3
    assert data["total"] == 3

    # Check first session (most recent)
    first_session = data["sessions"][0]
    assert first_session["session_id"] == "session_001"
    assert first_session["title"] == "What is Python?"
    assert "created_at" in first_session
    assert "last_active_at" in first_session


def test_get_sessions_with_pagination(client: TestClient):
    """
    Test GET /api/sessions with limit and offset pagination

    Scenario:
    1. User has multiple sessions
    2. Request with limit=2, offset=1
    3. Returns paginated results
    """
    # Arrange
    user_id = "user_abc123"

    from vlaarena_shared.models import Session

    mock_sessions = [
        Session(
            id=2,
            session_id="session_002",
            title="Second session",
            user_id=user_id,
            created_at=datetime(2025, 1, 19, 10, 0, 0, tzinfo=UTC),
            last_active_at=datetime(2025, 1, 20, 14, 0, 0, tzinfo=UTC),
        ),
        Session(
            id=3,
            session_id="session_003",
            title="Third session",
            user_id=user_id,
            created_at=datetime(2025, 1, 18, 10, 0, 0, tzinfo=UTC),
            last_active_at=datetime(2025, 1, 19, 12, 0, 0, tzinfo=UTC),
        ),
    ]

    with patch(
        "vlaarena_backend.services.session_service.SessionRepository"
    ) as mock_session_repo_class:
        # Mock paginated session list
        mock_session_repo = AsyncMock()
        mock_session_repo.get_by_user_id.return_value = mock_sessions
        mock_session_repo.count_by_user_id.return_value = 5  # Total sessions
        mock_session_repo_class.return_value = mock_session_repo

        # Act
        response = client.get(f"/api/sessions?user_id={user_id}&limit=2&offset=1")

    # Assert
    assert response.status_code == 200
    data = response.json()

    assert len(data["sessions"]) == 2
    assert data["total"] == 5
    assert data["sessions"][0]["session_id"] == "session_002"
    assert data["sessions"][1]["session_id"] == "session_003"


def test_get_session_battles_empty(client: TestClient):
    """
    Test GET /api/sessions/{session_id}/battles with no battles

    Scenario:
    1. Create session via API (which creates one battle)
    2. Query battles endpoint
    3. Returns battle list with one battle

    Note: Cannot have session without battles in current implementation
    since POST /api/sessions always creates first battle
    """
    # Arrange: Create session with first battle
    create_response = client.post(
        "/api/sessions",
        json={"prompt": "Test prompt", "user_id": "user_test123"},
    )
    assert create_response.status_code == 201
    session_data = create_response.json()
    session_id = session_data["session_id"]

    # Act
    response = client.get(f"/api/sessions/{session_id}/battles")

    # Assert
    assert response.status_code == 200
    data = response.json()

    assert "session_id" in data
    assert "battles" in data
    assert data["session_id"] == session_id
    # Note: Should have 1 battle, not 0, since session creation includes first battle
    assert len(data["battles"]) == 1
    assert data["battles"][0]["status"] == "ongoing"


def test_get_session_battles_with_votes(client: TestClient):
    """
    Test GET /api/sessions/{session_id}/battles with voted battles

    Scenario:
    1. Create session with first battle
    2. Vote on first battle
    3. Create second battle in same session
    4. Query battles endpoint
    5. Returns battles with vote information
    """
    # Arrange: Create first session/battle
    create_response = client.post(
        "/api/sessions",
        json={"prompt": "Hello", "user_id": "user_test123"},
    )
    assert create_response.status_code == 201
    first_data = create_response.json()
    session_id = first_data["session_id"]
    first_battle_id = first_data["battle_id"]

    # Vote on first battle
    vote_response = client.post(
        f"/api/battles/{first_battle_id}/vote",
        json={"vote": "left_better"},
    )
    assert vote_response.status_code == 200

    # Create second battle in same session
    second_battle_response = client.post(
        f"/api/sessions/{session_id}/battles",
        json={"prompt": "What is AI?"},
    )
    assert second_battle_response.status_code == 201
    second_battle_id = second_battle_response.json()["battle_id"]

    # Act
    response = client.get(f"/api/sessions/{session_id}/battles")

    # Assert
    assert response.status_code == 200
    data = response.json()

    assert data["session_id"] == session_id
    assert len(data["battles"]) == 2

    # Check first battle (voted)
    first_battle = data["battles"][0]
    assert first_battle["battle_id"] == first_battle_id
    assert first_battle["status"] == "voted"
    assert first_battle["vote"] == "left_better"
    assert "left_model_id" in first_battle
    assert "right_model_id" in first_battle
    assert "conversation" in first_battle
    assert len(first_battle["conversation"]) > 0  # Should have user + 2 assistant messages

    # Check second battle (ongoing)
    second_battle = data["battles"][1]
    assert second_battle["battle_id"] == second_battle_id
    assert second_battle["status"] == "ongoing"
    assert second_battle["vote"] is None
    assert "left_model_id" in second_battle
    assert "right_model_id" in second_battle
    assert "conversation" in second_battle
    assert len(second_battle["conversation"]) > 0


def test_get_session_battles_session_not_found(client: TestClient):
    """
    Test GET /api/sessions/{session_id}/battles with non-existent session

    Scenario:
    1. Session does not exist
    2. Returns 404 error
    """
    # Arrange
    session_id = "session_nonexistent"

    with patch(
        "vlaarena_backend.services.session_service.SessionRepository"
    ) as mock_session_repo_class:
        # Mock session not found
        mock_session_repo = AsyncMock()
        mock_session_repo.get_by_session_id.return_value = None
        mock_session_repo_class.return_value = mock_session_repo

        # Act
        response = client.get(f"/api/sessions/{session_id}/battles")

    # Assert
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()
