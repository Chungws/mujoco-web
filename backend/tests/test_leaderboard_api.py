"""
Tests for leaderboard API endpoints
"""

from datetime import UTC, datetime
from unittest.mock import patch

from fastapi.testclient import TestClient

from vlaarena_shared.schemas import (
    LeaderboardMetadata,
    LeaderboardResponse,
    ModelStatsResponse,
)


def test_get_leaderboard_success(client: TestClient):
    """
    Test successful leaderboard retrieval

    Scenario:
    1. Database has 5 models (3 with vote_count >= 5, 2 with vote_count < 5)
    2. User requests leaderboard
    3. Returns sorted leaderboard with only models having vote_count >= 5
    4. Models sorted by elo_score descending
    5. Includes metadata (total_models, total_votes, last_updated)
    """
    # Arrange - Mock leaderboard response
    mock_leaderboard = LeaderboardResponse(
        leaderboard=[
            ModelStatsResponse(
                rank=1,
                model_id="gpt-4o",
                model_name="gpt-4o",
                elo_score=1654,
                elo_ci=12.3,
                vote_count=1234,
                win_rate=0.68,
                organization="OpenAI",
                license="proprietary",
            ),
            ModelStatsResponse(
                rank=2,
                model_id="claude-3-5-sonnet",
                model_name="claude-3-5-sonnet",
                elo_score=1632,
                elo_ci=15.7,
                vote_count=987,
                win_rate=0.64,
                organization="Anthropic",
                license="proprietary",
            ),
            ModelStatsResponse(
                rank=3,
                model_id="llama-3-1-70b",
                model_name="llama-3-1-70b",
                elo_score=1580,
                elo_ci=18.9,
                vote_count=550,
                win_rate=0.58,
                organization="Meta",
                license="open-source",
            ),
        ],
        metadata=LeaderboardMetadata(
            total_models=3,
            total_votes=2771,
            last_updated=datetime.now(UTC),
        ),
    )

    with patch(
        "vlaarena_backend.services.leaderboard_service.LeaderboardService.get_leaderboard"
    ) as mock_get_leaderboard:
        # Mock service method
        mock_get_leaderboard.return_value = mock_leaderboard

        # Act
        response = client.get("/api/leaderboard")

    # Assert
    assert response.status_code == 200
    data = response.json()

    # Check response structure
    assert "leaderboard" in data
    assert "metadata" in data

    # Check leaderboard entries
    leaderboard = data["leaderboard"]
    assert len(leaderboard) == 3  # Only models with vote_count >= 5

    # Check first entry (highest ELO)
    first = leaderboard[0]
    assert first["rank"] == 1
    assert first["model_id"] == "gpt-4o"
    assert first["model_name"] == "gpt-4o"  # Should match model_id for now
    assert first["elo_score"] == 1654
    assert first["elo_ci"] == 12.3
    assert first["vote_count"] == 1234
    assert first["win_rate"] == 0.68
    assert first["organization"] == "OpenAI"
    assert first["license"] == "proprietary"

    # Check second entry
    second = leaderboard[1]
    assert second["rank"] == 2
    assert second["model_id"] == "claude-3-5-sonnet"
    assert second["elo_score"] == 1632

    # Check third entry
    third = leaderboard[2]
    assert third["rank"] == 3
    assert third["model_id"] == "llama-3-1-70b"
    assert third["elo_score"] == 1580

    # Check metadata
    metadata = data["metadata"]
    assert metadata["total_models"] == 3  # Only models with vote_count >= 5
    assert metadata["total_votes"] == 2771  # Sum of vote_counts
    assert "last_updated" in metadata


def test_get_leaderboard_empty_database(client: TestClient):
    """
    Test leaderboard retrieval with empty database

    Scenario:
    1. Database has no model_stats
    2. User requests leaderboard
    3. Returns empty leaderboard with zero metadata
    """
    # Arrange - Mock empty leaderboard response
    mock_leaderboard = LeaderboardResponse(
        leaderboard=[],
        metadata=LeaderboardMetadata(
            total_models=0,
            total_votes=0,
            last_updated=datetime.now(UTC),
        ),
    )

    with patch(
        "vlaarena_backend.services.leaderboard_service.LeaderboardService.get_leaderboard"
    ) as mock_get_leaderboard:
        mock_get_leaderboard.return_value = mock_leaderboard

        # Act
        response = client.get("/api/leaderboard")

    # Assert
    assert response.status_code == 200
    data = response.json()

    # Check response structure
    assert "leaderboard" in data
    assert "metadata" in data

    # Check empty leaderboard
    assert len(data["leaderboard"]) == 0

    # Check metadata
    metadata = data["metadata"]
    assert metadata["total_models"] == 0
    assert metadata["total_votes"] == 0
    assert "last_updated" in metadata


def test_get_leaderboard_all_below_threshold(client: TestClient):
    """
    Test leaderboard with all models below minimum vote threshold

    Scenario:
    1. Database has 2 models, both with vote_count < 5
    2. Repository filters them out
    3. Returns empty leaderboard (all models filtered)
    """
    # Arrange - Mock empty leaderboard response (all filtered)
    mock_leaderboard = LeaderboardResponse(
        leaderboard=[],
        metadata=LeaderboardMetadata(
            total_models=0,
            total_votes=0,
            last_updated=datetime.now(UTC),
        ),
    )

    with patch(
        "vlaarena_backend.services.leaderboard_service.LeaderboardService.get_leaderboard"
    ) as mock_get_leaderboard:
        mock_get_leaderboard.return_value = mock_leaderboard

        # Act
        response = client.get("/api/leaderboard")

    # Assert
    assert response.status_code == 200
    data = response.json()

    # Check empty leaderboard
    assert len(data["leaderboard"]) == 0

    # Check metadata
    metadata = data["metadata"]
    assert metadata["total_models"] == 0
    assert metadata["total_votes"] == 0


def test_get_leaderboard_sorting_by_elo(client: TestClient):
    """
    Test leaderboard is sorted by elo_score descending

    Scenario:
    1. Database has models with different ELO scores
    2. Repository returns them sorted by elo_score desc
    3. Ranks assigned correctly (1, 2, 3, ...)
    """
    # Arrange - Mock leaderboard response with sorted models
    mock_leaderboard = LeaderboardResponse(
        leaderboard=[
            ModelStatsResponse(
                rank=1,
                model_id="gpt-4o",
                model_name="gpt-4o",
                elo_score=1654,
                elo_ci=12.3,
                vote_count=1234,
                win_rate=0.68,
                organization="OpenAI",
                license="proprietary",
            ),
            ModelStatsResponse(
                rank=2,
                model_id="claude-3-5-sonnet",
                model_name="claude-3-5-sonnet",
                elo_score=1632,
                elo_ci=15.7,
                vote_count=987,
                win_rate=0.64,
                organization="Anthropic",
                license="proprietary",
            ),
            ModelStatsResponse(
                rank=3,
                model_id="llama-3-1-70b",
                model_name="llama-3-1-70b",
                elo_score=1580,
                elo_ci=18.9,
                vote_count=550,
                win_rate=0.58,
                organization="Meta",
                license="open-source",
            ),
        ],
        metadata=LeaderboardMetadata(
            total_models=3,
            total_votes=2771,
            last_updated=datetime.now(UTC),
        ),
    )

    with patch(
        "vlaarena_backend.services.leaderboard_service.LeaderboardService.get_leaderboard"
    ) as mock_get_leaderboard:
        mock_get_leaderboard.return_value = mock_leaderboard

        # Act
        response = client.get("/api/leaderboard")

    # Assert
    assert response.status_code == 200
    data = response.json()
    leaderboard = data["leaderboard"]

    # Check sorting order (descending by elo_score)
    assert len(leaderboard) == 3
    assert leaderboard[0]["elo_score"] == 1654  # gpt-4o
    assert leaderboard[1]["elo_score"] == 1632  # claude-3-5-sonnet
    assert leaderboard[2]["elo_score"] == 1580  # llama-3-1-70b

    # Check ranks
    assert leaderboard[0]["rank"] == 1
    assert leaderboard[1]["rank"] == 2
    assert leaderboard[2]["rank"] == 3


def test_get_leaderboard_metadata_accuracy(client: TestClient):
    """
    Test leaderboard metadata is calculated correctly

    Scenario:
    1. Database has models with vote_count >= 5
    2. Metadata reflects correct totals
    """
    # Arrange - Mock leaderboard response
    mock_leaderboard = LeaderboardResponse(
        leaderboard=[
            ModelStatsResponse(
                rank=1,
                model_id="gpt-4o",
                model_name="gpt-4o",
                elo_score=1654,
                elo_ci=12.3,
                vote_count=1234,
                win_rate=0.68,
                organization="OpenAI",
                license="proprietary",
            ),
            ModelStatsResponse(
                rank=2,
                model_id="claude-3-5-sonnet",
                model_name="claude-3-5-sonnet",
                elo_score=1632,
                elo_ci=15.7,
                vote_count=987,
                win_rate=0.64,
                organization="Anthropic",
                license="proprietary",
            ),
            ModelStatsResponse(
                rank=3,
                model_id="llama-3-1-70b",
                model_name="llama-3-1-70b",
                elo_score=1580,
                elo_ci=18.9,
                vote_count=550,
                win_rate=0.58,
                organization="Meta",
                license="open-source",
            ),
        ],
        metadata=LeaderboardMetadata(
            total_models=3,
            total_votes=2771,
            last_updated=datetime.now(UTC),
        ),
    )

    with patch(
        "vlaarena_backend.services.leaderboard_service.LeaderboardService.get_leaderboard"
    ) as mock_get_leaderboard:
        mock_get_leaderboard.return_value = mock_leaderboard

        # Act
        response = client.get("/api/leaderboard")

    # Assert
    assert response.status_code == 200
    data = response.json()
    metadata = data["metadata"]

    # Check metadata (only models with vote_count >= 5)
    assert metadata["total_models"] == 3
    assert metadata["total_votes"] == 2771
    assert "last_updated" in metadata
    # Verify last_updated is a valid datetime string
    datetime.fromisoformat(metadata["last_updated"].replace("Z", "+00:00"))
