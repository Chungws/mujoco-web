"""
Leaderboard business logic service
"""

from datetime import UTC, datetime
from typing import List

from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select

from vlaarena_shared.models import ModelStats, WorkerStatus
from vlaarena_shared.schemas import (
    LeaderboardMetadata,
    LeaderboardResponse,
    ModelStatsResponse,
)

from ..repositories.model_stats_repository import ModelStatsRepository


class LeaderboardService:
    """Service for leaderboard operations"""

    def __init__(self, db: AsyncSession):
        self.db = db
        self.model_stats_repo = ModelStatsRepository(db)

    async def get_leaderboard(self, min_vote_count: int = 5) -> LeaderboardResponse:
        """
        Get leaderboard with model rankings

        Args:
            min_vote_count: Minimum number of votes required to appear on leaderboard

        Returns:
            LeaderboardResponse with ranked models sorted by ELO score descending
        """
        # Get models from repository (sorted by elo_score desc)
        models = await self.model_stats_repo.get_leaderboard(min_vote_count)

        # Get total votes
        total_votes = await self.model_stats_repo.get_total_votes(min_vote_count)

        # Get last update time from worker_status table
        # This shows when worker last ran, even if no votes were processed
        result = await self.db.execute(
            select(WorkerStatus).where(WorkerStatus.worker_name == "elo_aggregator")
        )
        worker_status = result.scalar_one_or_none()
        last_updated = worker_status.last_run_at if worker_status else datetime.now(UTC)

        # Build leaderboard entries with ranks (1 = highest ELO)
        leaderboard_entries = self._build_leaderboard_entries(models)

        # Build metadata
        metadata = LeaderboardMetadata(
            total_models=len(models),
            total_votes=total_votes,
            last_updated=last_updated,
        )

        return LeaderboardResponse(
            leaderboard=leaderboard_entries,
            metadata=metadata,
        )

    def _build_leaderboard_entries(
        self,
        models: List[ModelStats],
    ) -> List[ModelStatsResponse]:
        """
        Build leaderboard entries with ranks

        Args:
            models: List of ModelStats sorted by elo_score descending

        Returns:
            List of ModelStatsResponse with ranks assigned (1 = highest ELO)
        """
        entries = []
        for rank, model in enumerate(models, start=1):
            entry = ModelStatsResponse(
                rank=rank,
                model_id=model.model_id,
                model_name=model.model_id,  # For MVP, model_name = model_id
                elo_score=model.elo_score,
                elo_ci=model.elo_ci,
                vote_count=model.vote_count,
                win_rate=model.win_rate,
                organization=model.organization,
                license=model.license,
            )
            entries.append(entry)

        return entries
