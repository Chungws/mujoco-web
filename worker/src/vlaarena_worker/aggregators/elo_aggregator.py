"""
ELO Aggregation Worker

Processes pending votes from PostgreSQL and updates model ELO ratings.

Workflow:
1. Read pending votes (processing_status = 'pending')
2. For each vote:
   - Get or create ModelStats for both models
   - Calculate new ELO ratings using elo_calculator
   - Update win/loss/tie counts
   - Calculate confidence intervals
   - Mark vote as processed
3. Handle errors and mark failed votes
"""

import logging
from datetime import UTC, datetime
from typing import Dict, Optional

from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select

from vlaarena_shared.models import ModelStats, Vote

from .elo_calculator import (
    INITIAL_ELO,
    calculate_ci,
    calculate_elo,
    get_score_from_vote,
)


logger = logging.getLogger("vlaarena_worker.elo_aggregator")


class ELOAggregator:
    """
    ELO rating aggregation worker

    Processes pending votes and updates model statistics in PostgreSQL.
    """

    def __init__(
        self,
        session: AsyncSession,
        model_configs: Optional[Dict[str, Dict[str, str]]] = None,
    ):
        """
        Initialize ELO aggregator

        Args:
            session: Async SQLAlchemy session for PostgreSQL
            model_configs: Optional dict mapping model_id -> {organization, license}
                          If None, defaults to "Unknown" values
        """
        self.session = session
        self.model_configs = model_configs or {}

    async def process_pending_votes(self) -> int:
        """
        Process all pending votes and update model statistics

        Returns:
            int: Number of votes processed

        Raises:
            Exception: If database operation fails
        """
        logger.info("Starting vote aggregation...")

        # Read all pending votes
        result = await self.session.execute(select(Vote).where(Vote.processing_status == "pending"))
        pending_votes = result.scalars().all()

        if not pending_votes:
            logger.info("No pending votes to process")
            return 0

        logger.info(f"Found {len(pending_votes)} pending votes")

        votes_processed = 0
        votes_failed = 0

        for vote in pending_votes:
            try:
                await self._process_single_vote(vote)
                votes_processed += 1
            except Exception as e:
                logger.error(
                    f"Failed to process vote {vote.vote_id}: {e}",
                    exc_info=True,
                )
                # Mark vote as failed
                vote.processing_status = "failed"
                vote.error_message = str(e)[:1000]  # Truncate to 1000 chars
                votes_failed += 1

        # Commit all changes
        await self.session.commit()

        logger.info(
            f"Vote aggregation complete: {votes_processed} processed, {votes_failed} failed"
        )
        return votes_processed

    async def _process_single_vote(self, vote: Vote) -> None:
        """
        Process a single vote and update model statistics

        Args:
            vote: Vote object to process

        Raises:
            ValueError: If vote type is invalid
            Exception: If database operation fails
        """
        # Get or create model stats for both models
        left_stats = await self._get_or_create_model_stats(vote.left_model_id)
        right_stats = await self._get_or_create_model_stats(vote.right_model_id)

        # Get scores for each model (raises ValueError if invalid vote type)
        left_score = get_score_from_vote(vote.vote, is_left=True)
        right_score = get_score_from_vote(vote.vote, is_left=False)

        # Calculate new ELO ratings
        new_left_elo = calculate_elo(
            left_stats.elo_score,
            right_stats.elo_score,
            left_score,
        )
        new_right_elo = calculate_elo(
            right_stats.elo_score,
            left_stats.elo_score,
            right_score,
        )

        # Update ELO scores
        left_stats.elo_score = round(new_left_elo)
        right_stats.elo_score = round(new_right_elo)

        # Update vote counts and win/loss/tie counts
        left_stats.vote_count += 1
        right_stats.vote_count += 1

        if left_score == 1.0:
            left_stats.win_count += 1
            right_stats.loss_count += 1
        elif right_score == 1.0:
            right_stats.win_count += 1
            left_stats.loss_count += 1
        elif left_score == 0.5:  # Tie
            left_stats.tie_count += 1
            right_stats.tie_count += 1
        # Note: both_bad (0.25) doesn't increment win/loss/tie

        # Calculate win rates
        left_stats.win_rate = (
            left_stats.win_count / left_stats.vote_count if left_stats.vote_count > 0 else 0.0
        )
        right_stats.win_rate = (
            right_stats.win_count / right_stats.vote_count if right_stats.vote_count > 0 else 0.0
        )

        # Calculate confidence intervals
        left_stats.elo_ci = calculate_ci(left_stats.vote_count)
        right_stats.elo_ci = calculate_ci(right_stats.vote_count)

        # Update timestamps
        left_stats.updated_at = datetime.now(UTC)
        right_stats.updated_at = datetime.now(UTC)

        # Mark vote as processed
        vote.processing_status = "processed"
        vote.processed_at = datetime.now(UTC)

        # Add to session (will be committed by caller)
        self.session.add(left_stats)
        self.session.add(right_stats)
        self.session.add(vote)

    async def _get_or_create_model_stats(self, model_id: str) -> ModelStats:
        """
        Get existing ModelStats or create new one with default values

        Args:
            model_id: Model identifier

        Returns:
            ModelStats: Existing or newly created model statistics

        Raises:
            Exception: If database operation fails
        """
        # Try to get existing stats
        result = await self.session.execute(
            select(ModelStats).where(ModelStats.model_id == model_id)
        )
        stats = result.scalar_one_or_none()

        if stats is not None:
            return stats

        # Get model config info if available
        model_info = self.model_configs.get(model_id, {})
        organization = model_info.get("organization", "Unknown")
        license_type = model_info.get("license", "Unknown")

        # Create new stats with default values
        logger.info(
            f"Creating new ModelStats for model: {model_id} "
            f"(org: {organization}, license: {license_type})"
        )
        stats = ModelStats(
            model_id=model_id,
            elo_score=INITIAL_ELO,
            elo_ci=200.0,
            vote_count=0,
            win_count=0,
            loss_count=0,
            tie_count=0,
            win_rate=0.0,
            organization=organization,
            license=license_type,
        )
        self.session.add(stats)
        await self.session.flush()  # Flush to get ID assigned

        return stats
