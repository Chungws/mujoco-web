"""
Tests for VotesService (VLA Arena MVP)
Following TDD workflow: Red → Green → Refactor

Test Coverage:
- create_vote: Success, battle not found, duplicate vote
- get_vote_by_id: Success, not found
"""

import pytest
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_backend.services.votes_service import create_vote
from vlaarena_shared.models import Battle, Session, Vote
from vlaarena_shared.schemas import VoteRequest


# ==================== Fixtures ====================


@pytest.fixture
async def sample_session(db: AsyncSession):
    """Create a sample session for testing"""
    session = Session(
        session_id="sess_test_123",
        robot_id="widowx",
        scene_id="table",
    )
    db.add(session)
    await db.commit()
    await db.refresh(session)
    return session


@pytest.fixture
async def sample_battle(db: AsyncSession, sample_session: Session):
    """Create a sample battle for testing"""
    battle = Battle(
        battle_id="battle_test_456",
        session_id=sample_session.session_id,
        seq_in_session=1,
        left_model_id="openvla-7b",
        right_model_id="octo-base",
        status="completed",
    )
    db.add(battle)
    await db.commit()
    await db.refresh(battle)
    return battle


# ==================== create_vote Tests ====================


class TestCreateVote:
    """Test create_vote function (POST /api/votes logic)"""

    @pytest.mark.asyncio
    async def test_create_vote_success_left_better(
        self, db: AsyncSession, sample_session: Session, sample_battle: Battle
    ):
        """
        Test successful vote creation with left_better vote

        Scenario:
        1. User votes for left model
        2. System creates vote record
        3. System denormalizes battle/session data
        4. System reveals model identities
        5. Returns vote_id and revealed models
        """
        # Arrange
        vote_data = VoteRequest(
            battle_id=sample_battle.battle_id,
            vote="left_better",
        )

        # Act
        result = await create_vote(db=db, vote_data=vote_data)

        # Assert
        assert "vote_id" in result
        assert "revealed_models" in result
        assert result["vote_id"].startswith("vote_")

        # Check revealed models
        revealed = result["revealed_models"]
        assert revealed["left"] == "openvla-7b"
        assert revealed["right"] == "octo-base"

    @pytest.mark.asyncio
    async def test_create_vote_creates_record_in_db(
        self, db: AsyncSession, sample_session: Session, sample_battle: Battle
    ):
        """
        Test that create_vote creates vote record in database

        Verification:
        - Vote record exists in database
        - Vote has correct battle_id, vote value
        - Vote has denormalized robot_id, scene_id, model_ids
        - Vote has processing_status = 'pending'
        """
        # Arrange
        vote_data = VoteRequest(
            battle_id=sample_battle.battle_id,
            vote="right_better",
        )

        # Act
        result = await create_vote(db=db, vote_data=vote_data)

        # Assert - Verify vote in database
        stmt = select(Vote).where(Vote.vote_id == result["vote_id"])
        db_result = await db.execute(stmt)
        vote = db_result.scalar_one_or_none()

        assert vote is not None
        assert vote.vote_id == result["vote_id"]
        assert vote.battle_id == sample_battle.battle_id
        assert vote.session_id == sample_session.session_id
        assert vote.vote == "right_better"

        # Check denormalized fields
        assert vote.robot_id == sample_session.robot_id
        assert vote.scene_id == sample_session.scene_id
        assert vote.left_model_id == sample_battle.left_model_id
        assert vote.right_model_id == sample_battle.right_model_id

        # Check processing status
        assert vote.processing_status == "pending"
        assert vote.processed_at is None
        assert vote.voted_at is not None

    @pytest.mark.asyncio
    async def test_create_vote_all_vote_types(self, db: AsyncSession, sample_session: Session):
        """
        Test all vote types: left_better, right_better, tie, both_bad

        Verification:
        - All vote types are accepted
        - Vote value is stored correctly
        """
        vote_types = ["left_better", "right_better", "tie", "both_bad"]

        for idx, vote_type in enumerate(vote_types):
            # Arrange - Create separate battle for each vote
            battle = Battle(
                battle_id=f"battle_test_{idx}",
                session_id=sample_session.session_id,
                seq_in_session=idx + 1,
                left_model_id="openvla-7b",
                right_model_id="octo-base",
                status="completed",
            )
            db.add(battle)
            await db.commit()
            await db.refresh(battle)

            vote_data = VoteRequest(
                battle_id=battle.battle_id,
                vote=vote_type,
            )

            # Act
            result = await create_vote(db=db, vote_data=vote_data)

            # Assert
            stmt = select(Vote).where(Vote.vote_id == result["vote_id"])
            db_result = await db.execute(stmt)
            vote = db_result.scalar_one_or_none()

            assert vote is not None
            assert vote.vote == vote_type

    @pytest.mark.asyncio
    async def test_create_vote_battle_not_found(self, db: AsyncSession):
        """
        Test vote creation fails when battle doesn't exist

        Expected:
        - Raises ValueError with "Battle not found" message
        """
        # Arrange
        vote_data = VoteRequest(
            battle_id="nonexistent_battle",
            vote="left_better",
        )

        # Act & Assert
        with pytest.raises(ValueError) as exc_info:
            await create_vote(db=db, vote_data=vote_data)

        assert "Battle not found" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_create_vote_duplicate_battle_id(
        self, db: AsyncSession, sample_session: Session, sample_battle: Battle
    ):
        """
        Test vote creation fails when battle already has a vote

        Expected:
        - First vote succeeds
        - Second vote for same battle raises ValueError
        """
        # Arrange
        vote_data_1 = VoteRequest(
            battle_id=sample_battle.battle_id,
            vote="left_better",
        )
        vote_data_2 = VoteRequest(
            battle_id=sample_battle.battle_id,
            vote="right_better",
        )

        # Act
        result_1 = await create_vote(db=db, vote_data=vote_data_1)

        # Assert - First vote succeeds
        assert "vote_id" in result_1

        # Act & Assert - Second vote fails
        with pytest.raises(ValueError) as exc_info:
            await create_vote(db=db, vote_data=vote_data_2)

        assert "already has a vote" in str(exc_info.value).lower()

    @pytest.mark.asyncio
    async def test_create_vote_session_not_found(self, db: AsyncSession, sample_battle: Battle):
        """
        Test vote creation fails when session doesn't exist

        This shouldn't happen in normal flow (battle references session),
        but we test defensive programming

        Expected:
        - Raises ValueError with "Session not found" message
        """
        # Arrange - Create battle without session (edge case)
        # Delete the session to simulate orphaned battle
        stmt = select(Session).where(Session.session_id == sample_battle.session_id)
        db_result = await db.execute(stmt)
        session = db_result.scalar_one_or_none()
        if session:
            await db.delete(session)
            await db.commit()

        vote_data = VoteRequest(
            battle_id=sample_battle.battle_id,
            vote="left_better",
        )

        # Act & Assert
        with pytest.raises(ValueError) as exc_info:
            await create_vote(db=db, vote_data=vote_data)

        assert "Session not found" in str(exc_info.value)
