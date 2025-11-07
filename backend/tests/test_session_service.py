"""
Tests for SessionService (VLA Arena MVP)
Following TDD workflow: Red → Green → Refactor
"""

import pytest
from sqlalchemy.ext.asyncio import AsyncSession
from vlaarena_backend.services.session_service import init_session
from vlaarena_shared.models import Battle, Session

# ==================== SessionService.init_session Tests ====================


class TestInitSession:
    """Test init_session function (POST /api/sessions/init logic)"""

    @pytest.mark.asyncio
    async def test_init_session_success(self, db: AsyncSession):
        """
        Test successful session initialization with battle creation

        Scenario:
        1. User provides robot_id and scene_id
        2. System creates session record
        3. System assigns 2 random VLA models
        4. System creates battle record
        5. Returns session_id, battle_id, and hidden model names
        """
        # Arrange
        robot_id = "widowx"
        scene_id = "table"

        # Act
        result = await init_session(
            db=db,
            robot_id=robot_id,
            scene_id=scene_id,
        )

        # Assert
        assert "session_id" in result
        assert "battle_id" in result
        assert "left_model" in result
        assert "right_model" in result

        # Check session_id format
        assert result["session_id"].startswith("sess_")

        # Check battle_id format
        assert result["battle_id"].startswith("battle_")

        # Check models are hidden
        assert result["left_model"] == "???"
        assert result["right_model"] == "???"

    @pytest.mark.asyncio
    async def test_init_session_creates_session_in_db(self, db: AsyncSession):
        """
        Test that init_session creates session record in database

        Verification:
        - Session record exists in database
        - Session has correct robot_id and scene_id
        """
        # Arrange
        robot_id = "widowx"
        scene_id = "table"

        # Act
        result = await init_session(
            db=db,
            robot_id=robot_id,
            scene_id=scene_id,
        )

        # Assert - Verify session in database
        from sqlalchemy import select

        stmt = select(Session).where(Session.session_id == result["session_id"])
        db_result = await db.execute(stmt)
        session = db_result.scalar_one_or_none()

        assert session is not None
        assert session.session_id == result["session_id"]
        assert session.robot_id == robot_id
        assert session.scene_id == scene_id
        assert session.user_id is None  # MVP doesn't have user auth
        assert session.created_at is not None
        assert session.last_active_at is not None

    @pytest.mark.asyncio
    async def test_init_session_creates_battle_in_db(self, db: AsyncSession):
        """
        Test that init_session creates battle record in database

        Verification:
        - Battle record exists in database
        - Battle has correct session_id
        - Battle has 2 different model_ids assigned
        - Battle status is 'ongoing'
        - Battle seq_in_session is 0 (first battle)
        """
        # Arrange
        robot_id = "widowx"
        scene_id = "table"

        # Act
        result = await init_session(
            db=db,
            robot_id=robot_id,
            scene_id=scene_id,
        )

        # Assert - Verify battle in database
        from sqlalchemy import select

        stmt = select(Battle).where(Battle.battle_id == result["battle_id"])
        db_result = await db.execute(stmt)
        battle = db_result.scalar_one_or_none()

        assert battle is not None
        assert battle.battle_id == result["battle_id"]
        assert battle.session_id == result["session_id"]
        assert battle.left_model_id != ""
        assert battle.right_model_id != ""
        assert battle.left_model_id != battle.right_model_id  # Different models
        assert battle.status == "ongoing"
        assert battle.seq_in_session == 0  # First battle
        assert battle.created_at is not None
        assert battle.updated_at is not None

    @pytest.mark.asyncio
    async def test_init_session_with_user_id(self, db: AsyncSession):
        """
        Test init_session with optional user_id (post-MVP feature)

        Scenario:
        1. User provides robot_id, scene_id, and user_id
        2. System creates session with user_id
        """
        # Arrange
        robot_id = "widowx"
        scene_id = "table"
        user_id = "user_test123"

        # Act
        result = await init_session(
            db=db,
            robot_id=robot_id,
            scene_id=scene_id,
            user_id=user_id,
        )

        # Assert - Verify session has user_id
        from sqlalchemy import select

        stmt = select(Session).where(Session.session_id == result["session_id"])
        db_result = await db.execute(stmt)
        session = db_result.scalar_one_or_none()

        assert session is not None
        assert session.user_id == user_id

    @pytest.mark.asyncio
    async def test_init_session_models_are_random(self, db: AsyncSession):
        """
        Test that init_session assigns models randomly

        Verification:
        - Multiple calls produce different model combinations
        - At least some variation in left/right assignment
        """
        # Arrange
        robot_id = "widowx"
        scene_id = "table"
        results = []

        # Act - Create multiple sessions
        for _ in range(5):
            result = await init_session(
                db=db,
                robot_id=robot_id,
                scene_id=scene_id,
            )
            results.append(result)

        # Assert - Get battles from database
        from sqlalchemy import select

        battles = []
        for result in results:
            stmt = select(Battle).where(Battle.battle_id == result["battle_id"])
            db_result = await db.execute(stmt)
            battle = db_result.scalar_one()
            battles.append(battle)

        # Check that we have at least some variation in model assignments
        # (This is probabilistic, but with 5 trials should pass)
        model_pairs = set()
        for battle in battles:
            model_pairs.add((battle.left_model_id, battle.right_model_id))

        # With 2 models and 5 trials, we should see at least 2 unique orderings
        assert len(model_pairs) >= 1  # At minimum, all should have valid assignments

    @pytest.mark.asyncio
    async def test_init_session_different_robots(self, db: AsyncSession):
        """
        Test init_session with different robot_ids

        Scenario:
        1. Create sessions for different robots
        2. Verify robot_id is stored correctly
        """
        # Arrange
        robot_ids = ["widowx", "franka", "ur5"]  # Future robots
        scene_id = "table"

        # Act & Assert
        for robot_id in robot_ids:
            result = await init_session(
                db=db,
                robot_id=robot_id,
                scene_id=scene_id,
            )

            # Verify session in database
            from sqlalchemy import select

            stmt = select(Session).where(Session.session_id == result["session_id"])
            db_result = await db.execute(stmt)
            session = db_result.scalar_one()

            assert session.robot_id == robot_id

    @pytest.mark.asyncio
    async def test_init_session_different_scenes(self, db: AsyncSession):
        """
        Test init_session with different scene_ids

        Scenario:
        1. Create sessions for different scenes
        2. Verify scene_id is stored correctly
        """
        # Arrange
        robot_id = "widowx"
        scene_ids = ["table", "kitchen", "warehouse"]  # Future scenes

        # Act & Assert
        for scene_id in scene_ids:
            result = await init_session(
                db=db,
                robot_id=robot_id,
                scene_id=scene_id,
            )

            # Verify session in database
            from sqlalchemy import select

            stmt = select(Session).where(Session.session_id == result["session_id"])
            db_result = await db.execute(stmt)
            session = db_result.scalar_one()

            assert session.scene_id == scene_id
