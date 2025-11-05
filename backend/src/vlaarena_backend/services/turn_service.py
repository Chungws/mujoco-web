"""
Turn service layer - Business logic for battle turns

Handles turn creation, VLA execution coordination, and episode storage.
Following FastAPI patterns: service layer contains all business logic.
"""

import logging
import secrets
from datetime import UTC, datetime

from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select

from vlaarena_shared.models import Battle, Session, Turn
from vlaarena_shared.mongodb_models import Episode, State
from vlaarena_shared.schemas import TurnRequest, TurnResponse

from .vla_service import MockVLAService


logger = logging.getLogger(__name__)


class TurnService:
    """Service for managing battle turns and episode generation"""

    # TODO: Refactor to use Protocol/ABC for VLA service interface
    # Current design tightly couples TurnService to MockVLAService.
    # When implementing real VLA service (Phase 2), refactor to:
    # 1. Define VLAService Protocol/ABC with generate_episode() interface
    # 2. Remove default MockVLAService() instantiation
    # 3. Require VLA service injection via __init__ (no default)
    # 4. Update API router to inject concrete implementation
    # This enables clean dependency injection and easier testing.

    def __init__(self, vla_service: MockVLAService | None = None):
        """
        Initialize turn service

        Args:
            vla_service: VLA execution service (defaults to MockVLAService)
        """
        self.vla_service = vla_service or MockVLAService()

    async def create_turn(
        self,
        battle_id: str,
        data: TurnRequest,
        db: AsyncSession,
    ) -> TurnResponse:
        """
        Create a new turn and generate episodes

        Flow:
        1. Validate battle exists and get battle details
        2. Create Turn record in PostgreSQL
        3. Execute left model → save Episode to MongoDB
        4. Execute right model → save Episode to MongoDB
        5. Update session.last_active_at
        6. Return TurnResponse

        Args:
            battle_id: Battle identifier
            data: Turn request with instruction
            db: Database session

        Returns:
            TurnResponse with turn_id and episode_ids

        Raises:
            ValueError: If battle or session not found
            Exception: If database or MongoDB operations fail
        """
        logger.info(f"Creating turn for battle {battle_id}: {data.instruction[:50]}...")

        try:
            # 1. Get battle and validate
            battle = await self._get_battle(battle_id, db)
            if not battle:
                raise ValueError(f"Battle not found: {battle_id}")

            # Get session to retrieve robot_id and scene_id
            session = await self._get_session(battle.session_id, db)
            if not session:
                raise ValueError(f"Session not found: {battle.session_id}")

            # 2. Create Turn record
            turn = await self._create_turn_record(battle, data.instruction, db)

            # 3. Execute left model and save episode
            left_episode_id = await self._execute_and_save_episode(
                episode_id=f"ep_{secrets.token_hex(8)}_left",
                turn=turn,
                battle=battle,
                session=session,
                model_id=battle.left_model_id,
                side="left",
                seq_in_turn=0,
                instruction=data.instruction,
            )

            # 4. Execute right model and save episode
            right_episode_id = await self._execute_and_save_episode(
                episode_id=f"ep_{secrets.token_hex(8)}_right",
                turn=turn,
                battle=battle,
                session=session,
                model_id=battle.right_model_id,
                side="right",
                seq_in_turn=1,
                instruction=data.instruction,
            )

            # 5. Update session last_active_at
            await self._update_session_last_active(session, db)

            # Commit transaction
            await db.commit()

            logger.info(
                f"Turn created: {turn.turn_id}, left={left_episode_id}, right={right_episode_id}"
            )

            # 6. Return response
            return TurnResponse(
                turn_id=turn.turn_id,
                left_episode_id=left_episode_id,
                right_episode_id=right_episode_id,
                status="completed",
            )

        except ValueError:
            # Re-raise validation errors (battle/session not found)
            await db.rollback()
            raise

        except Exception as e:
            # Rollback on any database or MongoDB error
            logger.error(f"Failed to create turn for battle {battle_id}: {e}")
            await db.rollback()
            raise

    async def _get_battle(self, battle_id: str, db: AsyncSession) -> Battle | None:
        """Get battle by ID"""
        statement = select(Battle).where(Battle.battle_id == battle_id)
        result = await db.execute(statement)
        return result.scalar_one_or_none()

    async def _get_session(self, session_id: str, db: AsyncSession) -> Session | None:
        """Get session by ID"""
        statement = select(Session).where(Session.session_id == session_id)
        result = await db.execute(statement)
        return result.scalar_one_or_none()

    async def _create_turn_record(self, battle: Battle, instruction: str, db: AsyncSession) -> Turn:
        """
        Create Turn record in PostgreSQL

        Args:
            battle: Battle object
            instruction: User instruction
            db: Database session

        Returns:
            Created Turn object
        """
        # Get next turn sequence number for this battle
        statement = select(Turn).where(Turn.battle_id == battle.battle_id).order_by(Turn.seq.desc())
        result = await db.execute(statement)
        last_turn = result.scalar_one_or_none()
        next_seq = (last_turn.seq + 1) if last_turn else 1

        # Create turn
        turn = Turn(
            turn_id=f"turn_{secrets.token_hex(8)}",
            session_id=battle.session_id,
            battle_id=battle.battle_id,
            battle_seq_in_session=battle.seq_in_session,
            seq=next_seq,
            instruction=instruction,
        )

        db.add(turn)
        await db.commit()
        await db.refresh(turn)

        logger.info(f"Turn record created: {turn.turn_id}, seq={next_seq}")
        return turn

    async def _execute_and_save_episode(
        self,
        episode_id: str,
        turn: Turn,
        battle: Battle,
        session: Session,
        model_id: str,
        side: str,
        seq_in_turn: int,
        instruction: str,
    ) -> str:
        """
        Execute VLA model and save episode to MongoDB

        Args:
            episode_id: Episode identifier
            turn: Turn object
            battle: Battle object
            session: Session object
            model_id: VLA model identifier
            side: "left" or "right"
            seq_in_turn: 0 for left, 1 for right
            instruction: User instruction

        Returns:
            Episode ID
        """
        logger.info(f"Executing {model_id} ({side}) for turn {turn.turn_id}")

        # Execute VLA model
        episode_data = self.vla_service.generate_episode(
            model_id=model_id,
            instruction=instruction,
            robot_id=session.robot_id,
            scene_id=session.scene_id,
        )

        # Convert states to State objects
        states = [State(**state_dict) for state_dict in episode_data["states"]]

        # Create Episode document
        episode = Episode(
            episode_id=episode_id,
            session_id=session.session_id,
            battle_id=battle.battle_id,
            turn_id=turn.turn_id,
            battle_seq_in_session=battle.seq_in_session,
            turn_seq=turn.seq,
            seq_in_turn=seq_in_turn,
            side=side,
            model_id=model_id,
            actions=episode_data["actions"],
            states=states,
            duration_ms=episode_data["duration_ms"],
        )

        # Save to MongoDB
        await episode.insert()

        logger.info(f"Episode saved: {episode_id}, {len(episode.actions)} steps")

        return episode_id

    async def _update_session_last_active(self, session: Session, db: AsyncSession) -> None:
        """Update session.last_active_at to current time"""
        session.last_active_at = datetime.now(UTC)
        db.add(session)
        await db.commit()
