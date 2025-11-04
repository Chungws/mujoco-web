"""
Session API endpoints
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.ext.asyncio import AsyncSession

from vlaarena_backend.database import get_db
from vlaarena_backend.services.session_service import (
    create_battle_in_session,
    create_session_with_battle,
    get_battles_by_session,
    get_sessions_by_user,
)
from vlaarena_shared.schemas import (
    BattleCreate,
    BattleListResponse,
    BattleResponse,
    SessionCreate,
    SessionListResponse,
    SessionResponse,
)


logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/sessions", response_model=SessionListResponse)
async def get_sessions(
    user_id: str = Query(..., description="User ID (UUID string for anonymous users)"),
    limit: int = Query(50, ge=1, le=100, description="Maximum number of sessions to return"),
    offset: int = Query(0, ge=0, description="Number of sessions to skip"),
    db: AsyncSession = Depends(get_db),
):
    """
    Get session list for a user with pagination

    Flow:
    1. User requests session list with user_id
    2. System retrieves sessions ordered by last_active_at DESC
    3. Returns session list with pagination info

    Args:
        user_id: User ID (UUID string for anonymous users)
        limit: Maximum number of sessions to return (1-100, default 50)
        offset: Number of sessions to skip (default 0)
        db: Database session

    Returns:
        SessionListResponse with sessions list and total count

    Raises:
        HTTPException 500: If internal error occurs
    """
    try:
        result = await get_sessions_by_user(user_id, db, limit=limit, offset=offset)
        return result

    except Exception as e:
        logger.error(f"Failed to get sessions for user {user_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get sessions: {str(e)}",
        )


@router.get("/sessions/{session_id}/battles", response_model=BattleListResponse)
async def get_session_battles(
    session_id: str,
    db: AsyncSession = Depends(get_db),
):
    """
    Get all battles for a specific session

    Flow:
    1. User requests battle list for session
    2. System retrieves all battles for session
    3. System includes vote information for voted battles
    4. Returns battle list

    Args:
        session_id: Session ID
        db: Database session

    Returns:
        BattleListResponse with session_id and battles list

    Raises:
        HTTPException 404: If session not found
        HTTPException 500: If internal error occurs
    """
    try:
        result = await get_battles_by_session(session_id, db)
        return result

    except ValueError:
        logger.error(f"Session not found: {session_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session not found: {session_id}",
        )

    except Exception as e:
        logger.error(f"Failed to get battles for session {session_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get battles: {str(e)}",
        )


@router.post("/sessions", response_model=SessionResponse, status_code=status.HTTP_201_CREATED)
async def create_session(
    data: SessionCreate,
    db: AsyncSession = Depends(get_db),
):
    """
    Create new session with first battle

    Flow:
    1. User submits initial prompt with optional user_id
    2. System creates session
    3. System selects 2 random models
    4. System calls both LLMs in parallel
    5. System creates battle with conversation
    6. Returns anonymous responses (model IDs hidden)

    Args:
        data: Session creation request with prompt and optional user_id
        db: Database session

    Returns:
        SessionResponse with session_id, battle_id, and anonymous responses

    Raises:
        HTTPException 500: If LLM API fails or internal error occurs
    """
    try:
        result = await create_session_with_battle(data.prompt, db, user_id=data.user_id)
        return result

    except Exception as e:
        logger.error(f"Failed to create session: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create session: {str(e)}",
        )


@router.post(
    "/sessions/{session_id}/battles",
    response_model=BattleResponse,
    status_code=status.HTTP_201_CREATED,
)
async def create_new_battle(
    session_id: str,
    data: BattleCreate,
    db: AsyncSession = Depends(get_db),
):
    """
    Create new battle in existing session

    Flow:
    1. User submits new prompt for second battle
    2. System verifies session exists
    3. System updates session.last_active_at
    4. System selects 2 NEW random models (different from previous battle)
    5. System calls both LLMs in parallel
    6. System creates new battle in same session
    7. Returns anonymous responses (model IDs hidden)

    Args:
        session_id: Existing session ID
        data: Battle creation request with prompt
        db: Database session

    Returns:
        BattleResponse with battle_id and anonymous responses

    Raises:
        HTTPException 404: If session not found
        HTTPException 500: If LLM API fails or internal error occurs
    """
    try:
        result = await create_battle_in_session(session_id, data.prompt, db)
        return result

    except ValueError:
        logger.error(f"Session not found: {session_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session not found: {session_id}",
        )

    except Exception as e:
        logger.error(f"Failed to create battle in session {session_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create battle: {str(e)}",
        )
