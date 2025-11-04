"""Initial schema

Revision ID: 001
Revises:
Create Date: 2025-01-21

"""
from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = '001'
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # sessions
    op.create_table('sessions',
        sa.Column('id', sa.BigInteger(), autoincrement=True, nullable=False),
        sa.Column('session_id', sa.String(length=50), nullable=False),
        sa.Column('title', sa.String(length=200), nullable=False),
        sa.Column('user_id', sa.BigInteger(), nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.Column('last_active_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('session_id')
    )
    op.create_index('idx_sessions_user_id', 'sessions', ['user_id'])
    op.create_index('idx_sessions_created_at', 'sessions', [sa.text('created_at DESC')])

    # battles
    op.create_table('battles',
        sa.Column('id', sa.BigInteger(), autoincrement=True, nullable=False),
        sa.Column('battle_id', sa.String(length=50), nullable=False),
        sa.Column('session_id', sa.String(length=50), nullable=False),
        sa.Column('left_model_id', sa.String(length=255), nullable=False),
        sa.Column('right_model_id', sa.String(length=255), nullable=False),
        sa.Column('conversation', postgresql.JSONB(), nullable=False, server_default=sa.text("'[]'::jsonb")),
        sa.Column('status', sa.String(length=20), nullable=False, server_default='ongoing'),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.Column('updated_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('battle_id'),
        sa.CheckConstraint("status IN ('ongoing', 'voted', 'abandoned')", name='battles_status_check')
        # Note: No FK constraint (ADR-001) - application-level referential integrity
    )
    op.create_index('idx_battles_session_id', 'battles', ['session_id'])
    op.create_index('idx_battles_status', 'battles', ['status'])
    op.create_index('idx_battles_session_status', 'battles', ['session_id', 'status'])
    op.create_index('idx_battles_created_at', 'battles', [sa.text('created_at DESC')])

    # votes
    op.create_table('votes',
        sa.Column('id', sa.BigInteger(), autoincrement=True, nullable=False),
        sa.Column('vote_id', sa.String(length=50), nullable=False),
        sa.Column('battle_id', sa.String(length=50), nullable=False),
        sa.Column('session_id', sa.String(length=50), nullable=False),
        sa.Column('vote', sa.String(length=20), nullable=False),
        sa.Column('left_model_id', sa.String(length=255), nullable=False),
        sa.Column('right_model_id', sa.String(length=255), nullable=False),
        sa.Column('processing_status', sa.String(length=20), nullable=False, server_default='pending'),
        sa.Column('processed_at', sa.TIMESTAMP(), nullable=True),
        sa.Column('error_message', sa.Text(), nullable=True),
        sa.Column('voted_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('vote_id'),
        sa.UniqueConstraint('battle_id'),
        sa.CheckConstraint("vote IN ('left_better', 'right_better', 'tie', 'both_bad')", name='votes_vote_check'),
        sa.CheckConstraint("processing_status IN ('pending', 'processed', 'failed')", name='votes_processing_status_check')
        # Note: No FK constraints (ADR-001) - application-level CASCADE deletion
    )
    op.create_index('idx_votes_processing_status', 'votes', ['processing_status'])
    op.create_index('idx_votes_session_id', 'votes', ['session_id'])
    op.create_index('idx_votes_voted_at', 'votes', ['voted_at'])

    # model_stats
    op.create_table('model_stats',
        sa.Column('id', sa.Integer(), autoincrement=True, nullable=False),
        sa.Column('model_id', sa.String(length=255), nullable=False),
        sa.Column('elo_score', sa.Integer(), nullable=False, server_default='1500'),
        sa.Column('elo_ci', sa.Float(), nullable=False, server_default='200.0'),
        sa.Column('vote_count', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('win_count', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('loss_count', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('tie_count', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('win_rate', sa.Float(), nullable=False, server_default='0.0'),
        sa.Column('organization', sa.String(length=255), nullable=False),
        sa.Column('license', sa.String(length=50), nullable=False),
        sa.Column('updated_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('model_id'),
        sa.CheckConstraint('elo_score >= 0', name='model_stats_elo_score_positive'),
        sa.CheckConstraint('vote_count >= 0', name='model_stats_vote_count_positive'),
        sa.CheckConstraint('win_rate >= 0.0 AND win_rate <= 1.0', name='model_stats_win_rate_range')
    )
    op.create_index('idx_model_stats_elo_score', 'model_stats', [sa.text('elo_score DESC')])
    op.create_index('idx_model_stats_vote_count', 'model_stats', [sa.text('vote_count DESC')])
    op.create_index('idx_model_stats_organization', 'model_stats', ['organization'])
    op.create_index('idx_model_stats_license', 'model_stats', ['license'])

    # worker_status
    op.create_table('worker_status',
        sa.Column('id', sa.Integer(), autoincrement=True, nullable=False),
        sa.Column('worker_name', sa.String(length=100), nullable=False),
        sa.Column('last_run_at', sa.TIMESTAMP(), server_default=sa.text('NOW()'), nullable=False),
        sa.Column('status', sa.String(length=50), nullable=False, server_default='idle'),
        sa.Column('votes_processed', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('error_message', sa.String(length=1000), nullable=True),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('worker_name'),
        sa.CheckConstraint('votes_processed >= 0', name='worker_status_votes_processed_positive'),
        sa.CheckConstraint("status IN ('idle', 'running', 'success', 'failed')", name='worker_status_status_check')
    )


def downgrade() -> None:
    op.drop_table('worker_status')
    op.drop_table('model_stats')
    op.drop_table('votes')
    op.drop_table('battles')
    op.drop_table('sessions')
