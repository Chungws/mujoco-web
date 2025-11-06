/**
 * TypeScript types for VLA Arena API
 * Based on shared/src/vlaarena_shared/schemas.py
 */

// ==================== Session Types ====================

export interface SessionInitRequest {
  robot_id: string;
  scene_id: string;
}

export interface SessionResponse {
  session_id: string;
  battle_id: string;
  left_model: string; // "???" until vote
  right_model: string; // "???" until vote
}

// ==================== Turn Types ====================

export interface TurnRequest {
  instruction: string;
}

export interface TurnResponse {
  turn_id: string;
  left_episode_id: string;
  right_episode_id: string;
  status: string; // "completed" | "failed" | "running"
}

// ==================== Episode Types ====================

export interface EpisodeState {
  qpos: number[];
  qvel: number[];
  time: number;
}

export interface EpisodeResponse {
  episode_id: string;
  actions: number[][]; // Variable length, up to 50
  states: EpisodeState[]; // Variable length, up to 50
}

// ==================== Vote Types ====================

export type VoteOption = "left_better" | "right_better" | "tie" | "both_bad";

export interface VoteRequest {
  battle_id: string;
  vote: VoteOption;
}

export interface RevealedModels {
  left: string; // model_id
  right: string; // model_id
}

export interface VoteResponse {
  vote_id: string;
  revealed_models: RevealedModels;
}

// ==================== Models Types ====================

export interface ModelInfo {
  model_id: string;
  name: string;
  provider: string;
  status: "active" | "inactive";
}

export interface ModelsListResponse {
  models: ModelInfo[];
}

// ==================== Leaderboard Types ====================

export interface ModelRanking {
  model_id: string;
  name: string;
  elo_score: number;
  vote_count: number;
  win_rate: number;
}

export interface LeaderboardResponse {
  rankings: ModelRanking[];
}

// ==================== Error Types ====================

export interface ErrorResponse {
  error: string;
  detail?: string;
  status_code: number;
}
