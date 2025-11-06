/**
 * API Client for VLA Arena Backend
 * Base URL: http://localhost:8000
 */

import {
  SessionInitRequest,
  SessionResponse,
  TurnRequest,
  TurnResponse,
  EpisodeResponse,
  VoteRequest,
  VoteResponse,
  ModelsListResponse,
  LeaderboardResponse,
  ErrorResponse,
} from "./types";

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";

class APIError extends Error {
  constructor(
    public statusCode: number,
    message: string,
    public detail?: string
  ) {
    super(message);
    this.name = "APIError";
  }
}

async function handleResponse<T>(response: Response): Promise<T> {
  if (!response.ok) {
    const errorData: ErrorResponse = await response.json().catch(() => ({
      error: "Unknown error",
      status_code: response.status,
    }));

    throw new APIError(
      errorData.status_code,
      errorData.error,
      errorData.detail
    );
  }

  return response.json();
}

export const apiClient = {
  // ==================== Session API ====================

  async initSession(
    request: SessionInitRequest
  ): Promise<SessionResponse> {
    const response = await fetch(`${API_BASE_URL}/api/sessions/init`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(request),
    });

    return handleResponse<SessionResponse>(response);
  },

  // ==================== Battle/Turn API ====================

  async createTurn(
    battleId: string,
    request: TurnRequest
  ): Promise<TurnResponse> {
    const response = await fetch(
      `${API_BASE_URL}/api/battles/${battleId}/turns`,
      {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(request),
      }
    );

    return handleResponse<TurnResponse>(response);
  },

  // ==================== Episode API ====================

  async getEpisode(episodeId: string): Promise<EpisodeResponse> {
    const response = await fetch(
      `${API_BASE_URL}/api/episodes/${episodeId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
        },
      }
    );

    return handleResponse<EpisodeResponse>(response);
  },

  // ==================== Vote API ====================

  async submitVote(request: VoteRequest): Promise<VoteResponse> {
    const response = await fetch(`${API_BASE_URL}/api/votes`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(request),
    });

    return handleResponse<VoteResponse>(response);
  },

  // ==================== Models API ====================

  async listModels(): Promise<ModelsListResponse> {
    const response = await fetch(`${API_BASE_URL}/api/models`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
      },
    });

    return handleResponse<ModelsListResponse>(response);
  },

  // ==================== Leaderboard API ====================

  async getLeaderboard(robotId?: string): Promise<LeaderboardResponse> {
    const url = robotId
      ? `${API_BASE_URL}/api/leaderboard?robot_id=${robotId}`
      : `${API_BASE_URL}/api/leaderboard`;

    const response = await fetch(url, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
      },
    });

    return handleResponse<LeaderboardResponse>(response);
  },
};

export { APIError };
