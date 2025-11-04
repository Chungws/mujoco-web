/**
 * API service for Leaderboard
 */

import { apiClient } from "@/lib/apiClient";
import type { LeaderboardResponse } from "./_types";

/**
 * Fetch leaderboard data
 *
 * Backend always returns data sorted by ELO score descending (rank 1 = highest ELO)
 * Client-side sorting is handled in use-leaderboard hook
 *
 * @returns Leaderboard data with metadata (sorted by ELO score)
 */
export async function getLeaderboard(): Promise<LeaderboardResponse> {
  return await apiClient.get<LeaderboardResponse>("/api/leaderboard");
}
