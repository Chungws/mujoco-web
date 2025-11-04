/**
 * Custom hook for managing leaderboard state
 */

"use client";

import React, { useState, useEffect, useCallback } from "react";
import { getLeaderboard } from "./service";
import type {
  LeaderboardEntry,
  LeaderboardMetadata,
  SortBy,
  SortOrder,
} from "./_types";

interface UseLeaderboardReturn {
  entries: LeaderboardEntry[];
  metadata: LeaderboardMetadata | null;
  isLoading: boolean;
  error: string | null;
  sortBy: SortBy;
  sortOrder: SortOrder;
  searchQuery: string;
  setSortBy: (sortBy: SortBy) => void;
  setSortOrder: (order: SortOrder) => void;
  setSearchQuery: (query: string) => void;
  refetch: () => Promise<void>;
}

export function useLeaderboard(): UseLeaderboardReturn {
  const [entries, setEntries] = useState<LeaderboardEntry[]>([]);
  const [metadata, setMetadata] = useState<LeaderboardMetadata | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [sortBy, setSortBy] = useState<SortBy>("rank");
  const [sortOrder, setSortOrder] = useState<SortOrder>("asc");
  const [searchQuery, setSearchQuery] = useState("");

  const fetchLeaderboard = useCallback(async () => {
    setIsLoading(true);
    setError(null);

    try {
      const data = await getLeaderboard();
      setEntries(data.leaderboard);
      setMetadata(data.metadata);
    } catch (err) {
      // If backend is not available, show empty state instead of error
      const errorMessage = err instanceof Error ? err.message : "Unknown error";
      if (errorMessage.includes("Failed to fetch") || errorMessage.includes("ERR_CONNECTION_REFUSED")) {
        // Backend not available - show empty state
        setEntries([]);
        setMetadata(null);
        setError(null);
      } else {
        // Real error - show error message
        setError(errorMessage);
        setEntries([]);
        setMetadata(null);
      }
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchLeaderboard();
  }, [fetchLeaderboard]);

  // Filter and sort entries (client-side)
  const processedEntries = React.useMemo(() => {
    // First, filter based on search query
    let filtered = searchQuery
      ? entries.filter(
          (entry) =>
            entry.model_name.toLowerCase().includes(searchQuery.toLowerCase()) ||
            entry.model_id.toLowerCase().includes(searchQuery.toLowerCase()) ||
            entry.organization.toLowerCase().includes(searchQuery.toLowerCase())
        )
      : entries;

    // Then, sort based on sortBy and sortOrder
    const sorted = [...filtered].sort((a, b) => {
      let aValue: string | number;
      let bValue: string | number;

      // Get values for comparison
      if (sortBy === "rank") {
        aValue = a.rank;
        bValue = b.rank;
      } else {
        aValue = a[sortBy];
        bValue = b[sortBy];
      }

      // Compare based on type
      if (typeof aValue === "string" && typeof bValue === "string") {
        // String comparison (case-insensitive)
        const comparison = aValue.toLowerCase().localeCompare(bValue.toLowerCase());
        return sortOrder === "asc" ? comparison : -comparison;
      } else if (typeof aValue === "number" && typeof bValue === "number") {
        // Number comparison
        return sortOrder === "asc" ? aValue - bValue : bValue - aValue;
      }

      return 0;
    });

    return sorted;
  }, [entries, searchQuery, sortBy, sortOrder]);

  return {
    entries: processedEntries,
    metadata,
    isLoading,
    error,
    sortBy,
    sortOrder,
    searchQuery,
    setSortBy,
    setSortOrder,
    setSearchQuery,
    refetch: fetchLeaderboard,
  };
}
