/**
 * Leaderboard Client Component
 *
 * Displays ELO-based rankings for all models
 */

"use client";

import { useLeaderboard } from "./use-leaderboard";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent } from "@/components/ui/card";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Alert } from "@/components/ui/alert";
import { ArrowUp, ArrowDown, ArrowUpDown, Search } from "lucide-react";
import type { SortBy } from "./_types";

export default function LeaderboardClient() {
  const {
    entries,
    metadata,
    isLoading,
    error,
    sortBy,
    sortOrder,
    searchQuery,
    setSortBy,
    setSortOrder,
    setSearchQuery,
  } = useLeaderboard();

  // Handle column header click for sorting
  const handleSort = (column: SortBy) => {
    if (sortBy === column) {
      // Toggle order if same column
      setSortOrder(sortOrder === "asc" ? "desc" : "asc");
    } else {
      // Set new column and always start with ascending
      setSortBy(column);
      setSortOrder("asc");
    }
  };

  // Render sort icon
  const renderSortIcon = (column: SortBy) => {
    if (sortBy !== column) {
      // Not currently sorted - show both arrows with low opacity
      return <ArrowUpDown className="ml-1 h-4 w-4 inline opacity-30" />;
    }
    // Currently sorted - show direction arrow
    return sortOrder === "asc" ? (
      <ArrowUp className="ml-1 h-4 w-4 inline" />
    ) : (
      <ArrowDown className="ml-1 h-4 w-4 inline" />
    );
  };

  // Format timestamp to local time
  const formatLastUpdated = (timestamp: string): string => {
    const date = new Date(timestamp);
    return date.toLocaleString(undefined, {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
      hour12: false,
    });
  };

  return (
    <div className="min-h-screen p-4 md:p-8">
      <div className="max-w-7xl mx-auto space-y-6">
        {/* Header */}
        <div className="space-y-2">
          <h1 className="text-3xl md:text-4xl font-bold">Leaderboard</h1>
          <p className="text-muted-foreground">
            ELO-based rankings for all models based on user votes
          </p>
        </div>

        {/* Metadata Row */}
        {metadata && (
          <div className="flex flex-wrap gap-6 text-sm">
            <div>
              <span className="text-muted-foreground">Last Updated: </span>
              <span className="font-medium">{formatLastUpdated(metadata.last_updated)}</span>
            </div>
            <div>
              <span className="text-muted-foreground">Total Votes: </span>
              <span className="font-medium">{metadata.total_votes.toLocaleString()}</span>
            </div>
            <div>
              <span className="text-muted-foreground">Total Models: </span>
              <span className="font-medium">{metadata.total_models}</span>
            </div>
          </div>
        )}

        {/* Search Input */}
        <div className="relative max-w-md">
          <Search className="absolute left-3 top-1/2 h-4 w-4 -translate-y-1/2 text-muted-foreground" />
          <Input
            type="text"
            placeholder="Search by model name..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            className="pl-9 pr-12"
          />
          <kbd className="pointer-events-none absolute right-3 top-1/2 -translate-y-1/2 hidden h-5 select-none items-center gap-1 rounded border bg-muted px-1.5 font-mono text-[10px] font-medium opacity-100 sm:flex">
            /
          </kbd>
        </div>

        {/* Error State */}
        {error && (
          <Alert variant="destructive">
            <p className="font-semibold">Error loading leaderboard</p>
            <p className="text-sm">{error}</p>
          </Alert>
        )}

        {/* Loading State */}
        {isLoading && (
          <Card>
            <CardContent className="p-12 text-center">
              <div className="flex justify-center items-center space-x-2">
                <div className="w-4 h-4 bg-primary rounded-full animate-bounce [animation-delay:-0.3s]"></div>
                <div className="w-4 h-4 bg-primary rounded-full animate-bounce [animation-delay:-0.15s]"></div>
                <div className="w-4 h-4 bg-primary rounded-full animate-bounce"></div>
              </div>
              <p className="mt-4 text-muted-foreground">Loading leaderboard...</p>
            </CardContent>
          </Card>
        )}

        {/* Leaderboard Table */}
        {!isLoading && !error && (
          <Card>
            <CardContent className="p-0">
              <div className="overflow-x-auto">
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead
                        className="w-[80px] text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("rank")}
                      >
                        Rank{renderSortIcon("rank")}
                      </TableHead>
                      <TableHead>Model</TableHead>
                      <TableHead
                        className="text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("elo_score")}
                      >
                        Score{renderSortIcon("elo_score")}
                      </TableHead>
                      <TableHead
                        className="text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("elo_ci")}
                      >
                        95% CI{renderSortIcon("elo_ci")}
                      </TableHead>
                      <TableHead
                        className="text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("vote_count")}
                      >
                        Votes{renderSortIcon("vote_count")}
                      </TableHead>
                      <TableHead
                        className="text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("win_rate")}
                      >
                        Win Rate{renderSortIcon("win_rate")}
                      </TableHead>
                      <TableHead
                        className="text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("organization")}
                      >
                        Organization{renderSortIcon("organization")}
                      </TableHead>
                      <TableHead
                        className="text-center cursor-pointer hover:bg-zinc-800/50 select-none"
                        onClick={() => handleSort("license")}
                      >
                        License{renderSortIcon("license")}
                      </TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {entries.length === 0 ? (
                      <TableRow>
                        <TableCell colSpan={8} className="h-24 text-center">
                          <p className="text-muted-foreground">
                            {searchQuery
                              ? "No models match your search"
                              : "No leaderboard data available"}
                          </p>
                        </TableCell>
                      </TableRow>
                    ) : (
                      entries.map((entry) => (
                        <TableRow key={entry.model_id}>
                          <TableCell className="text-center font-medium">
                            {entry.rank}
                          </TableCell>
                          <TableCell>
                            <div className="flex flex-col">
                              <span className="font-medium">
                                {entry.model_name}
                              </span>
                              <span className="text-xs text-muted-foreground">
                                {entry.model_id}
                              </span>
                            </div>
                          </TableCell>
                          <TableCell className="text-center font-semibold">
                            {entry.elo_score}
                          </TableCell>
                          <TableCell className="text-center text-muted-foreground">
                            ±{entry.elo_ci.toFixed(1)}
                          </TableCell>
                          <TableCell className="text-center">
                            {entry.vote_count.toLocaleString()}
                          </TableCell>
                          <TableCell className="text-center">
                            {(entry.win_rate * 100).toFixed(1)}%
                          </TableCell>
                          <TableCell className="text-center">
                            <Badge variant="secondary">{entry.organization}</Badge>
                          </TableCell>
                          <TableCell className="text-center">
                            <Badge
                              variant={
                                entry.license === "proprietary"
                                  ? "outline"
                                  : "default"
                              }
                            >
                              {entry.license}
                            </Badge>
                          </TableCell>
                        </TableRow>
                      ))
                    )}
                  </TableBody>
                </Table>
              </div>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
