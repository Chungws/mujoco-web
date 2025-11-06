/**
 * Leaderboard Client Component
 * Shows VLA model rankings with filtering by robot
 */

"use client";

import { useState, useEffect } from "react";
import { apiClient, APIError } from "@/lib/api/client";
import { ModelRanking } from "@/lib/api/types";
import { Card } from "@/components/ui/card";
import { Table } from "@/components/ui/table";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Alert } from "@/components/ui/alert";
import { Skeleton } from "@/components/ui/skeleton";

type FilterMode = "global" | "franka_panda";

export default function LeaderboardClient() {
  const [rankings, setRankings] = useState<ModelRanking[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [filterMode, setFilterMode] = useState<FilterMode>("global");

  useEffect(() => {
    fetchLeaderboard();
  }, [filterMode]);

  const fetchLeaderboard = async () => {
    try {
      setLoading(true);
      setError(null);

      const robotId = filterMode === "global" ? undefined : filterMode;
      const response = await apiClient.getLeaderboard(robotId);
      
      setRankings(response.rankings);
    } catch (err) {
      const errorMsg = err instanceof APIError 
        ? err.message 
        : "Failed to load leaderboard";
      setError(errorMsg);
    } finally {
      setLoading(false);
    }
  };

  const renderContent = () => {
    if (loading) {
      return (
        <Card className="p-6">
          <div className="space-y-4">
            {[1, 2, 3, 4, 5].map((i) => (
              <Skeleton key={i} className="h-16 w-full" />
            ))}
          </div>
        </Card>
      );
    }

    if (error) {
      return (
        <Alert variant="destructive">
          <p>{error}</p>
          <Button onClick={fetchLeaderboard} variant="outline" size="sm" className="mt-3">
            Retry
          </Button>
        </Alert>
      );
    }

    if (rankings.length === 0) {
      return (
        <Card className="p-12">
          <p className="text-center text-muted-foreground">
            No rankings available yet. Complete some battles to see rankings!
          </p>
        </Card>
      );
    }

    return (
      <Card className="p-6">
        <div className="overflow-x-auto">
          <table className="w-full">
            <thead>
              <tr className="border-b">
                <th className="text-left p-3 font-semibold">Rank</th>
                <th className="text-left p-3 font-semibold">Model</th>
                <th className="text-center p-3 font-semibold">ELO Score</th>
                <th className="text-center p-3 font-semibold">Votes</th>
                <th className="text-center p-3 font-semibold">Win Rate</th>
              </tr>
            </thead>
            <tbody>
              {rankings.map((ranking, index) => (
                <tr key={ranking.model_id} className="border-b hover:bg-slate-50">
                  <td className="p-3">
                    <div className="flex items-center gap-2">
                      {index === 0 && <Badge variant="default">🥇</Badge>}
                      {index === 1 && <Badge variant="secondary">🥈</Badge>}
                      {index === 2 && <Badge variant="outline">🥉</Badge>}
                      <span className="font-semibold">{index + 1}</span>
                    </div>
                  </td>
                  <td className="p-3">
                    <div className="space-y-1">
                      <p className="font-medium">{ranking.name}</p>
                      <p className="text-xs text-muted-foreground">
                        {ranking.model_id}
                      </p>
                    </div>
                  </td>
                  <td className="p-3 text-center">
                    <Badge variant="outline" className="font-mono">
                      {ranking.elo_score}
                    </Badge>
                  </td>
                  <td className="p-3 text-center">
                    <span className="text-sm font-mono">
                      {ranking.vote_count}
                    </span>
                  </td>
                  <td className="p-3 text-center">
                    <Badge 
                      variant={ranking.win_rate >= 0.5 ? "default" : "secondary"}
                      className="font-mono"
                    >
                      {(ranking.win_rate * 100).toFixed(1)}%
                    </Badge>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </Card>
    );
  };

  return (
    <div className="container max-w-6xl mx-auto py-8 space-y-6">
      {/* Header */}
      <div className="text-center space-y-2">
        <h1 className="text-3xl font-bold">VLA Arena - Leaderboard</h1>
        <p className="text-muted-foreground">
          Model rankings based on ELO scores from blind A/B testing
        </p>
      </div>

      {/* Filter buttons */}
      <Card className="p-4">
        <div className="flex items-center justify-center gap-3">
          <span className="text-sm font-medium">Filter by:</span>
          <div className="flex gap-2">
            <Button
              variant={filterMode === "global" ? "default" : "outline"}
              size="sm"
              onClick={() => setFilterMode("global")}
            >
              Global Rankings
            </Button>
            <Button
              variant={filterMode === "franka_panda" ? "default" : "outline"}
              size="sm"
              onClick={() => setFilterMode("franka_panda")}
            >
              Franka Panda
            </Button>
          </div>
        </div>
      </Card>

      {/* Rankings table */}
      {renderContent()}

      {/* Info footer */}
      <Card className="p-4 bg-blue-50 border-blue-200">
        <div className="text-sm text-center space-y-1">
          <p className="font-medium">ELO scores are updated hourly</p>
          <p className="text-xs text-muted-foreground">
            Rankings based on Bradley-Terry model with confidence intervals
          </p>
        </div>
      </Card>
    </div>
  );
}
