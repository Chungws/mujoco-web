/**
 * Leaderboard Page (Server Component)
 *
 * Shows VLA model rankings with robot-specific and global ELO scores
 */

import { Suspense } from "react";
import { Skeleton } from "@/components/ui/skeleton";
import LeaderboardClient from "@/components/leaderboard/leaderboard-client";

export default async function LeaderboardPage() {
  return (
    <Suspense fallback={
      <div className="container max-w-6xl mx-auto py-8 space-y-6">
        <Skeleton className="h-10 w-64" />
        <Skeleton className="h-96 w-full" />
      </div>
    }>
      <LeaderboardClient />
    </Suspense>
  );
}
