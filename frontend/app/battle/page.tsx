/**
 * Battle Mode Page (Server Component)
 *
 * Entry point for Battle Mode feature
 */

import { Suspense } from "react";
import { Skeleton } from "@/components/ui/skeleton";
import BattleClient from "@/components/battle/battle-arena-client";

export default async function BattlePage() {
  return (
    <Suspense fallback={
      <div className="container max-w-4xl mx-auto py-8 space-y-6">
        <Skeleton className="h-8 w-48" />
        <div className="space-y-4">
          <Skeleton className="h-32 w-full" />
          <Skeleton className="h-32 w-full" />
        </div>
      </div>
    }>
      <BattleClient />
    </Suspense>
  );
}
