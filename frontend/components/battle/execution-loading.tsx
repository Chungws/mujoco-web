/**
 * Execution Loading Component
 * Shows loading state during server-side VLA execution
 */

"use client";

import { Card } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";

interface ExecutionLoadingProps {
  instruction: string;
}

export default function ExecutionLoading({ instruction }: ExecutionLoadingProps) {
  return (
    <Card className="p-6">
      <div className="space-y-4">
        <div className="space-y-2">
          <h3 className="text-lg font-semibold">Executing Models</h3>
          <p className="text-sm text-muted-foreground">
            Instruction: {instruction}
          </p>
        </div>

        <div className="space-y-3">
          <div className="flex items-center gap-3">
            <div className="h-2 w-2 rounded-full bg-blue-500 animate-pulse" />
            <span className="text-sm">Running VLA models on server</span>
          </div>
          <div className="flex items-center gap-3">
            <div className="h-2 w-2 rounded-full bg-blue-500 animate-pulse animation-delay-200" />
            <span className="text-sm">Generating episodes (max 50 steps)</span>
          </div>
          <div className="flex items-center gap-3">
            <div className="h-2 w-2 rounded-full bg-blue-500 animate-pulse animation-delay-400" />
            <span className="text-sm">Saving states to database</span>
          </div>
        </div>

        <div className="pt-4 space-y-3">
          <div className="flex gap-4">
            <div className="flex-1">
              <p className="text-xs text-muted-foreground mb-2">Model A</p>
              <Skeleton className="h-48 w-full" />
            </div>
            <div className="flex-1">
              <p className="text-xs text-muted-foreground mb-2">Model B</p>
              <Skeleton className="h-48 w-full" />
            </div>
          </div>
        </div>

        <p className="text-xs text-muted-foreground text-center">
          This may take up to 60 seconds
        </p>
      </div>
    </Card>
  );
}
