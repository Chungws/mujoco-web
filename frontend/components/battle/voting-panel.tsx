/**
 * Voting Panel Component
 * Allows users to vote on which model performed better
 */

"use client";

import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { VoteOption } from "@/lib/api/types";

interface VotingPanelProps {
  onVote: (vote: VoteOption) => void;
  disabled?: boolean;
}

export default function VotingPanel({ onVote, disabled = false }: VotingPanelProps) {
  return (
    <Card className="p-6">
      <div className="space-y-4">
        <div className="text-center space-y-2">
          <h3 className="text-lg font-semibold">Which model performed better?</h3>
          <p className="text-sm text-muted-foreground">
            Compare the two executions and vote for the better performance
          </p>
        </div>

        <div className="grid grid-cols-2 gap-3">
          <Button
            onClick={() => onVote("left_better")}
            disabled={disabled}
            variant="outline"
            size="lg"
            className="h-20 flex flex-col items-center justify-center gap-2 hover:bg-blue-50 hover:border-blue-300"
          >
            <span className="text-2xl font-bold">A</span>
            <span className="text-xs">Model A is better</span>
          </Button>

          <Button
            onClick={() => onVote("right_better")}
            disabled={disabled}
            variant="outline"
            size="lg"
            className="h-20 flex flex-col items-center justify-center gap-2 hover:bg-blue-50 hover:border-blue-300"
          >
            <span className="text-2xl font-bold">B</span>
            <span className="text-xs">Model B is better</span>
          </Button>
        </div>

        <div className="grid grid-cols-2 gap-3">
          <Button
            onClick={() => onVote("tie")}
            disabled={disabled}
            variant="outline"
            className="hover:bg-slate-50 hover:border-slate-300"
          >
            Tie
          </Button>

          <Button
            onClick={() => onVote("both_bad")}
            disabled={disabled}
            variant="outline"
            className="hover:bg-slate-50 hover:border-slate-300"
          >
            Both Bad
          </Button>
        </div>
      </div>
    </Card>
  );
}
