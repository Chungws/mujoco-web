/**
 * Model Reveal Component
 * Shows revealed model identities after voting with animation
 */

"use client";

import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { RevealedModels } from "@/lib/api/types";

interface ModelRevealProps {
  revealedModels: RevealedModels;
  voteResult: string;
  onNewBattle: () => void;
}

export default function ModelReveal({
  revealedModels,
  voteResult,
  onNewBattle,
}: ModelRevealProps) {
  const getVoteResultText = (vote: string) => {
    switch (vote) {
      case "left_better":
        return "You voted: Model A is better";
      case "right_better":
        return "You voted: Model B is better";
      case "tie":
        return "You voted: Tie";
      case "both_bad":
        return "You voted: Both Bad";
      default:
        return "";
    }
  };

  return (
    <Card className="p-6 bg-gradient-to-br from-blue-50 to-purple-50">
      <div className="space-y-6">
        <div className="text-center space-y-2">
          <h3 className="text-xl font-bold">Models Revealed!</h3>
          <p className="text-sm text-muted-foreground">
            {getVoteResultText(voteResult)}
          </p>
        </div>

        <div className="grid grid-cols-2 gap-4">
          {/* Model A */}
          <div className="space-y-3">
            <div className="flex items-center justify-center gap-2">
              <Badge variant="default" className="text-lg px-3 py-1">
                A
              </Badge>
              <span className="text-sm font-medium text-muted-foreground">
                Model A
              </span>
            </div>
            <Card className="p-4 bg-white">
              <div className="text-center space-y-2">
                <p className="font-semibold">{revealedModels.left}</p>
                <Badge variant="outline" className="text-xs">
                  {revealedModels.left.includes("octo") ? "Octo" : "SmolVLA"}
                </Badge>
              </div>
            </Card>
          </div>

          {/* Model B */}
          <div className="space-y-3">
            <div className="flex items-center justify-center gap-2">
              <Badge variant="secondary" className="text-lg px-3 py-1">
                B
              </Badge>
              <span className="text-sm font-medium text-muted-foreground">
                Model B
              </span>
            </div>
            <Card className="p-4 bg-white">
              <div className="text-center space-y-2">
                <p className="font-semibold">{revealedModels.right}</p>
                <Badge variant="outline" className="text-xs">
                  {revealedModels.right.includes("octo") ? "Octo" : "SmolVLA"}
                </Badge>
              </div>
            </Card>
          </div>
        </div>

        <div className="flex flex-col gap-3 pt-4">
          <Button onClick={onNewBattle} size="lg" className="w-full">
            Start New Battle
          </Button>
          <Button variant="outline" size="sm" className="w-full">
            View Leaderboard
          </Button>
        </div>

        <p className="text-xs text-center text-muted-foreground">
          Your vote has been recorded and will be included in the next ELO update
        </p>
      </div>
    </Card>
  );
}
