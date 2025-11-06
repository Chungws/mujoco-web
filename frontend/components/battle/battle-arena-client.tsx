/**
 * Battle Arena Client Component
 * Main orchestrator for the battle page flow
 *
 * Flow:
 * 1. Initialize session (auto on mount)
 * 2. User provides instruction
 * 3. Execute models (server-side)
 * 4. Show side-by-side episode replay
 * 5. User votes
 * 6. Reveal models
 * 7. Optional: Start new turn or new battle
 */

"use client";

import { useState, useEffect } from "react";
import { apiClient, APIError } from "@/lib/api/client";
import {
  SessionResponse,
  TurnResponse,
  EpisodeResponse,
  VoteOption,
  RevealedModels,
} from "@/lib/api/types";
import InstructionInput from "./instruction-input";
import ExecutionLoading from "./execution-loading";
import MujocoViewer from "./mujoco-viewer";
import VotingPanel from "./voting-panel";
import ModelReveal from "./model-reveal";
import { Alert } from "@/components/ui/alert";
import { Card } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";

type BattleState =
  | "initializing"
  | "ready"
  | "executing"
  | "replaying"
  | "voting"
  | "revealed"
  | "error";

export default function BattleClient() {
  // Session state
  const [session, setSession] = useState<SessionResponse | null>(null);
  const [battleState, setBattleState] = useState<BattleState>("initializing");
  const [error, setError] = useState<string | null>(null);

  // Turn state
  const [currentInstruction, setCurrentInstruction] = useState<string>("");
  const [turnResponse, setTurnResponse] = useState<TurnResponse | null>(null);

  // Episode state
  const [leftEpisode, setLeftEpisode] = useState<EpisodeResponse | null>(null);
  const [rightEpisode, setRightEpisode] = useState<EpisodeResponse | null>(null);

  // Vote state
  const [currentVote, setCurrentVote] = useState<VoteOption | null>(null);
  const [revealedModels, setRevealedModels] = useState<RevealedModels | null>(null);

  // Initialize session on mount
  useEffect(() => {
    initializeSession();
  }, []);

  const initializeSession = async () => {
    try {
      setBattleState("initializing");
      setError(null);

      const response = await apiClient.initSession({
        robot_id: "franka_panda", // MVP: Fixed robot
        scene_id: "table", // MVP: Fixed scene
      });

      setSession(response);
      setBattleState("ready");
    } catch (err) {
      const errorMsg = err instanceof APIError ? err.message : "Failed to initialize session";
      setError(errorMsg);
      setBattleState("error");
    }
  };

  const handleInstructionSubmit = async (instruction: string) => {
    if (!session) return;

    try {
      setBattleState("executing");
      setError(null);
      setCurrentInstruction(instruction);

      // Execute turn (server-side VLA execution)
      const turnRes = await apiClient.createTurn(session.battle_id, {
        instruction,
      });

      setTurnResponse(turnRes);

      // Fetch both episodes
      const [leftEp, rightEp] = await Promise.all([
        apiClient.getEpisode(turnRes.left_episode_id),
        apiClient.getEpisode(turnRes.right_episode_id),
      ]);

      setLeftEpisode(leftEp);
      setRightEpisode(rightEp);
      setBattleState("replaying");

      // Auto-transition to voting after brief delay (simulate replay viewing)
      setTimeout(() => {
        setBattleState("voting");
      }, 2000);
    } catch (err) {
      const errorMsg = err instanceof APIError ? err.message : "Failed to execute models";
      setError(errorMsg);
      setBattleState("error");
    }
  };

  const handleVote = async (vote: VoteOption) => {
    if (!session) return;

    try {
      setCurrentVote(vote);
      setBattleState("executing"); // Show loading during vote submission

      const voteRes = await apiClient.submitVote({
        battle_id: session.battle_id,
        vote,
      });

      setRevealedModels(voteRes.revealed_models);
      setBattleState("revealed");
    } catch (err) {
      const errorMsg = err instanceof APIError ? err.message : "Failed to submit vote";
      setError(errorMsg);
      setBattleState("voting"); // Return to voting on error
    }
  };

  const handleNewBattle = () => {
    // Reset all state and initialize new session
    setSession(null);
    setTurnResponse(null);
    setLeftEpisode(null);
    setRightEpisode(null);
    setCurrentVote(null);
    setRevealedModels(null);
    setCurrentInstruction("");
    initializeSession();
  };

  // Render helpers
  const renderContent = () => {
    switch (battleState) {
      case "initializing":
        return (
          <Card className="p-8">
            <div className="space-y-4">
              <Skeleton className="h-8 w-48 mx-auto" />
              <Skeleton className="h-32 w-full" />
              <Skeleton className="h-32 w-full" />
            </div>
          </Card>
        );

      case "ready":
        return (
          <div className="space-y-4">
            <Card className="p-4 bg-blue-50 border-blue-200">
              <p className="text-sm text-center">
                Battle initialized! Two models have been assigned.
                <br />
                <span className="text-xs text-muted-foreground">
                  Models will be revealed after you vote
                </span>
              </p>
            </Card>
            <InstructionInput onSubmit={handleInstructionSubmit} />
          </div>
        );

      case "executing":
        return <ExecutionLoading instruction={currentInstruction} />;

      case "replaying":
      case "voting":
        return (
          <div className="space-y-6">
            {/* Episode viewers */}
            <div className="grid grid-cols-2 gap-4">
              {leftEpisode && (
                <MujocoViewer episode={leftEpisode} side="left" title="Model A" />
              )}
              {rightEpisode && (
                <MujocoViewer episode={rightEpisode} side="right" title="Model B" />
              )}
            </div>

            {/* Instruction display */}
            <Card className="p-3 bg-slate-50">
              <p className="text-sm">
                <span className="font-medium">Instruction: </span>
                {currentInstruction}
              </p>
            </Card>

            {/* Voting panel */}
            {battleState === "voting" && (
              <VotingPanel onVote={handleVote} />
            )}
          </div>
        );

      case "revealed":
        return (
          <div className="space-y-6">
            {/* Keep episode viewers visible */}
            <div className="grid grid-cols-2 gap-4">
              {leftEpisode && (
                <MujocoViewer episode={leftEpisode} side="left" title="Model A" />
              )}
              {rightEpisode && (
                <MujocoViewer episode={rightEpisode} side="right" title="Model B" />
              )}
            </div>

            {/* Model reveal */}
            {revealedModels && currentVote && (
              <ModelReveal
                revealedModels={revealedModels}
                voteResult={currentVote}
                onNewBattle={handleNewBattle}
              />
            )}
          </div>
        );

      case "error":
        return (
          <Alert variant="destructive">
            <p>{error}</p>
          </Alert>
        );

      default:
        return null;
    }
  };

  return (
    <div className="container max-w-6xl mx-auto py-8 space-y-6">
      {/* Header */}
      <div className="text-center space-y-2">
        <h1 className="text-3xl font-bold">VLA Arena - Battle Mode</h1>
        <p className="text-muted-foreground">
          Compare two Vision-Language-Action models in blind A/B testing
        </p>
      </div>

      {/* Battle info */}
      {session && (
        <Card className="p-3">
          <div className="flex items-center justify-between text-sm">
            <div className="flex gap-4">
              <span className="text-muted-foreground">
                Session: <span className="font-mono">{session.session_id.substring(0, 12)}...</span>
              </span>
              <span className="text-muted-foreground">
                Robot: <span className="font-medium">Franka Panda</span>
              </span>
              <span className="text-muted-foreground">
                Scene: <span className="font-medium">Table</span>
              </span>
            </div>
          </div>
        </Card>
      )}

      {/* Main content */}
      {renderContent()}

      {/* Error display */}
      {error && battleState !== "error" && (
        <Alert variant="destructive">
          <p className="text-sm">{error}</p>
        </Alert>
      )}
    </div>
  );
}
