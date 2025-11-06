/**
 * MuJoCo Viewer Component
 * Renders MuJoCo states using WASM (placeholder for now)
 *
 * TODO: Integrate actual MuJoCo WASM library for state-based replay
 * For MVP, this shows episode metadata and placeholder for 3D visualization
 */

"use client";

import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { EpisodeResponse } from "@/lib/api/types";

interface MujocoViewerProps {
  episode: EpisodeResponse;
  side: "left" | "right";
  title: string;
}

export default function MujocoViewer({
  episode,
  side,
  title,
}: MujocoViewerProps) {
  const stepCount = episode.states.length;
  const actionCount = episode.actions.length;

  return (
    <Card className="p-4">
      <div className="space-y-3">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-medium">{title}</h3>
          <Badge variant="outline">
            {stepCount} steps
          </Badge>
        </div>

        {/* Placeholder for MuJoCo 3D viewer */}
        <div className="aspect-video bg-slate-900 rounded-md flex items-center justify-center relative overflow-hidden">
          {/* Grid background */}
          <div
            className="absolute inset-0 opacity-10"
            style={{
              backgroundImage: `
                linear-gradient(to right, white 1px, transparent 1px),
                linear-gradient(to bottom, white 1px, transparent 1px)
              `,
              backgroundSize: "20px 20px",
            }}
          />

          {/* Center content */}
          <div className="relative z-10 text-center space-y-2">
            <div className="text-white text-sm font-mono">
              MuJoCo WASM Viewer
            </div>
            <div className="text-slate-400 text-xs">
              Episode: {episode.episode_id.substring(0, 8)}...
            </div>
            <div className="text-slate-500 text-xs">
              {stepCount} states • {actionCount} actions
            </div>
          </div>

          {/* Side indicator */}
          <div className="absolute top-2 right-2">
            <Badge variant={side === "left" ? "default" : "secondary"}>
              {side === "left" ? "A" : "B"}
            </Badge>
          </div>
        </div>

        {/* Episode info */}
        <div className="text-xs text-muted-foreground space-y-1">
          <div className="flex justify-between">
            <span>States:</span>
            <span className="font-mono">{stepCount}</span>
          </div>
          <div className="flex justify-between">
            <span>Actions:</span>
            <span className="font-mono">{actionCount}</span>
          </div>
          <div className="flex justify-between">
            <span>Duration:</span>
            <span className="font-mono">
              {(episode.states[episode.states.length - 1]?.time || 0).toFixed(2)}s
            </span>
          </div>
        </div>

        <p className="text-xs text-muted-foreground text-center pt-2">
          State-based replay (MuJoCo WASM integration pending)
        </p>
      </div>
    </Card>
  );
}
