"use client";

import { MessageSquare } from "lucide-react";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";
import type { SessionItem } from "@/lib/services/session-service";

interface SessionItemProps {
  session: SessionItem;
  isActive: boolean;
  onClick: (sessionId: string) => void;
}

export function SessionItemComponent({
  session,
  isActive,
  onClick,
}: SessionItemProps) {
  return (
    <Button
      variant={isActive ? "secondary" : "ghost"}
      className={cn(
        "w-full justify-start gap-2 h-auto py-2 px-3 text-left",
        "hover:bg-accent/50 transition-colors"
      )}
      onClick={() => onClick(session.session_id)}
    >
      <MessageSquare className="h-4 w-4 shrink-0" />
      <div className="flex-1 overflow-hidden">
        <p className="text-sm font-medium truncate">{session.title}</p>
        <p className="text-xs text-muted-foreground">
          {new Date(session.last_active_at).toLocaleDateString()}
        </p>
      </div>
    </Button>
  );
}
