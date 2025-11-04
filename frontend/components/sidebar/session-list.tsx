"use client";

import { useRouter, useSearchParams } from "next/navigation";
import { ScrollArea } from "@/components/ui/scroll-area";
import { SessionItemComponent } from "@/components/sidebar/session-item";
import { useSessionContext } from "@/lib/contexts/session-context";
import { useEffect } from "react";

export function SessionList() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const { sessions, loading, error, selectSession, activeSessionId } =
    useSessionContext();

  // Get session_id from URL query param
  const urlSessionId = searchParams.get("session_id");

  // Sync URL session_id with SessionContext
  useEffect(() => {
    if (urlSessionId !== activeSessionId) {
      selectSession(urlSessionId);
    }
  }, [urlSessionId, activeSessionId, selectSession]);

  const handleSessionClick = (sessionId: string) => {
    // Update SessionContext
    selectSession(sessionId);
    // Navigate to battle page with session_id query param
    router.push(`/battle?session_id=${sessionId}`);
  };

  if (loading) {
    return (
      <div className="px-2 py-4 text-center">
        <p className="text-xs text-muted-foreground">Loading sessions...</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className="px-2 py-4 text-center">
        <p className="text-xs text-destructive">{error}</p>
      </div>
    );
  }

  if (sessions.length === 0) {
    return (
      <div className="px-2 py-4 text-center">
        <p className="text-xs text-muted-foreground">No sessions yet</p>
      </div>
    );
  }

  return (
    <ScrollArea className="flex-1">
      <div className="space-y-1 px-2 pb-2">
        {sessions.map((session) => (
          <SessionItemComponent
            key={session.session_id}
            session={session}
            isActive={session.session_id === activeSessionId}
            onClick={handleSessionClick}
          />
        ))}
      </div>
    </ScrollArea>
  );
}
