/**
 * Battle Mode Client Component
 *
 * Displays all battles in a session as a continuous chat stream
 */

"use client";

import { useState, useEffect, useRef } from "react";
import { useSearchParams, useRouter } from "next/navigation";
import { useUser } from "@/lib/hooks/use-user";
import { useSessionDetail } from "@/lib/hooks/use-session-detail";
import { useSessionContext } from "@/lib/contexts/session-context";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Textarea } from "@/components/ui/textarea";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { Skeleton } from "@/components/ui/skeleton";
import * as service from "./service";
import type { VoteOption } from "./_types";
import { cn } from "@/lib/utils";

export default function BattleClient() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const sessionIdFromUrl = searchParams.get("session_id");
  const { userId } = useUser();
  const { refetchSessions } = useSessionContext();
  const { battles, loading: sessionLoading, refetch: refetchBattles } = useSessionDetail(sessionIdFromUrl);

  const [promptInput, setPromptInput] = useState("");
  const [loadingPrompt, setLoadingPrompt] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [hoveredVote, setHoveredVote] = useState<VoteOption | null>(null);
  const [showOtherResponses, setShowOtherResponses] = useState<Record<string, boolean>>({});

  const chatEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom function
  const scrollToBottom = () => {
    setTimeout(() => {
      chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, 100);
  };

  // Auto-scroll to bottom when battles change
  useEffect(() => {
    scrollToBottom();
  }, [battles.length]);

  // Auto-scroll when loading starts
  useEffect(() => {
    if (isLoading) {
      scrollToBottom();
    }
  }, [isLoading]);

  // Sort battles by created_at (oldest first, newest last)
  const sortedBattles = [...battles].sort((a, b) => {
    const timeA = new Date(a.created_at || 0).getTime();
    const timeB = new Date(b.created_at || 0).getTime();
    return timeA - timeB;
  });

  // Get current battle (latest ongoing battle)
  const currentBattle = sortedBattles.find((b) => b.status === "ongoing") || sortedBattles[sortedBattles.length - 1];
  const isCurrentOngoing = currentBattle?.status === "ongoing";

  /**
   * Create new session with initial prompt
   */
  const handleStartSession = async () => {
    if (!promptInput.trim()) return;

    setLoadingPrompt(promptInput);
    setIsLoading(true);
    setError(null);

    try {
      const response = await service.createSession(promptInput, userId);
      router.push(`/battle?session_id=${response.session_id}`);
      await refetchSessions();
      setPromptInput("");
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to create session");
    } finally {
      setIsLoading(false);
      setLoadingPrompt("");
    }
  };

  /**
   * Send message - either follow-up or new battle
   */
  const handleSendMessage = async () => {
    if (!promptInput.trim() || !sessionIdFromUrl) return;

    setLoadingPrompt(promptInput);
    setIsLoading(true);
    setError(null);

    try {
      if (isCurrentOngoing && currentBattle) {
        // Ongoing battle - send follow-up
        await service.sendFollowUp(currentBattle.battle_id, promptInput);
      } else {
        // Voted battle - create new battle
        await service.createBattle(sessionIdFromUrl, promptInput);
      }

      setPromptInput("");

      // Refetch battles to update UI
      await refetchBattles();

      // Scroll to bottom to show new message
      scrollToBottom();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to send message");
    } finally {
      setIsLoading(false);
      setLoadingPrompt("");
    }
  };

  /**
   * Submit vote
   */
  const handleVote = async (vote: VoteOption) => {
    if (!currentBattle) return;

    setIsLoading(true);
    setError(null);

    try {
      await service.submitVote(currentBattle.battle_id, vote);

      // Refetch battles to update UI
      await refetchBattles();

      // Scroll to bottom after vote
      scrollToBottom();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to submit vote");
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = async (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      if (!sessionIdFromUrl) {
        await handleStartSession();
      } else {
        await handleSendMessage();
      }
    }
  };

  const toggleOtherResponse = (battleId: string) => {
    setShowOtherResponses((prev) => ({
      ...prev,
      [battleId]: !prev[battleId],
    }));
  };

  return (
    <div className="flex flex-col h-full overflow-hidden">
      <div className="flex-none p-4 md:p-8">
        {/* Header */}
        <div className="max-w-7xl mx-auto space-y-2">
          <h1 className="text-3xl md:text-4xl font-bold">Battle Mode</h1>
          <p className="text-muted-foreground">
            Compare responses from two randomly selected models
          </p>
        </div>

        {/* Error Alert */}
        {error && (
          <Alert variant="destructive" className="max-w-7xl mx-auto mt-4">
            <AlertDescription>{error}</AlertDescription>
          </Alert>
        )}
      </div>

      <div className="flex-1 overflow-auto px-4 md:px-8 pb-6">
        <div className="max-w-7xl mx-auto space-y-6">
          {/* No Session - Initial Prompt */}
          {!sessionIdFromUrl && !isLoading && (
            <Card>
              <CardHeader>
                <CardTitle className="text-base font-semibold">
                  Start a New Battle
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <Textarea
                  placeholder="Enter your prompt to start comparing models..."
                  value={promptInput}
                  onChange={(e) => setPromptInput(e.target.value)}
                  onKeyDown={handleKeyDown}
                  className="min-h-[120px]"
                  disabled={isLoading}
                />
                <Button
                  onClick={handleStartSession}
                  disabled={!promptInput.trim() || isLoading}
                  className="w-full"
                >
                  {isLoading ? "Starting..." : "Start Battle"}
                </Button>
              </CardContent>
            </Card>
          )}

          {/* Loading Skeleton - Initial Session Creation */}
          {!sessionIdFromUrl && isLoading && loadingPrompt && (
            <div className="space-y-4">
              {/* User Message - Show actual message */}
              <div className="flex justify-end">
                <div className="max-w-[80%] bg-primary text-primary-foreground rounded-lg px-4 py-3">
                  <div className="text-sm">{loadingPrompt}</div>
                </div>
              </div>

              {/* AI Response Skeletons - Side by side */}
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <Card>
                  <CardContent className="pt-4 space-y-3">
                    <Skeleton className="h-4 w-24" />
                    <div className="space-y-2">
                      <Skeleton className="h-4 w-full" />
                      <Skeleton className="h-4 w-full" />
                      <Skeleton className="h-4 w-3/4" />
                    </div>
                  </CardContent>
                </Card>
                <Card>
                  <CardContent className="pt-4 space-y-3">
                    <Skeleton className="h-4 w-24" />
                    <div className="space-y-2">
                      <Skeleton className="h-4 w-full" />
                      <Skeleton className="h-4 w-full" />
                      <Skeleton className="h-4 w-3/4" />
                    </div>
                  </CardContent>
                </Card>
              </div>
            </div>
          )}

          {/* Active Session - Continuous Chat Stream */}
          {sessionIdFromUrl && (
            <div className="space-y-8">
              {sessionLoading ? (
                <div className="text-center text-muted-foreground">
                  Loading battles...
                </div>
              ) : sortedBattles.length === 0 ? (
                <div className="text-center text-muted-foreground">
                  No battles yet
                </div>
              ) : (
                sortedBattles.map((battle) => {
                  const isOngoing = battle.status === "ongoing";
                  const isVoted = battle.status === "voted";
                  const showOther = showOtherResponses[battle.battle_id] || false;

                  // Determine visible side for voted battles
                  const getVisibleSide = () => {
                    if (isOngoing) return "both";
                    if (!battle.vote) return "both";
                    if (showOther) return "both";
                    if (battle.vote === "left_better") return "left";
                    if (battle.vote === "right_better") return "right";
                    return "both"; // tie or both_bad
                  };

                  const visibleSide = getVisibleSide();
                  const canToggle = isVoted && (battle.vote === "left_better" || battle.vote === "right_better");

                  // Group messages
                  const messages = battle.conversation;
                  const userMessages = messages.filter((m) => m.role === "user");

                  return (
                    <div key={battle.battle_id} className="space-y-6">
                      {userMessages.map((userMsg, idx) => {
                        const leftMsg = messages.find(
                          (m) => m.role === "assistant" && m.position === "left" && messages.indexOf(m) > messages.indexOf(userMsg)
                        );
                        const rightMsg = messages.find(
                          (m) => m.role === "assistant" && m.position === "right" && messages.indexOf(m) > messages.indexOf(userMsg)
                        );

                        return (
                          <div key={`${battle.battle_id}-msg-${idx}`} className="space-y-4">
                            {/* User Message - Right aligned */}
                            <div className="flex justify-end">
                              <div className="max-w-[80%] bg-primary text-primary-foreground rounded-lg px-4 py-3">
                                <div className="text-sm">{userMsg.content}</div>
                              </div>
                            </div>

                            {/* Model Responses */}
                            {isOngoing ? (
                              /* Ongoing Battle - Side by side comparison */
                              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                                {leftMsg && (
                                  <Card
                                    className={cn(
                                      "transition-colors duration-150",
                                      hoveredVote === "left_better" && "border-green-500 border-2",
                                      hoveredVote === "tie" && "border-green-500 border-2",
                                      hoveredVote === "both_bad" && "border-red-500 border-2"
                                    )}
                                  >
                                    <CardContent className="pt-4">
                                      <div className="text-xs font-medium text-muted-foreground mb-2">
                                        Assistant A
                                      </div>
                                      <div className="text-sm whitespace-pre-wrap">{leftMsg.content}</div>
                                    </CardContent>
                                  </Card>
                                )}
                                {rightMsg && (
                                  <Card
                                    className={cn(
                                      "transition-colors duration-150",
                                      hoveredVote === "right_better" && "border-green-500 border-2",
                                      hoveredVote === "tie" && "border-green-500 border-2",
                                      hoveredVote === "both_bad" && "border-red-500 border-2"
                                    )}
                                  >
                                    <CardContent className="pt-4">
                                      <div className="text-xs font-medium text-muted-foreground mb-2">
                                        Assistant B
                                      </div>
                                      <div className="text-sm whitespace-pre-wrap">{rightMsg.content}</div>
                                    </CardContent>
                                  </Card>
                                )}
                              </div>
                            ) : (
                              /* Voted Battle - Chat style or Side-by-side for tie/both_bad */
                              <>
                                {battle.vote === "tie" || battle.vote === "both_bad" ? (
                                  /* Tie or Both Bad - Side by side with borders */
                                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                                    {leftMsg && (
                                      <Card
                                        className={cn(
                                          "transition-colors duration-150",
                                          battle.vote === "tie" && "border-green-500 border-2",
                                          battle.vote === "both_bad" && "border-red-500 border-2"
                                        )}
                                      >
                                        <CardContent className="pt-4">
                                          <div className="text-xs font-medium text-muted-foreground mb-2">
                                            {battle.left_model_id}
                                          </div>
                                          <div className="text-sm whitespace-pre-wrap">{leftMsg.content}</div>
                                        </CardContent>
                                      </Card>
                                    )}
                                    {rightMsg && (
                                      <Card
                                        className={cn(
                                          "transition-colors duration-150",
                                          battle.vote === "tie" && "border-green-500 border-2",
                                          battle.vote === "both_bad" && "border-red-500 border-2"
                                        )}
                                      >
                                        <CardContent className="pt-4">
                                          <div className="text-xs font-medium text-muted-foreground mb-2">
                                            {battle.right_model_id}
                                          </div>
                                          <div className="text-sm whitespace-pre-wrap">{rightMsg.content}</div>
                                        </CardContent>
                                      </Card>
                                    )}
                                  </div>
                                ) : showOther ? (
                                  /* Left Better or Right Better - Show both side-by-side with colors */
                                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                                    {leftMsg && (
                                      <Card
                                        className={cn(
                                          "transition-colors duration-150",
                                          battle.vote === "left_better" && "border-green-500 border-2",
                                          battle.vote === "right_better" && "border-red-500 border-2"
                                        )}
                                      >
                                        <CardContent className="pt-4">
                                          <div className="text-xs font-medium text-muted-foreground mb-2">
                                            {battle.left_model_id}
                                          </div>
                                          <div className="text-sm whitespace-pre-wrap">{leftMsg.content}</div>
                                        </CardContent>
                                      </Card>
                                    )}
                                    {rightMsg && (
                                      <Card
                                        className={cn(
                                          "transition-colors duration-150",
                                          battle.vote === "right_better" && "border-green-500 border-2",
                                          battle.vote === "left_better" && "border-red-500 border-2"
                                        )}
                                      >
                                        <CardContent className="pt-4">
                                          <div className="text-xs font-medium text-muted-foreground mb-2">
                                            {battle.right_model_id}
                                          </div>
                                          <div className="text-sm whitespace-pre-wrap">{rightMsg.content}</div>
                                        </CardContent>
                                      </Card>
                                    )}
                                  </div>
                                ) : (
                                  /* Left Better or Right Better - Single chat style */
                                  <div className="space-y-2">
                                    {(visibleSide === "both" || visibleSide === "left") && leftMsg && (
                                      <div className="flex justify-start">
                                        <div className="max-w-[80%] bg-muted rounded-lg px-4 py-3">
                                          <div className="text-xs font-medium text-muted-foreground mb-1">
                                            {battle.left_model_id}
                                          </div>
                                          <div className="text-sm whitespace-pre-wrap">{leftMsg.content}</div>
                                        </div>
                                      </div>
                                    )}
                                    {(visibleSide === "both" || visibleSide === "right") && rightMsg && (
                                      <div className="flex justify-start">
                                        <div className="max-w-[80%] bg-muted rounded-lg px-4 py-3">
                                          <div className="text-xs font-medium text-muted-foreground mb-1">
                                            {battle.right_model_id}
                                          </div>
                                          <div className="text-sm whitespace-pre-wrap">{rightMsg.content}</div>
                                        </div>
                                      </div>
                                    )}
                                  </div>
                                )}
                              </>
                            )}
                          </div>
                        );
                      })}

                      {/* See other response button */}
                      {canToggle && (
                        <div className="flex justify-start">
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={() => toggleOtherResponse(battle.battle_id)}
                          >
                            {showOther ? "Hide" : "See"} other response
                          </Button>
                        </div>
                      )}
                    </div>
                  );
                })
              )}

              {/* Loading Skeleton - Show while AI is generating responses */}
              {isLoading && sessionIdFromUrl && loadingPrompt && (
                <div className="space-y-4">
                  {/* User Message - Show actual message */}
                  <div className="flex justify-end">
                    <div className="max-w-[80%] bg-primary text-primary-foreground rounded-lg px-4 py-3">
                      <div className="text-sm">{loadingPrompt}</div>
                    </div>
                  </div>

                  {/* AI Response Skeletons - Side by side */}
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <Card>
                      <CardContent className="pt-4 space-y-3">
                        <Skeleton className="h-4 w-24" />
                        <div className="space-y-2">
                          <Skeleton className="h-4 w-full" />
                          <Skeleton className="h-4 w-full" />
                          <Skeleton className="h-4 w-3/4" />
                        </div>
                      </CardContent>
                    </Card>
                    <Card>
                      <CardContent className="pt-4 space-y-3">
                        <Skeleton className="h-4 w-24" />
                        <div className="space-y-2">
                          <Skeleton className="h-4 w-full" />
                          <Skeleton className="h-4 w-full" />
                          <Skeleton className="h-4 w-3/4" />
                        </div>
                      </CardContent>
                    </Card>
                  </div>
                </div>
              )}

              {/* Auto-scroll anchor */}
              <div ref={chatEndRef} />
            </div>
          )}
        </div>
      </div>

      {/* Sticky Vote Buttons - Only for ongoing battle */}
      {sessionIdFromUrl && isCurrentOngoing && currentBattle && (
        <div className="flex-none border-t bg-background/95 backdrop-blur supports-backdrop-filter:bg-background/60">
          <div className="max-w-7xl mx-auto p-3 md:p-4">
            <div className="grid grid-cols-2 md:grid-cols-4 gap-2">
              <Button
                variant="outline"
                onClick={() => handleVote("left_better")}
                onMouseEnter={() => setHoveredVote("left_better")}
                onMouseLeave={() => setHoveredVote(null)}
                disabled={isLoading}
              >
                <span className="mr-2">👈</span>
                A is Better
              </Button>
              <Button
                variant="outline"
                onClick={() => handleVote("tie")}
                onMouseEnter={() => setHoveredVote("tie")}
                onMouseLeave={() => setHoveredVote(null)}
                disabled={isLoading}
              >
                <span className="mr-2">🤝</span>
                Tie
              </Button>
              <Button
                variant="outline"
                onClick={() => handleVote("both_bad")}
                onMouseEnter={() => setHoveredVote("both_bad")}
                onMouseLeave={() => setHoveredVote(null)}
                disabled={isLoading}
              >
                <span className="mr-2">👎</span>
                Both Bad
              </Button>
              <Button
                variant="outline"
                onClick={() => handleVote("right_better")}
                onMouseEnter={() => setHoveredVote("right_better")}
                onMouseLeave={() => setHoveredVote(null)}
                disabled={isLoading}
              >
                <span className="mr-2">👉</span>
                B is Better
              </Button>
            </div>
          </div>
        </div>
      )}

      {/* Sticky Bottom Input - Always "Ask followup..." */}
      {sessionIdFromUrl && (
        <div className="flex-none border-t bg-background/95 backdrop-blur supports-backdrop-filter:bg-background/60">
          <div className="max-w-7xl mx-auto p-3 md:p-4">
            <div className="flex gap-2">
              <Textarea
                placeholder="Ask followup..."
                value={promptInput}
                onChange={(e) => setPromptInput(e.target.value)}
                onKeyDown={handleKeyDown}
                className="min-h-[60px] resize-none"
                disabled={isLoading}
              />
              <Button
                onClick={handleSendMessage}
                disabled={!promptInput.trim() || isLoading}
                variant="secondary"
                className="shrink-0"
              >
                {isLoading ? "Sending..." : "Send"}
              </Button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
