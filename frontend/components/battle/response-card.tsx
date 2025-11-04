/**
 * Response Card Component
 *
 * Displays assistant response with highlight border effects
 */

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";
import type { ConversationMessage } from "@/app/battle/_types";
import { cn } from "@/lib/utils";

interface ResponseCardProps {
  title: string;
  revealedModel?: string;
  userMessages: ConversationMessage[];
  assistantMessages: ConversationMessage[];
  highlight: "green" | "red" | "none";
}

export function ResponseCard({
  title,
  revealedModel,
  userMessages,
  assistantMessages,
  highlight,
}: ResponseCardProps) {
  const borderClass = cn(
    "transition-colors duration-150",
    {
      "border-green-500 border-2": highlight === "green",
      "border-red-500 border-2": highlight === "red",
    }
  );

  return (
    <Card className={borderClass}>
      <CardHeader>
        <CardTitle className="text-base font-semibold">
          {title}
          {revealedModel && (
            <span className="ml-2 text-xs font-normal text-muted-foreground">
              ({revealedModel})
            </span>
          )}
        </CardTitle>
      </CardHeader>
      <CardContent>
        <ScrollArea className="h-[400px] pr-4">
          <div className="space-y-6">
            {userMessages.map((userMsg, idx) => (
              <div key={`${title}-${idx}`} className="space-y-3">
                <div className="text-xs font-medium text-muted-foreground uppercase tracking-wide">
                  You
                </div>
                <div className="text-sm bg-accent/10 p-4 rounded-lg border border-accent/20">
                  {userMsg.content}
                </div>
                {assistantMessages[idx] && (
                  <>
                    <div className="text-xs font-medium text-muted-foreground uppercase tracking-wide mt-4">
                      {title}
                    </div>
                    <div className="text-sm p-4 rounded-lg bg-card border-2">
                      {assistantMessages[idx].content}
                    </div>
                  </>
                )}
              </div>
            ))}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  );
}
