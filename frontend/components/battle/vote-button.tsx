/**
 * Vote Button Component
 *
 * Individual voting button with hover state management
 */

import { Button } from "@/components/ui/button";
import type { VoteOption } from "@/app/battle/_types";

interface VoteButtonProps {
  label: string;
  voteOption: VoteOption;
  onClick: () => void;
  onHoverChange: (option: VoteOption | null) => void;
  disabled?: boolean;
  icon?: string;
}

export function VoteButton({
  label,
  voteOption,
  onClick,
  onHoverChange,
  disabled = false,
  icon,
}: VoteButtonProps) {
  return (
    <Button
      onClick={onClick}
      onMouseEnter={() => onHoverChange(voteOption)}
      onMouseLeave={() => onHoverChange(null)}
      onFocus={() => onHoverChange(voteOption)}
      onBlur={() => onHoverChange(null)}
      disabled={disabled}
      variant="outline"
      size="sm"
      aria-label={`Vote ${label}`}
      className="transition-colors"
    >
      {icon && <span className="mr-1">{icon}</span>}
      {label}
    </Button>
  );
}
