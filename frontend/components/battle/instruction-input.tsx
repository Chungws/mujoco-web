/**
 * Instruction Input Component
 * Allows users to provide natural language instructions for VLA models
 */

"use client";

import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Textarea } from "@/components/ui/textarea";
import { Card } from "@/components/ui/card";

interface InstructionInputProps {
  onSubmit: (instruction: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

export default function InstructionInput({
  onSubmit,
  disabled = false,
  placeholder = "Enter instruction (e.g., 'Pick up the red cube')",
}: InstructionInputProps) {
  const [instruction, setInstruction] = useState("");

  const handleSubmit = () => {
    if (instruction.trim()) {
      onSubmit(instruction.trim());
      setInstruction("");
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <Card className="p-4">
      <div className="space-y-3">
        <label className="text-sm font-medium">
          Instruction
        </label>
        <Textarea
          value={instruction}
          onChange={(e) => setInstruction(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          disabled={disabled}
          rows={3}
          className="resize-none"
        />
        <div className="flex justify-end">
          <Button
            onClick={handleSubmit}
            disabled={disabled || !instruction.trim()}
          >
            Execute Models
          </Button>
        </div>
      </div>
    </Card>
  );
}
