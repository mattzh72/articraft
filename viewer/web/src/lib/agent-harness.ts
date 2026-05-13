import type { AgentHarness, RecordSummary } from "@/lib/types";

export const AGENT_HARNESSES = ["articraft", "codex", "claude-code"] as const satisfies readonly AgentHarness[];

const AGENT_HARNESS_LABELS: Record<AgentHarness, string> = {
  articraft: "Articraft",
  codex: "Codex",
  "claude-code": "Claude Code",
};

export function agentHarnessLabel(value: string): string {
  return AGENT_HARNESS_LABELS[value as AgentHarness] ?? value;
}

export function normalizeAgentHarnessFilters(values: string[]): AgentHarness[] {
  return Array.from(
    new Set(
      values
        .map((value) => value.trim())
        .filter((value): value is AgentHarness =>
          AGENT_HARNESSES.includes(value as AgentHarness),
        ),
    ),
  );
}

export function sortAgentHarnesses(values: string[]): AgentHarness[] {
  const order = new Map<AgentHarness, number>(AGENT_HARNESSES.map((value, index) => [value, index]));
  return normalizeAgentHarnessFilters(values).sort(
    (left, right) => (order.get(left) ?? AGENT_HARNESSES.length) - (order.get(right) ?? AGENT_HARNESSES.length),
  );
}

export function recordMatchesAgentHarnessFilters(
  record: RecordSummary,
  filters: AgentHarness[],
): boolean {
  if (filters.length === 0) return true;
  return filters.includes(record.agent_harness ?? "articraft");
}
