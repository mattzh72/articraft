import { lazy, Suspense, useMemo, useState, type JSX } from "react";
import { Maximize2 } from "lucide-react";

import type { RecordSummary } from "@/lib/types";
import {
  asRecord,
  formatUsd,
  formatCount,
  overallTraceTotals,
  parseEnrichedTurns,
} from "./trajectory-types";

const TrajectoryInspectorModal = lazy(() =>
  import("./TrajectoryInspectorModal").then((module) => ({
    default: module.TrajectoryInspectorModal,
  })),
);

type TracePanelProps = {
  cost: Record<string, unknown> | null;
  traceText: string | null;
  record?: RecordSummary | null;
};

export function TracePanel({ cost, traceText, record = null }: TracePanelProps): JSX.Element | null {
  const total = overallTraceTotals(cost);
  const totalTokens = asRecord(total?.tokens);
  const totalCosts = asRecord(total?.costs_usd);
  const parsedTurns = useMemo(() => parseEnrichedTurns(traceText, cost), [traceText, cost]);
  const [inspectorOpen, setInspectorOpen] = useState(false);

  if (!cost && !traceText && !record) return null;

  const modelId =
    (typeof cost?.model_id === "string" ? cost.model_id : null) ?? record?.model_id ?? null;

  const totalCostDisplay =
    totalCosts?.total != null
      ? formatUsd(totalCosts.total)
      : record?.total_cost_usd != null
        ? `$${record.total_cost_usd.toFixed(4)}`
        : "--";

  const totalTokensDisplay =
    totalTokens?.total_tokens != null
      ? formatCount(totalTokens.total_tokens)
      : record?.input_tokens != null && record?.output_tokens != null
        ? formatCount(record.input_tokens + record.output_tokens)
        : "--";

  const turnCountDisplay =
    parsedTurns.length > 0
      ? String(parsedTurns.length)
      : record?.turn_count != null
        ? String(record.turn_count)
        : "--";

  const hasTrajectory = Boolean(cost || traceText);

  return (
    <section>
      <div className="flex items-center gap-2 pb-2">
        <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">
          Agent Trace
        </span>
        <div className="h-px flex-1 bg-[var(--border-subtle)]" />
      </div>

      <div className="space-y-0">
        <div className="prop-row">
          <span className="prop-label">Model</span>
          <span className="prop-value font-mono text-[10px]">{modelId ?? "--"}</span>
        </div>
        <div className="prop-row">
          <span className="prop-label">Total Cost</span>
          <span className="prop-value font-mono">{totalCostDisplay}</span>
        </div>
        <div className="prop-row">
          <span className="prop-label">Total Tokens</span>
          <span className="prop-value font-mono">{totalTokensDisplay}</span>
        </div>
        <div className="prop-row">
          <span className="prop-label">Turns</span>
          <span className="prop-value font-mono">{turnCountDisplay}</span>
        </div>
      </div>

      {hasTrajectory ? (
        <button
          type="button"
          onClick={() => setInspectorOpen(true)}
          className="mt-2.5 inline-flex items-center gap-1 text-[11px] text-[var(--accent)] transition-colors duration-100 hover:text-[var(--text-primary)]"
        >
          <Maximize2 className="size-3" />
          View full trajectory
        </button>
      ) : null}

      {inspectorOpen ? (
        <Suspense fallback={null}>
          <TrajectoryInspectorModal
            open={inspectorOpen}
            onOpenChange={setInspectorOpen}
            cost={cost}
            traceText={traceText}
          />
        </Suspense>
      ) : null}
    </section>
  );
}
