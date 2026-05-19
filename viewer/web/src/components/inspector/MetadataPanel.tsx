import { useEffect, useState, type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import {
  fetchRecordFile,
  fetchRecordHistory,
  fetchRecordTraceFile,
  fetchStagingFile,
  fetchStagingTraceFile,
} from "@/lib/api";
import type { RecordHistory, RecordHistoryRevision, RecordSummary } from "@/lib/types";
import { findStagingEntryInBootstrap } from "@/lib/record-summary";
import { TracePanel } from "@/components/inspector/TracePanel";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Skeleton } from "@/components/ui/skeleton";

function statusVariant(status: string | null | undefined): "success" | "warning" | "destructive" | "secondary" {
  const normalized = (status ?? "").toLowerCase();
  if (normalized === "success" || normalized === "materialized") return "success";
  if (normalized === "running" || normalized === "pending") return "success";
  if (normalized === "failed") return "destructive";
  return "secondary";
}

function formatDate(value: string | null | undefined): string {
  if (!value) return "--";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return date.toLocaleString();
}

function SectionLabel({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <div className="flex items-center gap-2 pb-2">
      <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">{children}</span>
      <div className="h-px flex-1 bg-[var(--border-subtle)]" />
    </div>
  );
}

function externalAgentLabel(agent: string | null | undefined): string {
  if (agent === "codex") return "Codex";
  if (agent === "claude-code") return "Claude Code";
  return agent ?? "External";
}

function formatCost(value: number | null | undefined): string {
  return value != null ? `$${value.toFixed(4)}` : "--";
}

function HistoryRevisionRow({ row }: { row: RecordHistoryRevision }): JSX.Element {
  const dispatch = useViewerDispatch();
  const modelLabel = [row.provider, row.model_id].filter(Boolean).join(" / ");
  const statusLabel = row.status && row.status !== "unknown" ? row.status : null;
  const costLabel = row.total_cost_usd != null ? formatCost(row.total_cost_usd) : null;
  const promptLabel = row.prompt_preview?.trim() || null;

  return (
    <li className="border-t border-[var(--border-subtle)] first:border-t-0">
      <button
        type="button"
        onClick={() => dispatch({ type: "SELECT_RECORD", payload: row.record_id })}
        className="-mx-2 block w-full rounded px-2 py-2 text-left transition-colors hover:bg-[var(--surface-2)]"
        aria-label={`Select ${row.record_id}`}
      >
        <div className="flex items-start justify-between gap-2">
          <span
            className="min-w-0 flex-1 font-mono text-[10px] text-[var(--text-secondary)]"
            style={{ overflowWrap: "anywhere", wordBreak: "break-word" }}
          >
            {row.record_id}
          </span>
          {costLabel ? (
            <span className="shrink-0 font-mono text-[10px] text-[var(--text-tertiary)]">
              {costLabel}
            </span>
          ) : null}
        </div>
        {promptLabel ? (
          <p className="mt-1 line-clamp-2 text-[11px] text-[var(--text-secondary)]">
            {promptLabel}
          </p>
        ) : null}
        {modelLabel || statusLabel ? (
          <div className="mt-1 flex items-center justify-between gap-2 text-[10px] text-[var(--text-tertiary)]">
            {modelLabel ? <span className="min-w-0 truncate">{modelLabel}</span> : <span />}
            {statusLabel ? <span className="shrink-0">{statusLabel}</span> : null}
          </div>
        ) : null}
      </button>
    </li>
  );
}

function HistoryPanel({
  history,
  selectedRecord,
}: {
  history: RecordHistory | null;
  selectedRecord: RecordSummary;
}): JSX.Element | null {
  const dispatch = useViewerDispatch();
  if (!history) return null;
  const hasAncestors = history.ancestors.length > 0;
  const hasDescendants = history.descendants.length > 0;
  const hasOrigin =
    !!selectedRecord.origin_record_id &&
    selectedRecord.origin_record_id !== selectedRecord.record_id;
  if (!hasAncestors && !hasDescendants && !hasOrigin) return null;

  return (
    <section>
      <SectionLabel>History</SectionLabel>
      <div className="space-y-3">
        {hasOrigin ? (
          <div className="prop-row-stacked">
            <span className="prop-label">Origin</span>
            <span className="prop-value font-mono text-[10px]">
              {selectedRecord.origin_record_id}
            </span>
          </div>
        ) : null}

        {hasAncestors ? (
          <div>
            <p className="pb-1 text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">
              Ancestors
            </p>
            <ul>
              {history.ancestors.map((row) => (
                <HistoryRevisionRow
                  key={`${row.record_id}:${row.revision_id}:ancestor`}
                  row={row}
                />
              ))}
            </ul>
          </div>
        ) : null}

        {hasDescendants ? (
          <div>
            <p className="pb-1 text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">
              Descendants
            </p>
            <ul>
              {history.descendants.map((descendant) => (
                <li
                  key={descendant.record_id}
                  className="border-t border-[var(--border-subtle)] first:border-t-0"
                >
                  <button
                    type="button"
                    onClick={() =>
                      dispatch({ type: "SELECT_RECORD", payload: descendant.record_id })
                    }
                    className="-mx-2 block w-full rounded px-2 py-2 text-left transition-colors hover:bg-[var(--surface-2)]"
                    aria-label={`Select ${descendant.record_id}`}
                  >
                    <span
                      className="block font-mono text-[10px] text-[var(--text-secondary)]"
                      style={{ overflowWrap: "anywhere", wordBreak: "break-word" }}
                    >
                      {descendant.record_id}
                    </span>
                    {descendant.title ? (
                      <p className="mt-1 line-clamp-2 text-[11px] text-[var(--text-secondary)]">
                        {descendant.title}
                      </p>
                    ) : null}
                  </button>
                </li>
              ))}
            </ul>
          </div>
        ) : null}
      </div>
    </section>
  );
}

export function MetadataPanel(): JSX.Element {
  const { bootstrap, selectedRecordId, selectedRecordSummary, selection } = useViewer();
  const [compileReport, setCompileReport] = useState<Record<string, unknown> | null>(null);
  const [cost, setCost] = useState<Record<string, unknown> | null>(null);
  const [traceText, setTraceText] = useState<string | null>(null);
  const [history, setHistory] = useState<RecordHistory | null>(null);
  const [loadingExtras, setLoadingExtras] = useState(false);

  const isStaging = selection?.kind === "staging";
  const stagingEntry = isStaging
    ? findStagingEntryInBootstrap(bootstrap, selection.runId, selection.recordId)
    : null;
  const record = selection?.kind === "record" ? selectedRecordSummary : null;
  const compileStatus =
    typeof compileReport?.status === "string" ? compileReport.status : record?.has_compile_report ? "available" : null;
  const stagingSelectionKey =
    selection?.kind === "staging" ? `${selection.runId}:${selection.recordId}` : null;
  const recordSelectionKey = selection?.kind === "record" ? selection.recordId : null;
  const hasSelectedRecord = Boolean(selectedRecordId && record);
  const recordHasCompileReport = record?.has_compile_report ?? false;
  const recordHasCost = record?.has_cost ?? false;
  const recordHasTraces = record?.has_traces ?? false;
  const isExternalRecord = record?.creator_mode === "external_agent";
  const stagingHasCost = stagingEntry?.has_cost ?? false;
  const stagingHasTraces = stagingEntry?.has_traces ?? false;
  const stagingRecordId = stagingEntry?.record_id ?? null;
  const stagingRunId = stagingEntry?.run_id ?? null;

  useEffect(() => {
    // Staging metadata loading
    if (isStaging && stagingRunId && stagingRecordId) {
      let cancelled = false;
      setLoadingExtras(true);
      setCompileReport(null);

      const promises: Promise<void>[] = [];

      if (stagingHasCost) {
        promises.push(
          fetchStagingFile(stagingRunId, stagingRecordId, "cost.json")
            .then((text) => {
              if (!cancelled) setCost(JSON.parse(text) as Record<string, unknown>);
            })
            .catch(() => {
              if (!cancelled) setCost(null);
            }),
        );
      } else {
        setCost(null);
      }

      if (stagingHasTraces) {
        promises.push(
          fetchStagingTraceFile(stagingRunId, stagingRecordId, "trajectory.jsonl")
            .then((text) => {
              if (!cancelled) setTraceText(text);
            })
            .catch(() => {
              if (!cancelled) setTraceText(null);
            }),
        );
      } else {
        setTraceText(null);
      }

      Promise.all(promises).finally(() => {
        if (!cancelled) setLoadingExtras(false);
      });

      return () => { cancelled = true; };
    }

    // Record metadata loading
    if (!selectedRecordId || !hasSelectedRecord) {
      setCompileReport(null);
      setCost(null);
      setTraceText(null);
      setHistory(null);
      setLoadingExtras(false);
      return;
    }

    let cancelled = false;
    setLoadingExtras(true);

    const promises: Promise<void>[] = [];

    if (recordHasCompileReport) {
      promises.push(
        fetchRecordFile(selectedRecordId, "compile_report.json")
          .then((text) => {
            if (!cancelled) setCompileReport(JSON.parse(text) as Record<string, unknown>);
          })
          .catch(() => {
            if (!cancelled) setCompileReport(null);
          }),
      );
    } else {
      setCompileReport(null);
    }

    if (recordHasCost) {
      promises.push(
        fetchRecordFile(selectedRecordId, "cost.json")
          .then((text) => {
            if (!cancelled) setCost(JSON.parse(text) as Record<string, unknown>);
          })
          .catch(() => {
            if (!cancelled) setCost(null);
          }),
      );
    } else {
      setCost(null);
    }

    if (recordHasTraces) {
      promises.push(
        fetchRecordTraceFile(selectedRecordId, "trajectory.jsonl")
          .then((text) => {
            if (!cancelled) setTraceText(text);
          })
          .catch(() => {
            if (!cancelled) setTraceText(null);
          }),
      );
    } else {
      setTraceText(null);
    }

    Promise.all(promises).finally(() => {
      if (!cancelled) setLoadingExtras(false);
    });

    return () => {
      cancelled = true;
    };
  }, [
    hasSelectedRecord,
    isStaging,
    recordHasCompileReport,
    recordHasCost,
    recordHasTraces,
    recordSelectionKey,
    selectedRecordId,
    stagingHasCost,
    stagingHasTraces,
    stagingRecordId,
    stagingRunId,
    stagingSelectionKey,
  ]);

  useEffect(() => {
    if (!selectedRecordId || !record) {
      setHistory(null);
      return;
    }

    let cancelled = false;
    fetchRecordHistory(selectedRecordId)
      .then((payload) => {
        if (!cancelled) setHistory(payload);
      })
      .catch(() => {
        if (!cancelled) setHistory(null);
      });
    return () => {
      cancelled = true;
    };
  }, [record, selectedRecordId]);

  if (!selection) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">Select a record</p>
      </div>
    );
  }

  // ── Staging metadata view ──
  if (isStaging && stagingEntry) {
    return (
      <ScrollArea className="h-full">
        <div className="space-y-5 pb-3">
          <section>
            <SectionLabel>Identity</SectionLabel>
            <div className="space-y-0">
              <div className="prop-row-stacked">
                <span className="prop-label">Run ID</span>
                <span className="prop-value font-mono text-[10px]">{stagingEntry.run_id}</span>
              </div>
              <div className="prop-row-stacked">
                <span className="prop-label">Record ID</span>
                <span className="prop-value font-mono text-[10px]">{stagingEntry.record_id}</span>
              </div>
              <div className="prop-row-stacked">
                <span className="prop-label">Staging Dir</span>
                <span className="prop-value font-mono text-[10px]">{stagingEntry.staging_dir}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Created</span>
                <span className="prop-value">{formatDate(stagingEntry.created_at)}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Updated</span>
                <span className="prop-value">{formatDate(stagingEntry.updated_at)}</span>
              </div>
            </div>
          </section>

          <section>
            <SectionLabel>Status</SectionLabel>
            <div className="flex flex-wrap items-center gap-1.5">
              <Badge variant={statusVariant(stagingEntry.status)}>
                {stagingEntry.status ?? "unknown"}
              </Badge>
            </div>
            {stagingEntry.message ? (
              <p className="mt-2 text-[11px] text-[var(--text-secondary)]">{stagingEntry.message}</p>
            ) : null}
          </section>

          <section>
            <SectionLabel>Metrics</SectionLabel>
            <div className="space-y-0">
              <div className="prop-row">
                <span className="prop-label">Turns</span>
                <span className="prop-value font-mono">{stagingEntry.turn_count != null ? String(stagingEntry.turn_count) : "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Tool Calls</span>
                <span className="prop-value font-mono">{stagingEntry.tool_call_count != null ? String(stagingEntry.tool_call_count) : "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Compile Attempts</span>
                <span className="prop-value font-mono">{stagingEntry.compile_attempt_count != null ? String(stagingEntry.compile_attempt_count) : "--"}</span>
              </div>
            </div>
          </section>

          {loadingExtras && (
            <div className="space-y-1.5">
              <Skeleton className="h-3 w-24" />
              <Skeleton className="h-14 w-full" />
            </div>
          )}

          {!loadingExtras && <TracePanel cost={cost} traceText={traceText} />}
        </div>
      </ScrollArea>
    );
  }

  // ── Record metadata view ──
  if (!selectedRecordId) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">Select a record</p>
      </div>
    );
  }

  if (!record) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">Loading record…</p>
      </div>
    );
  }

  return (
    <ScrollArea className="h-full">
      <div className="space-y-5 pb-3">
        {/* Identity */}
        <section>
          <SectionLabel>Identity</SectionLabel>
          <div className="space-y-0">
            <div className="prop-row-stacked">
              <span className="prop-label">Record ID</span>
              <span className="prop-value font-mono text-[10px]">{record.record_id}</span>
            </div>
            <div className="prop-row-stacked">
              <span className="prop-label">Run ID</span>
              <span className="prop-value font-mono text-[10px]">{record.run_id ?? "--"}</span>
            </div>
            <div className="prop-row">
              <span className="prop-label">Created</span>
              <span className="prop-value">{formatDate(record.created_at)}</span>
            </div>
            <div className="prop-row">
              <span className="prop-label">Updated</span>
              <span className="prop-value">{formatDate(record.updated_at)}</span>
            </div>
          </div>
        </section>

        {/* Status */}
        <section>
          <SectionLabel>Status</SectionLabel>
          <div className="flex flex-wrap items-center gap-1.5">
            <Badge variant={statusVariant(record.run_status)}>Run: {record.run_status ?? "unknown"}</Badge>
            <Badge variant={statusVariant(compileStatus)}>Compile: {compileStatus ?? "unknown"}</Badge>
            <Badge variant={statusVariant(record.materialization_status)}>
              Assets: {record.materialization_status ?? "unknown"}
            </Badge>
            {isExternalRecord ? (
              <Badge variant="secondary">{externalAgentLabel(record.external_agent)}</Badge>
            ) : null}
          </div>
          {record.run_message ? (
            <p className="mt-2 text-[11px] text-[var(--text-secondary)]">{record.run_message}</p>
          ) : null}
        </section>

        {/* Metrics */}
        <section>
          <SectionLabel>Metrics</SectionLabel>
          <div className="space-y-0">
            <div className="prop-row">
              <span className="prop-label">Collections</span>
              <span className="prop-value">{record.collections.join(", ") || "--"}</span>
            </div>
            <div className="prop-row">
              <span className="prop-label">Turns</span>
              <span className="prop-value font-mono">{record.turn_count != null ? String(record.turn_count) : "--"}</span>
            </div>
            <div className="prop-row">
              <span className="prop-label">Cost</span>
              <span className="prop-value font-mono">{record.total_cost_usd != null ? `$${record.total_cost_usd.toFixed(4)}` : "--"}</span>
            </div>
            <div className="prop-row">
              <span className="prop-label">Assets</span>
              <span className="prop-value">{record.materialization_status ?? "unknown"}</span>
            </div>
          </div>
        </section>

        <HistoryPanel history={history} selectedRecord={record} />

        {loadingExtras && (
          <div className="space-y-1.5">
            <Skeleton className="h-3 w-24" />
            <Skeleton className="h-14 w-full" />
          </div>
        )}

        {/* Compile Report */}
        {!loadingExtras && compileReport && (
          <section>
            <SectionLabel>Compile Report</SectionLabel>
            <div className="space-y-0">
              {Object.entries(compileReport).map(([key, val]) => (
                <div key={key} className="prop-row">
                  <span className="prop-label font-mono">{key}</span>
                  <span className="prop-value font-mono text-[10px]">
                    {typeof val === "object" ? JSON.stringify(val) : String(val)}
                  </span>
                </div>
              ))}
            </div>
          </section>
        )}

        {!loadingExtras && isExternalRecord && !recordHasTraces ? (
          <section>
            <SectionLabel>Agent Trace</SectionLabel>
            <p className="text-[11px] text-[var(--text-quaternary)]">No Articraft agent trace</p>
          </section>
        ) : null}

        {!loadingExtras && (!isExternalRecord || recordHasTraces) ? (
          <TracePanel cost={cost} traceText={traceText} record={record} />
        ) : null}
      </div>
    </ScrollArea>
  );
}
