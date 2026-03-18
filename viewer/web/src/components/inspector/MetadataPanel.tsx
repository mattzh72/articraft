import { useEffect, useState, type JSX } from "react";

import { useViewer } from "@/lib/viewer-context";
import { fetchRecordFile, fetchRecordTraceFile } from "@/lib/api";
import { findRecordInBootstrap, findRunInBootstrap } from "@/lib/record-summary";
import { TracePanel } from "@/components/inspector/TracePanel";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Skeleton } from "@/components/ui/skeleton";

function statusVariant(status: string | null | undefined): "success" | "warning" | "destructive" | "secondary" {
  const normalized = (status ?? "").toLowerCase();
  if (normalized === "success" || normalized === "materialized") return "success";
  if (normalized === "running" || normalized === "pending") return "warning";
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

export function MetadataPanel(): JSX.Element {
  const { bootstrap, selectedRecordId } = useViewer();
  const [compileReport, setCompileReport] = useState<Record<string, unknown> | null>(null);
  const [cost, setCost] = useState<Record<string, unknown> | null>(null);
  const [traceText, setTraceText] = useState<string | null>(null);
  const [loadingExtras, setLoadingExtras] = useState(false);

  const record = selectedRecordId ? findRecordInBootstrap(bootstrap, selectedRecordId) : null;
  const run = findRunInBootstrap(bootstrap, record?.run_id);
  const compileStatus =
    typeof compileReport?.status === "string" ? compileReport.status : record?.has_compile_report ? "available" : null;

  useEffect(() => {
    if (!selectedRecordId || !record) {
      setCompileReport(null);
      setCost(null);
      setTraceText(null);
      return;
    }

    let cancelled = false;
    setLoadingExtras(true);

    const promises: Promise<void>[] = [];

    if (record.has_compile_report) {
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

    if (record.has_cost) {
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

    promises.push(
      fetchRecordTraceFile(selectedRecordId, "conversation.jsonl")
        .then((text) => {
          if (!cancelled) setTraceText(text);
        })
        .catch(() => {
          if (!cancelled) setTraceText(null);
        }),
    );

    Promise.all(promises).finally(() => {
      if (!cancelled) setLoadingExtras(false);
    });

    return () => {
      cancelled = true;
    };
  }, [selectedRecordId, record]);

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
        <p className="text-[11px] text-[var(--text-quaternary)]">Record not found</p>
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
            <div className="prop-row">
              <span className="prop-label">Record ID</span>
              <span className="prop-value font-mono text-[10px]">{record.record_id}</span>
            </div>
            <div className="prop-row">
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
            <Badge variant={statusVariant(run?.status)}>Run: {run?.status ?? "unknown"}</Badge>
            <Badge variant={statusVariant(compileStatus)}>Compile: {compileStatus ?? "unknown"}</Badge>
            <Badge variant={statusVariant(record.materialization_status)}>
              Assets: {record.materialization_status ?? "unknown"}
            </Badge>
          </div>
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

        {!loadingExtras && <TracePanel cost={cost} traceText={traceText} />}
      </div>
    </ScrollArea>
  );
}
