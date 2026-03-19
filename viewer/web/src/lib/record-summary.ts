import type { RecordSummary, RunSummary, StagingEntry, ViewerBootstrap } from "@/lib/types";

export function findRecordInBootstrap(
  bootstrap: ViewerBootstrap | null,
  recordId: string,
): RecordSummary | null {
  if (!bootstrap) return null;

  for (const entry of bootstrap.workbench_entries) {
    if (entry.record_id === recordId && entry.record) return entry.record;
  }
  for (const entry of bootstrap.dataset_entries) {
    if (entry.record_id === recordId && entry.record) return entry.record;
  }
  return null;
}

export function findStagingEntryInBootstrap(
  bootstrap: ViewerBootstrap | null,
  runId: string,
  recordId: string,
): StagingEntry | null {
  if (!bootstrap) return null;
  return (
    bootstrap.staging_entries.find(
      (entry) => entry.run_id === runId && entry.record_id === recordId,
    ) ?? null
  );
}

export function findRunInBootstrap(
  bootstrap: ViewerBootstrap | null,
  runId: string | null | undefined,
): RunSummary | null {
  if (!bootstrap || !runId) return null;
  return bootstrap.runs.find((run) => run.run_id === runId) ?? null;
}
