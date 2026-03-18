import { type JSX } from "react";
import { RefreshCw } from "lucide-react";

import { fetchBootstrap } from "@/lib/api";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Button } from "@/components/ui/button";

const TRAILING_PUNCTUATION = /[\s,.;:!?)]*$/;

function truncateWithEllipsis(value: string, maxLength = 88): string {
  const normalized = value.replace(/\s+/g, " ").trim();
  if (!normalized) return "";

  const withoutExistingEllipsis = normalized.replace(/\.\.\.$/, "").trimEnd();
  if (withoutExistingEllipsis.length <= maxLength) {
    return withoutExistingEllipsis;
  }

  const truncated = withoutExistingEllipsis.slice(0, maxLength).trimEnd();
  return `${truncated.replace(TRAILING_PUNCTUATION, "")}...`;
}

export function AppHeader(): JSX.Element {
  const state = useViewer();
  const dispatch = useViewerDispatch();

  const selectedRecord = (() => {
    if (!state.bootstrap || !state.selectedRecordId) return null;

    for (const entry of state.bootstrap.workbench_entries) {
      if (entry.record_id === state.selectedRecordId && entry.record) {
        return entry.record;
      }
    }
    for (const entry of state.bootstrap.dataset_entries) {
      if (entry.record_id === state.selectedRecordId && entry.record) {
        return entry.record;
      }
    }
    return null;
  })();

  const selectedRecordTitleFull = selectedRecord?.title ?? null;
  const selectedRecordTitle = selectedRecord ? truncateWithEllipsis(selectedRecord.title, 72) : null;

  const handleRefresh = () => {
    dispatch({ type: "SET_LOADING", payload: true });
    fetchBootstrap()
      .then((data) => {
        dispatch({ type: "SET_BOOTSTRAP", payload: data });
      })
      .catch((err) => {
        dispatch({
          type: "SET_ERROR",
          payload: err instanceof Error ? err.message : "Failed to refresh.",
        });
      });
  };

  return (
    <header className="flex h-11 shrink-0 items-center gap-3 border-b border-[var(--border-default)] bg-[var(--surface-0)] px-4">
      <div className="flex items-center gap-2 text-[12px]">
        <span className="font-semibold tracking-[-0.02em] text-[var(--text-primary)]">Articraft</span>
        <span className="text-[var(--border-strong)]">/</span>
        <span className="text-[var(--text-tertiary)]">Viewer</span>
      </div>

      <div className="mx-1 flex min-w-0 flex-1 items-center justify-center">
        {selectedRecordTitle ? (
          <p
            className="max-w-full truncate text-[12px] text-[var(--text-secondary)]"
            title={selectedRecordTitleFull ?? undefined}
          >
            {selectedRecordTitle}
          </p>
        ) : (
          <p className="text-[12px] text-[var(--text-quaternary)]">No record selected</p>
        )}
      </div>

      <div className="flex items-center">
        <Button
          variant="ghost"
          size="sm"
          onClick={handleRefresh}
          disabled={state.loading}
          className="h-7 w-7 rounded-md p-0 text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
        >
          <RefreshCw
            className={`size-3.5 ${state.loading ? "animate-spin" : ""}`}
          />
        </Button>
      </div>
    </header>
  );
}
