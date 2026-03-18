import { type JSX } from "react";

import type { SourceFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { ExplorerFilters } from "@/components/browser/ExplorerFilters";
import { RecordSearch } from "@/components/browser/RecordSearch";
import { RecordList } from "@/components/browser/RecordList";

const filters: { value: SourceFilter; label: string }[] = [
  { value: "workbench", label: "Workbench" },
  { value: "dataset", label: "Dataset" },
];

export function RecordBrowser(): JSX.Element {
  const { sourceFilter, loading, error } = useViewer();
  const dispatch = useViewerDispatch();

  return (
    <div className="flex h-full min-h-0 min-w-0 flex-col">
      {/* Tab bar */}
      <div className="flex items-center border-b border-[var(--border-default)] px-4">
        <div className="flex gap-0">
          {filters.map((f) => (
            <button
              key={f.value}
              type="button"
              onClick={() =>
                dispatch({ type: "SET_SOURCE_FILTER", payload: f.value })
              }
              className={`relative px-3 py-3 text-[11px] font-medium tracking-[0.01em] transition-colors duration-150 ${
                sourceFilter === f.value
                  ? "text-[var(--text-primary)] after:absolute after:bottom-0 after:left-3 after:right-3 after:h-[1.5px] after:rounded-full after:bg-[var(--text-primary)]"
                  : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
              }`}
            >
              {f.label}
            </button>
          ))}
        </div>
      </div>

      {/* Search + filters */}
      <div className="space-y-2 px-3 py-3">
        <RecordSearch />
        <ExplorerFilters />
      </div>

      {error ? (
        <div className="border-b border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-4 py-2">
          <p className="text-[11px] text-[var(--destructive)]">{error}</p>
        </div>
      ) : null}

      {loading ? (
        <div className="flex flex-1 items-center justify-center">
          <p className="text-[11px] text-[var(--text-quaternary)]">Loading…</p>
        </div>
      ) : (
        <RecordList />
      )}
    </div>
  );
}
