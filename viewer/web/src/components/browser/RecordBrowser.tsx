import { useCallback, useState, type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { BulkActionBar } from "@/components/browser/BulkActionBar";
import { ExplorerFilters } from "@/components/browser/ExplorerFilters";
import { RecordSearch } from "@/components/browser/RecordSearch";
import { RecordList } from "@/components/browser/RecordList";
import { StagingList } from "@/components/browser/StagingList";

type BrowserCounts = {
  visible: number;
  total: number;
};

export function RecordBrowser(): JSX.Element {
  const { browserTab, loading, error, multiSelection } = useViewer();
  const dispatch = useViewerDispatch();
  const [visibleRecordIds, setVisibleRecordIds] = useState<string[]>([]);
  const [browserCounts, setBrowserCounts] = useState<BrowserCounts>({ visible: 0, total: 0 });
  const handleVisibleIdsChange = useCallback((ids: string[]) => setVisibleRecordIds(ids), []);
  const handleCountsChange = useCallback((counts: BrowserCounts) => setBrowserCounts(counts), []);

  return (
    <div className="flex h-full min-h-0 min-w-0 flex-col">
      <div className="flex items-center border-b border-[var(--border-default)] px-4">
        <div className="flex gap-0">
          <button
            type="button"
            onClick={() => {
              dispatch({ type: "SET_BROWSER_TAB", payload: "workbench" });
            }}
            className={`relative px-3 py-3 text-[11px] font-medium tracking-[0.01em] transition-colors duration-150 ${
              browserTab === "workbench"
                ? "text-[var(--text-primary)] after:absolute after:bottom-0 after:left-3 after:right-3 after:h-[1.5px] after:rounded-full after:bg-[var(--text-primary)]"
                : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
            }`}
          >
            Workbench
          </button>
          <button
            type="button"
            onClick={() => {
              dispatch({ type: "SET_BROWSER_TAB", payload: "dataset" });
            }}
            className={`relative px-3 py-3 text-[11px] font-medium tracking-[0.01em] transition-colors duration-150 ${
              browserTab === "dataset"
                ? "text-[var(--text-primary)] after:absolute after:bottom-0 after:left-3 after:right-3 after:h-[1.5px] after:rounded-full after:bg-[var(--text-primary)]"
                : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
            }`}
          >
            Dataset
          </button>
          <button
            type="button"
            onClick={() => dispatch({ type: "SET_BROWSER_TAB", payload: "staging" })}
            className={`relative px-3 py-3 text-[11px] font-medium tracking-[0.01em] transition-colors duration-150 ${
              browserTab === "staging"
                ? "text-[var(--success)] after:absolute after:bottom-0 after:left-3 after:right-3 after:h-[1.5px] after:rounded-full after:bg-[var(--success)]"
                : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
            }`}
          >
            Staging
          </button>
        </div>
      </div>

      {error ? (
        <div className="border-b border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-4 py-2">
          <p className="text-[11px] text-[var(--destructive)]">{error}</p>
        </div>
      ) : null}

      {browserTab === "staging" ? (
        <>
          <div className="space-y-2 border-b border-[var(--border-default)] px-3 py-3">
            <RecordSearch placeholder="Search staging objects…" />
            <p className="text-[10px] text-[var(--text-tertiary)]">
              {browserCounts.visible} / {browserCounts.total}
            </p>
          </div>
          <StagingList onCountsChange={handleCountsChange} />
        </>
      ) : (
        <>
          <div className="space-y-2 px-3 py-3">
            <RecordSearch placeholder="Search records…" />
            <ExplorerFilters />
            <p className="text-[10px] text-[var(--text-tertiary)]">
              {browserCounts.visible} / {browserCounts.total}
            </p>
          </div>

          {loading ? (
            <div className="flex flex-1 items-center justify-center">
              <p className="text-[11px] text-[var(--text-quaternary)]">Loading…</p>
            </div>
          ) : (
            <>
              <RecordList onVisibleIdsChange={handleVisibleIdsChange} onCountsChange={handleCountsChange} />
              {multiSelection.size > 0 ? (
                <BulkActionBar visibleRecordIds={visibleRecordIds} />
              ) : null}
            </>
          )}
        </>
      )}
    </div>
  );
}
