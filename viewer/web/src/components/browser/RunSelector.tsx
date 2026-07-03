import { type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

const RUN_SELECT_CLASS =
  "h-7 w-full rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] px-2.5 text-[11px] font-mono text-[var(--text-primary)] outline-none transition-colors focus-visible:border-[var(--accent)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]";

export function RunSelector(): JSX.Element {
  const { bootstrap, selectedRunId } = useViewer();
  const dispatch = useViewerDispatch();

  const runs = bootstrap?.runs ?? [];

  if (runs.length === 0) {
    return (
      <p className="text-[11px] text-[#bbb]">No runs available</p>
    );
  }

  return (
    <select
      value={selectedRunId ?? "all"}
      className={RUN_SELECT_CLASS}
      aria-label="Run filter"
      onChange={(event) =>
        dispatch({
          type: "SET_RUN_FILTER",
          payload: event.target.value === "all" ? null : event.target.value,
        })
      }
    >
      <option value="all">All runs</option>
      {runs.map((run) => (
        <option key={run.run_id} value={run.run_id}>
          {run.run_id}
        </option>
      ))}
    </select>
  );
}
