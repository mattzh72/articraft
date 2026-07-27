import { type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { NativeSelect } from "@/components/ui/native-select";

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
    <NativeSelect
      aria-label="Run filter"
      selectSize="sm"
      className="h-7 w-full font-mono text-[11px]"
      value={selectedRunId ?? "all"}
      onChange={(event) => {
        const value = event.currentTarget.value;
        dispatch({
          type: "SET_RUN_FILTER",
          payload: value === "all" ? null : value,
        });
      }}
    >
      <option value="all">All runs</option>
      {runs.map((run) => (
        <option key={run.run_id} value={run.run_id}>
          {run.run_id}
        </option>
      ))}
    </NativeSelect>
  );
}
