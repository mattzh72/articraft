import { type JSX } from "react";
import { CheckSquare, Square, X } from "lucide-react";

import { Button } from "@/components/ui/button";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

export function BulkActionBar({ visibleRecordIds }: { visibleRecordIds: string[] }): JSX.Element {
  const { multiSelection } = useViewer();
  const dispatch = useViewerDispatch();
  const allVisibleSelected =
    visibleRecordIds.length > 0 && visibleRecordIds.every((id) => multiSelection.has(id));

  return (
    <div className="border-t border-[var(--border-default)] bg-[var(--surface-0)] px-3 py-2">
      <div className="flex items-center justify-between gap-2">
        <p className="text-[10px] text-[var(--text-tertiary)]">{multiSelection.size} selected</p>
        <div className="flex items-center gap-1">
          <Button
            type="button"
            variant="ghost"
            size="sm"
            className="h-7 w-7 p-0"
            aria-label={allVisibleSelected ? "Clear visible selection" : "Select visible records"}
            onClick={() => {
              if (allVisibleSelected) {
                dispatch({ type: "CLEAR_MULTI_SELECT" });
                return;
              }
              dispatch({ type: "SET_MULTI_SELECT_ALL", payload: visibleRecordIds });
            }}
          >
            {allVisibleSelected ? <Square className="size-3.5" /> : <CheckSquare className="size-3.5" />}
          </Button>
          <Button
            type="button"
            variant="ghost"
            size="sm"
            className="h-7 w-7 p-0"
            aria-label="Clear selection"
            onClick={() => dispatch({ type: "CLEAR_MULTI_SELECT" })}
          >
            <X className="size-3.5" />
          </Button>
        </div>
      </div>
    </div>
  );
}
