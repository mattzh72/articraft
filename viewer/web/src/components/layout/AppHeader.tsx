import { type JSX } from "react";
import { ChevronLeft, ChevronRight, RefreshCw } from "lucide-react";

import { fetchBootstrap } from "@/lib/api";
import { useRoute } from "@/lib/useRoute";
import { navigateTo } from "@/lib/router";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { findStagingEntryInBootstrap } from "@/lib/record-summary";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";

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
  const route = useRoute();

  const isStagingSelection = state.selection?.kind === "staging";
  const stagingEntry = isStagingSelection && state.selection?.kind === "staging"
    ? findStagingEntryInBootstrap(state.bootstrap, state.selection.runId, state.selection.recordId)
    : null;

  const titleSource = isStagingSelection
    ? stagingEntry?.title ?? null
    : state.selectedRecordSummary?.title ?? null;
  const selectedRecordTitleFull = titleSource;
  const selectedRecordTitle = titleSource ? truncateWithEllipsis(titleSource, 72) : null;

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

  const isDashboard = route.page === "dashboard";

  return (
    <header className="flex h-11 shrink-0 items-center gap-3 border-b border-[var(--border-default)] bg-[var(--surface-0)] px-4">
      <div className="flex items-center gap-2 text-[12px]">
        <span className="font-semibold tracking-[-0.02em] text-[var(--text-primary)]">Articraft</span>
        <span className="text-[var(--border-strong)]">/</span>
        <span className="text-[var(--text-tertiary)]">{isDashboard ? "Dashboard" : "Viewer"}</span>
      </div>

      <div className="mx-1 flex min-w-0 flex-1 items-center justify-center gap-2">
        {!isDashboard && selectedRecordTitle ? (
          <>
            {isStagingSelection ? <Badge variant="success">STAGING</Badge> : null}
            <p
              className="max-w-full truncate text-[12px] text-[var(--text-secondary)]"
              title={selectedRecordTitleFull ?? undefined}
            >
              {selectedRecordTitle}
            </p>
          </>
        ) : !isDashboard ? (
          <p className="text-[12px] text-[var(--text-quaternary)]">No record selected</p>
        ) : null}
      </div>

      <div className="flex items-center gap-1">
        {isDashboard ? (
          <Button
            variant="ghost"
            size="sm"
            onClick={() => navigateTo({ page: "viewer" })}
            className="h-7 gap-1 rounded-md px-2 text-[11px] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
          >
            Viewer
            <ChevronRight className="size-3" />
          </Button>
        ) : (
          <Button
            variant="ghost"
            size="sm"
            onClick={() => navigateTo({ page: "dashboard" })}
            className="h-7 gap-1 rounded-md px-2 text-[11px] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
          >
            <ChevronLeft className="size-3" />
            Dashboard
          </Button>
        )}
        <Tooltip>
          <TooltipTrigger asChild>
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
          </TooltipTrigger>
          <TooltipContent side="bottom">Refresh</TooltipContent>
        </Tooltip>
      </div>
    </header>
  );
}
