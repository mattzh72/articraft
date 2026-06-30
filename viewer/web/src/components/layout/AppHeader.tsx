import { type JSX } from "react";
import { useIsFetching, useQueryClient } from "@tanstack/react-query";
import { RefreshCw } from "lucide-react";

import { viewerQueryKeys } from "@/lib/viewer-queries";
import { useViewer } from "@/lib/viewer-context";
import { findStagingEntryInBootstrap } from "@/lib/record-summary";
import { Badge } from "@/components/ui/badge";
import { Tooltip, TooltipContent, TooltipTrigger } from "@/components/ui/tooltip";

const TRAILING_PUNCTUATION = /[\s,.;:!?)]*$/;

function truncateWithEllipsis(value: string, maxLength = 88): string {
  const normalized = value.replace(/\s+/g, " ").trim();
  if (!normalized) return "";
  const withoutExistingEllipsis = normalized.replace(/\.\.\.$/, "").trimEnd();
  if (withoutExistingEllipsis.length <= maxLength) {
    return withoutExistingEllipsis;
  }
  return `${withoutExistingEllipsis.slice(0, maxLength).trimEnd().replace(TRAILING_PUNCTUATION, "")}...`;
}

export function AppHeader(): JSX.Element {
  const state = useViewer();
  const queryClient = useQueryClient();
  const activeFetchCount = useIsFetching({ queryKey: viewerQueryKeys.root() });
  const isStagingSelection = state.selection?.kind === "staging";
  const stagingEntry =
    isStagingSelection && state.selection?.kind === "staging"
      ? findStagingEntryInBootstrap(state.bootstrap, state.selection.runId, state.selection.recordId)
      : null;
  const titleSource = isStagingSelection
    ? stagingEntry?.title ?? null
    : state.selectedRecordSummary?.title ?? null;
  const selectedRecordTitle = titleSource ? truncateWithEllipsis(titleSource, 72) : null;

  const handleRefresh = async () => {
    await queryClient.invalidateQueries({ queryKey: viewerQueryKeys.root() });
  };

  return (
    <header className="flex h-11 shrink-0 items-center gap-3 border-b border-[var(--border-default)] bg-[var(--surface-0)] px-4">
      <div className="flex items-center gap-2 text-[12px]">
        <span className="font-semibold text-[var(--text-primary)]">Articraft</span>
        <span className="text-[var(--border-strong)]">/</span>
        <span className="text-[var(--text-tertiary)]">Viewer</span>
      </div>

      <div className="mx-1 flex min-w-0 flex-1 items-center justify-center gap-2">
        {selectedRecordTitle ? (
          <>
            {isStagingSelection ? <Badge variant="success">STAGING</Badge> : null}
            <p className="max-w-full truncate text-[12px] text-[var(--text-secondary)]" title={titleSource ?? undefined}>
              {selectedRecordTitle}
            </p>
          </>
        ) : (
          <p className="text-[12px] text-[var(--text-quaternary)]">No record selected</p>
        )}
      </div>

      <Tooltip>
        <TooltipTrigger
          type="button"
          onClick={handleRefresh}
          disabled={activeFetchCount > 0}
          className="inline-flex h-7 w-7 items-center justify-center rounded-md p-0 text-[var(--text-tertiary)] transition-all duration-150 hover:bg-[var(--surface-2)] hover:text-[var(--text-secondary)] disabled:pointer-events-none disabled:opacity-40 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent)] focus-visible:ring-offset-1"
        >
          <RefreshCw className={`size-3.5 ${activeFetchCount > 0 ? "animate-spin" : ""}`} />
        </TooltipTrigger>
        <TooltipContent side="bottom">Refresh</TooltipContent>
      </Tooltip>
    </header>
  );
}
