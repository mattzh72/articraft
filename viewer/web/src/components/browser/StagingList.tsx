import { useEffect, useMemo, useState, type JSX } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { FolderOpen, MoreVertical, Trash2 } from "lucide-react";

import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import type { StagingEntry, ViewerSelection } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { cn } from "@/lib/utils";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Button } from "@/components/ui/button";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { deleteStagingEntry, openStagingFolder } from "@/lib/api";
import { viewerQueryKeys } from "@/lib/viewer-queries";
import {
  getPreviewStagingEntries,
  isPreviewStagingEntry,
} from "@/components/browser/staging-preview";

function truncateWithEllipsis(value: string, maxLength = 88): string {
  const normalized = value.replace(/\s+/g, " ").trim();
  if (!normalized) return "";
  if (normalized.length <= maxLength) {
    return normalized;
  }
  return `${normalized.slice(0, maxLength).trimEnd()}...`;
}

function formatTimeAgo(dateString: string | null): string | null {
  if (!dateString) return null;
  const date = new Date(dateString);
  if (Number.isNaN(date.getTime())) return null;

  const seconds = Math.floor((Date.now() - date.getTime()) / 1000);
  if (seconds < 60) return "just now";
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ago`;
  const hours = Math.floor(minutes / 60);
  if (hours < 24) return `${hours}h ago`;
  const days = Math.floor(hours / 24);
  return `${days}d ago`;
}

function statusDotClass(status: string | null): string {
  switch ((status ?? "").toLowerCase()) {
    case "failed":
      return "bg-[var(--destructive)]";
    case "success":
      return "bg-[var(--success)]";
    default:
      return "bg-[var(--success)] staging-pulse";
  }
}

function isVisibleStagingEntry(entry: StagingEntry): boolean {
  const status = (entry.status ?? "").toLowerCase();
  return status !== "success";
}

function isFailedEntry(entry: StagingEntry): boolean {
  return (entry.status ?? "").toLowerCase() === "failed";
}

function isStagingSelected(selection: ViewerSelection | null, entry: StagingEntry): boolean {
  return (
    selection?.kind === "staging" &&
    selection.runId === entry.run_id &&
    selection.recordId === entry.record_id
  );
}

function matchesSearch(entry: StagingEntry, query: string): boolean {
  if (!query) {
    return true;
  }

  const haystack = [
    entry.title,
    entry.prompt_preview,
    entry.record_id,
    entry.run_id,
    entry.model_id,
    entry.provider,
    entry.status,
  ]
    .filter((value): value is string => Boolean(value))
    .join(" ")
    .toLowerCase();

  return haystack.includes(query.toLowerCase());
}

function StagingListItem({ entry }: { entry: StagingEntry }): JSX.Element {
  const { bootstrap, selection } = useViewer();
  const dispatch = useViewerDispatch();
  const queryClient = useQueryClient();
  const isPreview = isPreviewStagingEntry(entry);
  const isSelected = isStagingSelected(selection, entry);
  const isFailed = isFailedEntry(entry);
  const [menuOpen, setMenuOpen] = useState(false);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [deletePending, setDeletePending] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const summaryText = truncateWithEllipsis(entry.prompt_preview || entry.title || "Untitled");
  const metadata = [
    !isFailed ? entry.status : null,
    entry.model_id,
    entry.turn_count !== null ? `${entry.turn_count} turns` : null,
    formatTimeAgo(entry.updated_at),
  ].filter((value): value is string => Boolean(value));

  useEffect(() => {
    if (openState === "idle") {
      return;
    }

    const timeoutId = window.setTimeout(() => {
      setOpenState("idle");
    }, 1800);

    return () => {
      window.clearTimeout(timeoutId);
    };
  }, [openState]);

  const handleOpenStagingFolder = async () => {
    try {
      await openStagingFolder(entry.run_id, entry.record_id);
      setOpenState("opened");
    } catch {
      setOpenState("error");
    }
  };

  const handleDeleteIntent = () => {
    setMenuOpen(false);
    setDeleteError(null);
    setConfirmOpen(true);
  };

  const handleDeleteCancel = () => {
    if (deletePending) {
      return;
    }
    setConfirmOpen(false);
    setDeleteError(null);
  };

  const handleConfirmDelete = async () => {
    if (deletePending) {
      return;
    }

    setDeletePending(true);
    setDeleteError(null);

    try {
      await deleteStagingEntry(entry.run_id, entry.record_id);
      if (bootstrap) {
        dispatch({
          type: "UPDATE_STAGING",
          payload: bootstrap.staging_entries.filter(
            (candidate) => candidate.run_id !== entry.run_id || candidate.record_id !== entry.record_id,
          ),
        });
      }
      setConfirmOpen(false);
      await queryClient.invalidateQueries({ queryKey: viewerQueryKeys.root() });
    } catch (error) {
      setDeleteError(error instanceof Error ? error.message : "Failed to delete staging entry.");
    } finally {
      setDeletePending(false);
    }
  };

  return (
    <>
      <div className="px-1.5">
        <div
          className={cn(
            "group flex items-start gap-0.5 rounded-lg px-2.5 py-2 transition-colors duration-100",
            isSelected ? "bg-[rgba(26,138,74,0.08)]" : "hover:bg-[var(--surface-1)]",
            isPreview && "border border-dashed border-[rgba(26,138,74,0.22)] bg-[rgba(26,138,74,0.04)]",
          )}
        >
          <button
            type="button"
            disabled={isPreview}
            onClick={() =>
              dispatch({
                type: "SELECT_ITEM",
                payload: { kind: "staging", runId: entry.run_id, recordId: entry.record_id },
              })
            }
            className="min-w-0 flex-1 rounded-sm py-0 text-left focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[rgba(26,138,74,0.18)] disabled:cursor-default"
            title={summaryText}
          >
            <div className="flex items-start gap-2">
              <span className={cn("mt-[5px] size-[6px] shrink-0 rounded-full", statusDotClass(entry.status))} />

              <div className="min-w-0 flex-1">
                <div className="flex items-center gap-1.5">
                  <p
                    className={cn(
                      "break-words text-[11px] leading-[1.45]",
                      isSelected ? "font-medium text-[var(--text-primary)]" : "text-[var(--text-secondary)]",
                      isPreview && "text-[var(--text-primary)]",
                    )}
                  >
                    {summaryText}
                  </p>
                  {isPreview ? (
                    <span className="shrink-0 rounded-full border border-[rgba(26,138,74,0.22)] bg-[rgba(26,138,74,0.07)] px-1.5 py-0.5 text-[8.5px] font-medium uppercase tracking-[0.06em] text-[var(--success)]">
                      Preview
                    </span>
                  ) : null}
                </div>

                {isFailed ? (
                  <p className="mt-1 text-[8.5px] font-semibold tracking-[0.08em] text-[var(--destructive)]">
                    FAILED
                  </p>
                ) : null}

                {metadata.length > 0 ? (
                  <div className="mt-1 flex flex-wrap items-center gap-x-1 gap-y-0.5 text-[9.5px] text-[var(--text-tertiary)]">
                    {metadata.map((item, index) => (
                      <span key={`${entry.run_id}-${entry.record_id}-${item}`} className="flex items-center gap-x-1">
                        {index > 0 ? <span className="text-[var(--border-strong)]">·</span> : null}
                        <span>{item}</span>
                      </span>
                    ))}
                  </div>
                ) : null}

                <p className="mt-1 text-[9.5px] text-[var(--text-quaternary)]">
                  {isPreview ? "Temporary UI-only preview row" : entry.run_id}
                </p>
              </div>
            </div>
          </button>

          {!isPreview ? (
            <DropdownMenu open={menuOpen} onOpenChange={setMenuOpen}>
              <div className="relative shrink-0 pt-px">
                <Tooltip>
                  <TooltipTrigger asChild>
                    <DropdownMenuTrigger asChild>
                      <button
                        type="button"
                        aria-label={`Open actions for ${entry.title || entry.record_id}`}
                        className={cn(
                          "flex size-6 items-center justify-center rounded-md text-[var(--text-tertiary)] opacity-0 transition-all duration-100 group-hover:opacity-100 focus-visible:opacity-100 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[rgba(26,138,74,0.18)]",
                          menuOpen
                            ? "bg-[var(--surface-0)] text-[var(--text-primary)] opacity-100 shadow-[0_1px_3px_rgba(0,0,0,0.06)]"
                            : "hover:bg-[var(--surface-0)] hover:text-[var(--text-primary)]",
                        )}
                        onClick={(event) => event.stopPropagation()}
                      >
                        <MoreVertical className="size-3" />
                      </button>
                    </DropdownMenuTrigger>
                  </TooltipTrigger>
                  {!menuOpen ? <TooltipContent side="right">Actions</TooltipContent> : null}
                </Tooltip>

                <DropdownMenuContent onClick={(event) => event.stopPropagation()}>
                  <DropdownMenuItem onSelect={() => void handleOpenStagingFolder()}>
                    <FolderOpen className="size-3.5" />
                    <span>
                      {openState === "opened"
                        ? "Opened staging folder"
                        : openState === "error"
                          ? "Open failed"
                          : "Open staging folder"}
                    </span>
                  </DropdownMenuItem>
                  <DropdownMenuItem
                    onSelect={handleDeleteIntent}
                    className="text-[var(--destructive)] focus:bg-[rgba(209,52,21,0.06)] focus:text-[var(--destructive)]"
                  >
                    <Trash2 className="size-3.5" />
                    <span>Delete</span>
                  </DropdownMenuItem>
                </DropdownMenuContent>
              </div>
            </DropdownMenu>
          ) : null}
        </div>
      </div>

      <AlertDialog open={confirmOpen} onOpenChange={setConfirmOpen}>
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Delete staging entry?</AlertDialogTitle>
            <AlertDialogDescription>
              This will permanently remove this staging entry and its staged files from the viewer
              cache.
            </AlertDialogDescription>
          </AlertDialogHeader>

          {deleteError ? (
            <div className="mt-3 rounded-lg border border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-3 py-2 text-[11px] text-[var(--destructive)]">
              {deleteError}
            </div>
          ) : null}

          <AlertDialogFooter>
            <AlertDialogCancel asChild>
              <Button type="button" variant="outline" onClick={handleDeleteCancel} disabled={deletePending}>
                Cancel
              </Button>
            </AlertDialogCancel>
            <AlertDialogAction asChild>
              <Button
                type="button"
                onClick={() => void handleConfirmDelete()}
                disabled={deletePending}
                className="bg-[var(--destructive)] text-white hover:bg-[#b82d12]"
              >
                {deletePending ? "Deleting..." : "Delete"}
              </Button>
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
}

export function StagingList({
  onCountsChange,
}: {
  onCountsChange?: (counts: { visible: number; total: number }) => void;
}): JSX.Element {
  const { bootstrap, loading, searchQuery } = useViewer();

  const entries = useMemo(
    () => getPreviewStagingEntries((bootstrap?.staging_entries ?? []).filter(isVisibleStagingEntry)),
    [bootstrap],
  );
  const visibleEntries = useMemo(
    () => entries.filter((entry) => matchesSearch(entry, searchQuery.trim())),
    [entries, searchQuery],
  );

  useEffect(() => {
    onCountsChange?.({
      visible: visibleEntries.length,
      total: entries.length,
    });
  }, [entries.length, onCountsChange, visibleEntries.length]);

  if (!bootstrap && loading) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">Loading…</p>
      </div>
    );
  }

  if (visibleEntries.length === 0) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">
          {searchQuery ? "No matching staging objects" : "No staging objects"}
        </p>
      </div>
    );
  }

  return (
    <ScrollArea className="min-h-0 flex-1">
      <div className="py-0.5">
        {visibleEntries.map((entry) => (
          <StagingListItem key={`${entry.run_id}:${entry.record_id}`} entry={entry} />
        ))}
      </div>
    </ScrollArea>
  );
}
