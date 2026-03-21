import { useEffect, useId, useMemo, useRef, useState, type JSX, type MouseEvent as ReactMouseEvent } from "react";
import { FolderOpen, MoreVertical, Trash2 } from "lucide-react";

import type { StagingEntry, ViewerSelection } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { cn } from "@/lib/utils";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Button } from "@/components/ui/button";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { deleteStagingEntry, fetchBootstrap, openStagingFolder } from "@/lib/api";
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
  const isPreview = isPreviewStagingEntry(entry);
  const isSelected = isStagingSelected(selection, entry);
  const isFailed = isFailedEntry(entry);
  const [menuOpen, setMenuOpen] = useState(false);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [deletePending, setDeletePending] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const menuRef = useRef<HTMLDivElement | null>(null);
  const titleId = useId();
  const descriptionId = useId();
  const summaryText = truncateWithEllipsis(entry.prompt_preview || entry.title || "Untitled");
  const metadata = [
    !isFailed ? entry.status : null,
    entry.model_id,
    entry.turn_count !== null ? `${entry.turn_count} turns` : null,
    formatTimeAgo(entry.updated_at),
  ].filter((value): value is string => Boolean(value));

  useEffect(() => {
    if (!menuOpen) {
      return;
    }

    const handlePointerDown = (event: MouseEvent) => {
      if (!menuRef.current) {
        return;
      }
      if (event.target instanceof Node && !menuRef.current.contains(event.target)) {
        setMenuOpen(false);
      }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setMenuOpen(false);
      }
    };

    document.addEventListener("mousedown", handlePointerDown);
    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("mousedown", handlePointerDown);
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [menuOpen]);

  useEffect(() => {
    if (!confirmOpen) {
      return;
    }

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape" && !deletePending) {
        setConfirmOpen(false);
        setDeleteError(null);
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [confirmOpen, deletePending]);

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

  const handleMenuToggle = (event: ReactMouseEvent<HTMLButtonElement>) => {
    event.stopPropagation();
    setMenuOpen((current) => !current);
  };

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

      try {
        const refreshedBootstrap = await fetchBootstrap();
        dispatch({ type: "SET_BOOTSTRAP", payload: refreshedBootstrap });
      } catch (error) {
        dispatch({
          type: "SET_ERROR",
          payload: error instanceof Error ? error.message : "Failed to refresh viewer data.",
        });
      }
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
                  {isPreview ? "Temporary UI-only preview row" : `run ${entry.run_id}`}
                </p>
              </div>
            </div>
          </button>

          {!isPreview ? (
            <div ref={menuRef} className="relative shrink-0 pt-px">
              <Tooltip>
                <TooltipTrigger asChild>
                  <button
                    type="button"
                    aria-label={`Open actions for ${entry.title || entry.record_id}`}
                    aria-haspopup="menu"
                    aria-expanded={menuOpen}
                    onClick={handleMenuToggle}
                    className={cn(
                      "flex size-6 items-center justify-center rounded-md text-[var(--text-tertiary)] opacity-0 transition-all duration-100 group-hover:opacity-100 focus-visible:opacity-100 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[rgba(26,138,74,0.18)]",
                      menuOpen
                        ? "bg-[var(--surface-0)] text-[var(--text-primary)] opacity-100 shadow-[0_1px_3px_rgba(0,0,0,0.06)]"
                        : "hover:bg-[var(--surface-0)] hover:text-[var(--text-primary)]",
                    )}
                  >
                    <MoreVertical className="size-3" />
                  </button>
                </TooltipTrigger>
                {!menuOpen ? <TooltipContent side="right">Actions</TooltipContent> : null}
              </Tooltip>

              {menuOpen ? (
                <div
                  role="menu"
                  className="absolute right-0 top-full z-20 mt-1 min-w-[11rem] overflow-hidden rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 text-[var(--text-primary)] shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
                >
                  <button
                    type="button"
                    role="menuitem"
                    onClick={handleOpenStagingFolder}
                    className="flex w-full items-center gap-2 whitespace-nowrap rounded-md px-2.5 py-1.5 text-left font-sans text-[12px] font-medium text-[var(--text-secondary)] transition-colors hover:bg-[var(--surface-1)] hover:text-[var(--text-primary)]"
                  >
                    <FolderOpen className="size-3.5" />
                    <span>
                      {openState === "opened"
                        ? "Opened staging folder"
                        : openState === "error"
                          ? "Open failed"
                          : "Open staging folder"}
                    </span>
                  </button>
                  <button
                    type="button"
                    role="menuitem"
                    onClick={handleDeleteIntent}
                    className="flex w-full items-center gap-2 whitespace-nowrap rounded-md px-2.5 py-1.5 text-left font-sans text-[12px] font-medium text-[var(--destructive)] transition-colors hover:bg-[rgba(209,52,21,0.06)]"
                  >
                    <Trash2 className="size-3.5" />
                    <span>Delete</span>
                  </button>
                </div>
              ) : null}
            </div>
          ) : null}
        </div>
      </div>

      {confirmOpen ? (
        <div
          className="fixed inset-0 z-50 flex items-center justify-center bg-[rgba(0,0,0,0.15)] px-4 backdrop-blur-[2px]"
          onMouseDown={(event) => {
            if (event.target === event.currentTarget) {
              handleDeleteCancel();
            }
          }}
        >
          <div
            role="dialog"
            aria-modal="true"
            aria-labelledby={titleId}
            aria-describedby={descriptionId}
            className="w-full max-w-[380px] rounded-xl border border-[var(--border-default)] bg-[var(--surface-0)] p-5 shadow-[0_16px_48px_rgba(0,0,0,0.12)]"
            onMouseDown={(event) => event.stopPropagation()}
          >
            <div className="space-y-1.5">
              <h2 id={titleId} className="text-[14px] font-semibold text-[var(--text-primary)]">
                Delete staging entry?
              </h2>
              <p id={descriptionId} className="text-[12px] leading-5 text-[var(--text-secondary)]">
                This will permanently remove this staging entry and its staged files from the viewer cache.
              </p>
            </div>

            {deleteError ? (
              <div className="mt-3 rounded-lg border border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-3 py-2 text-[11px] text-[var(--destructive)]">
                {deleteError}
              </div>
            ) : null}

            <div className="mt-5 flex items-center justify-end gap-2">
              <Button type="button" variant="outline" onClick={handleDeleteCancel} disabled={deletePending}>
                Cancel
              </Button>
              <Button
                type="button"
                onClick={handleConfirmDelete}
                disabled={deletePending}
                className="bg-[var(--destructive)] text-white hover:bg-[#b82d12]"
              >
                {deletePending ? "Deleting..." : "Delete"}
              </Button>
            </div>
          </div>
        </div>
      ) : null}
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
      total: bootstrap?.staging_entries.length ?? 0,
    });
  }, [bootstrap, onCountsChange, visibleEntries.length]);

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
