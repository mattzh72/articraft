import { useEffect, useId, useState, type JSX } from "react";
import { Copy, FolderOpen, Trash2 } from "lucide-react";

import { Button } from "@/components/ui/button";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { deleteRecord, fetchBootstrap, openRecordFolder } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import { cn } from "@/lib/utils";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

const MAX_OPEN_FOLDERS = 10;

interface BulkActionBarProps {
  visibleRecordIds: string[];
}

export function BulkActionBar({ visibleRecordIds }: BulkActionBarProps): JSX.Element {
  const { bootstrap, multiSelection, recordCache } = useViewer();
  const dispatch = useViewerDispatch();
  const titleId = useId();
  const descriptionId = useId();
  const selectedCount = multiSelection.size;

  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [deletePending, setDeletePending] = useState(false);
  const [deleteProgress, setDeleteProgress] = useState(0);
  const [deleteError, setDeleteError] = useState<string | null>(null);

  useEffect(() => {
    if (copyState === "idle") return;
    const timeoutId = window.setTimeout(() => setCopyState("idle"), 1800);
    return () => window.clearTimeout(timeoutId);
  }, [copyState]);

  useEffect(() => {
    if (openState === "idle") return;
    const timeoutId = window.setTimeout(() => setOpenState("idle"), 1800);
    return () => window.clearTimeout(timeoutId);
  }, [openState]);

  useEffect(() => {
    if (!confirmOpen) return;
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape" && !deletePending) {
        setConfirmOpen(false);
        setDeleteError(null);
      }
    };
    document.addEventListener("keydown", handleKeyDown);
    return () => document.removeEventListener("keydown", handleKeyDown);
  }, [confirmOpen, deletePending]);

  const selectedIds = [...multiSelection];

  const handleSelectAll = () => {
    dispatch({ type: "SET_MULTI_SELECT_ALL", payload: visibleRecordIds });
  };

  const handleClear = () => {
    dispatch({ type: "CLEAR_MULTI_SELECT" });
  };

  const handleCopyPaths = async () => {
    if (!bootstrap) {
      setCopyState("error");
      return;
    }
    try {
      const paths = selectedIds.map((id) => buildRecordPath(bootstrap.repo_root, id));
      await copyTextToClipboard(paths.join("\n"));
      setCopyState("copied");
    } catch {
      setCopyState("error");
    }
  };

  const handleOpenFolders = async () => {
    const idsToOpen = selectedIds.slice(0, MAX_OPEN_FOLDERS);
    try {
      for (const id of idsToOpen) {
        await openRecordFolder(id);
      }
      setOpenState("opened");
    } catch {
      setOpenState("error");
    }
  };

  const handleDeleteIntent = () => {
    setDeleteError(null);
    setDeleteProgress(0);
    setConfirmOpen(true);
  };

  const handleDeleteCancel = () => {
    if (deletePending) return;
    setConfirmOpen(false);
    setDeleteError(null);
  };

  const handleConfirmDelete = async () => {
    if (deletePending) return;
    setDeletePending(true);
    setDeleteError(null);
    setDeleteProgress(0);

    try {
      for (let i = 0; i < selectedIds.length; i++) {
        setDeleteProgress(i + 1);
        await deleteRecord(selectedIds[i]);
        dispatch({ type: "DELETE_RECORD_LOCAL", payload: selectedIds[i] });
      }
      setConfirmOpen(false);
      dispatch({ type: "CLEAR_MULTI_SELECT" });

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
      setDeleteError(error instanceof Error ? error.message : "Failed to delete record.");
    } finally {
      setDeletePending(false);
    }
  };

  // Build preview titles for confirmation dialog
  const selectedRecords = selectedIds
    .map((recordId) => recordCache[recordId] ?? null)
    .filter((record): record is NonNullable<typeof record> => record !== null);
  const previewTitles = selectedRecords.slice(0, 3).map((record) => record.title);

  return (
    <>
      <div className="flex items-center justify-between border-t border-[var(--border-default)] px-3 py-1.5">
        <div className="flex items-center gap-1 text-[10px]">
          <span className="font-medium text-[var(--text-secondary)]">{selectedCount}</span>
          <button
            type="button"
            onClick={handleSelectAll}
            className="text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-primary)]"
          >
            All
          </button>
          <span className="text-[var(--border-strong)]">·</span>
          <button
            type="button"
            onClick={handleClear}
            className="text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-primary)]"
          >
            Clear
          </button>
        </div>

        <div className="flex items-center gap-0.5">
          <Tooltip>
            <TooltipTrigger asChild>
              <button
                type="button"
                onClick={handleCopyPaths}
                className={cn(
                  "flex size-6 items-center justify-center rounded-md transition-colors",
                  copyState === "copied"
                    ? "text-[var(--success)]"
                    : copyState === "error"
                      ? "text-[var(--destructive)]"
                      : "text-[var(--text-tertiary)] hover:bg-[var(--surface-1)] hover:text-[var(--text-primary)]",
                )}
              >
                <Copy className="size-3" />
              </button>
            </TooltipTrigger>
            <TooltipContent side="top">
              {copyState === "copied" ? "Copied!" : copyState === "error" ? "Failed" : "Copy paths"}
            </TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <button
                type="button"
                onClick={handleOpenFolders}
                className={cn(
                  "flex size-6 items-center justify-center rounded-md transition-colors",
                  openState === "opened"
                    ? "text-[var(--success)]"
                    : openState === "error"
                      ? "text-[var(--destructive)]"
                      : "text-[var(--text-tertiary)] hover:bg-[var(--surface-1)] hover:text-[var(--text-primary)]",
                )}
              >
                <FolderOpen className="size-3" />
              </button>
            </TooltipTrigger>
            <TooltipContent side="top">
              {openState === "opened"
                ? "Opened!"
                : openState === "error"
                  ? "Failed"
                  : selectedCount > MAX_OPEN_FOLDERS
                    ? `Open first ${MAX_OPEN_FOLDERS}`
                    : "Open folders"}
            </TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <button
                type="button"
                onClick={handleDeleteIntent}
                className="flex size-6 items-center justify-center rounded-md text-[var(--text-tertiary)] transition-colors hover:bg-[rgba(209,52,21,0.06)] hover:text-[var(--destructive)]"
              >
                <Trash2 className="size-3" />
              </button>
            </TooltipTrigger>
            <TooltipContent side="top">Delete</TooltipContent>
          </Tooltip>
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
            className="w-full max-w-[340px] rounded-xl border border-[var(--border-default)] bg-[var(--surface-0)] p-4 shadow-[0_16px_48px_rgba(0,0,0,0.12)]"
            onMouseDown={(event) => event.stopPropagation()}
          >
            <div className="space-y-1">
              <h2 id={titleId} className="text-[13px] font-semibold text-[var(--text-primary)]">
                Delete {selectedCount} record{selectedCount === 1 ? "" : "s"}?
              </h2>
              <p id={descriptionId} className="text-[11px] leading-[1.5] text-[var(--text-tertiary)]">
                This will permanently remove {selectedCount === 1 ? "this record" : "these records"} and stored files.
              </p>
              {previewTitles.length > 0 ? (
                <div className="mt-1.5 space-y-px text-[10px] text-[var(--text-quaternary)]">
                  {previewTitles.map((title) => (
                    <p key={title} className="truncate">
                      {title}
                    </p>
                  ))}
                  {selectedRecords.length > 3 ? (
                    <p>+{selectedRecords.length - 3} more</p>
                  ) : null}
                </div>
              ) : null}
            </div>

            {deleteError ? (
              <div className="mt-2.5 rounded-md border border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-2.5 py-1.5 text-[10px] text-[var(--destructive)]">
                {deleteError}
              </div>
            ) : null}

            <div className="mt-4 flex items-center justify-end gap-1.5">
              <Button type="button" variant="outline" size="sm" onClick={handleDeleteCancel} disabled={deletePending}>
                Cancel
              </Button>
              <Button
                type="button"
                size="sm"
                onClick={handleConfirmDelete}
                disabled={deletePending}
                className="bg-[var(--destructive)] text-white hover:bg-[#b82d12]"
              >
                {deletePending ? `${deleteProgress}/${selectedCount}…` : "Delete"}
              </Button>
            </div>
          </div>
        </div>
      ) : null}
    </>
  );
}
