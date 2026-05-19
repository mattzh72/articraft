import { useEffect, useState, type JSX } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { Copy, FolderOpen, Trash2 } from "lucide-react";

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
import { Button } from "@/components/ui/button";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { deleteRecord, openRecordFolder } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import { cn } from "@/lib/utils";
import { browseRecordIdsQueryOptions, viewerQueryKeys } from "@/lib/viewer-queries";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

const MAX_OPEN_FOLDERS = 10;

interface BulkActionBarProps {
  visibleRecordIds: string[];
}

export function BulkActionBar({ visibleRecordIds }: BulkActionBarProps): JSX.Element {
  const {
    authorFilters,
    agentHarnessFilters,
    bootstrap,
    categoryFilters,
    costFilter,
    modelFilter,
    multiSelection,
    ratingFilter,
    recordCache,
    sdkFilter,
    searchQuery,
    secondaryRatingFilter,
    selectedRunId,
    sourceFilter,
    timeFilter,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const queryClient = useQueryClient();
  const selectedCount = multiSelection.size;

  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [selectAllPending, setSelectAllPending] = useState(false);
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

  const selectedIds = [...multiSelection];

  const handleSelectAll = async () => {
    if (sourceFilter !== "dataset") {
      dispatch({ type: "SET_MULTI_SELECT_ALL", payload: visibleRecordIds });
      return;
    }

    setSelectAllPending(true);
    try {
      const response = await queryClient.fetchQuery(
        browseRecordIdsQueryOptions({
          source: "dataset",
          query: searchQuery.trim(),
          runId: selectedRunId,
          timeFilter,
          modelFilter,
          sdkFilter,
          agentHarnessFilters,
          authorFilters,
          categoryFilters,
          costFilter,
          ratingFilter,
          secondaryRatingFilter,
        }),
      );
      dispatch({ type: "SET_MULTI_SELECT_ALL", payload: response.record_ids });
    } finally {
      setSelectAllPending(false);
    }
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
      await queryClient.invalidateQueries({ queryKey: viewerQueryKeys.root() });
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
            onClick={() => void handleSelectAll()}
            disabled={selectAllPending}
            className="text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-primary)]"
          >
            {selectAllPending ? "Loading" : "All"}
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

      <AlertDialog open={confirmOpen} onOpenChange={setConfirmOpen}>
        <AlertDialogContent className="max-w-[340px] p-4">
          <AlertDialogHeader className="space-y-1">
            <AlertDialogTitle className="text-[13px]">
              Delete {selectedCount} record{selectedCount === 1 ? "" : "s"}?
            </AlertDialogTitle>
            <AlertDialogDescription className="text-[11px] leading-[1.5] text-[var(--text-tertiary)]">
              This will permanently remove {selectedCount === 1 ? "this record" : "these records"} and stored files.
            </AlertDialogDescription>
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
          </AlertDialogHeader>

          {deleteError ? (
            <div className="mt-2.5 rounded-md border border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-2.5 py-1.5 text-[10px] text-[var(--destructive)]">
              {deleteError}
            </div>
          ) : null}

          <AlertDialogFooter className="mt-4 gap-1.5">
            <AlertDialogCancel asChild>
              <Button type="button" variant="outline" size="sm" onClick={handleDeleteCancel} disabled={deletePending}>
                Cancel
              </Button>
            </AlertDialogCancel>
            <AlertDialogAction asChild>
              <Button
                type="button"
                size="sm"
                onClick={() => void handleConfirmDelete()}
                disabled={deletePending}
                className="bg-[var(--destructive)] text-white hover:bg-[#b82d12]"
              >
                {deletePending ? `${deleteProgress}/${selectedCount}...` : "Delete"}
              </Button>
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
}
