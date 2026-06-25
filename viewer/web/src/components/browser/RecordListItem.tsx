import { memo, useEffect, useState, type JSX, type MouseEvent as ReactMouseEvent } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { Check, Copy, FolderOpen, MoreVertical, Star, Trash2 } from "lucide-react";

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
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Tooltip, TooltipContent, TooltipTrigger } from "@/components/ui/tooltip";
import { deleteRecord, openRecordFolder } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import type { RecordSummary } from "@/lib/types";
import { cn } from "@/lib/utils";
import { useViewerDispatch } from "@/lib/viewer-context";
import { viewerQueryKeys } from "@/lib/viewer-queries";

interface RecordListItemProps {
  recordId: string;
  record: RecordSummary | null;
  dataRoot: string | null;
  isSelected: boolean;
  multiSelectActive: boolean;
  isMultiSelected: boolean;
  onSelect: (recordId: string) => void;
  onMultiSelectToggle: (recordId: string, shiftKey: boolean) => void;
}

const TRAILING_PUNCTUATION = /[\s,.;:!?)]*$/;
const COST_FORMATTER = new Intl.NumberFormat(undefined, {
  style: "currency",
  currency: "USD",
  minimumFractionDigits: 2,
  maximumFractionDigits: 2,
});
const DATE_FORMATTER_CURRENT_YEAR = new Intl.DateTimeFormat(undefined, {
  month: "short",
  day: "numeric",
});

function truncateWithEllipsis(value: string, maxLength = 88): string {
  const normalized = value.replace(/\s+/g, " ").trim();
  if (!normalized) return "";
  const withoutExistingEllipsis = normalized.replace(/\.\.\.$/, "").trimEnd();
  if (withoutExistingEllipsis.length <= maxLength) {
    return withoutExistingEllipsis;
  }
  return `${withoutExistingEllipsis.slice(0, maxLength).trimEnd().replace(TRAILING_PUNCTUATION, "")}...`;
}

function formatCost(value: number | null): string | null {
  if (value === null || Number.isNaN(value)) return null;
  if (value < 0.1) return `$${value.toFixed(3)}`;
  return COST_FORMATTER.format(value);
}

function formatDate(value: string | null): string | null {
  if (!value) return null;
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return null;
  const now = new Date();
  return date.getFullYear() === now.getFullYear()
    ? DATE_FORMATTER_CURRENT_YEAR.format(date)
    : new Intl.DateTimeFormat(undefined, {
        month: "short",
        day: "numeric",
        year: "numeric",
      }).format(date);
}

function externalAgentLabel(agent: string | null | undefined): string {
  if (agent === "codex") return "Codex";
  if (agent === "claude-code") return "Claude Code";
  if (agent === "cursor") return "Cursor";
  return agent ?? "External";
}

function RecordListItemInner({
  recordId,
  record,
  dataRoot,
  isSelected,
  multiSelectActive,
  isMultiSelected,
  onSelect,
  onMultiSelectToggle,
}: RecordListItemProps): JSX.Element {
  const dispatch = useViewerDispatch();
  const queryClient = useQueryClient();
  const [menuOpen, setMenuOpen] = useState(false);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [deletePending, setDeletePending] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const summaryText = truncateWithEllipsis(record?.prompt_preview || record?.title || recordId);
  const metadata = [
    record?.category_slug ?? null,
    record?.creator_mode === "external_agent" ? externalAgentLabel(record.external_agent) : null,
    record?.model_id ?? null,
    record?.turn_count != null
      ? `${record.turn_count} turn${record.turn_count === 1 ? "" : "s"}`
      : null,
    formatCost(record?.total_cost_usd ?? null),
    formatDate(record?.updated_at ?? record?.created_at ?? null),
  ].filter((item): item is string => Boolean(item));
  const effectiveRating = record?.effective_rating ?? null;
  const effectiveRatingLabel =
    effectiveRating == null
      ? null
      : Number.isInteger(effectiveRating)
        ? effectiveRating.toFixed(0)
        : effectiveRating.toFixed(1);

  useEffect(() => {
    if (copyState === "idle") {
      return;
    }
    const timeoutId = window.setTimeout(() => setCopyState("idle"), 1800);
    return () => window.clearTimeout(timeoutId);
  }, [copyState]);

  useEffect(() => {
    if (openState === "idle") {
      return;
    }
    const timeoutId = window.setTimeout(() => setOpenState("idle"), 1800);
    return () => window.clearTimeout(timeoutId);
  }, [openState]);

  const handleClick = (event: ReactMouseEvent<HTMLButtonElement>) => {
    if (event.metaKey || event.ctrlKey || multiSelectActive) {
      event.preventDefault();
      onMultiSelectToggle(recordId, event.shiftKey);
      return;
    }
    onSelect(recordId);
  };

  const handleCopyPath = async () => {
    if (!dataRoot) {
      setCopyState("error");
      return;
    }
    const path = buildRecordPath(dataRoot, recordId);
    try {
      await copyTextToClipboard(path);
      setCopyState("copied");
    } catch {
      setCopyState("error");
    }
  };

  const handleOpenFolder = async () => {
    try {
      await openRecordFolder(recordId);
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
    if (deletePending) return;
    setConfirmOpen(false);
    setDeleteError(null);
  };

  const handleConfirmDelete = async () => {
    if (deletePending) return;
    setDeletePending(true);
    setDeleteError(null);
    try {
      await deleteRecord(recordId);
      dispatch({ type: "DELETE_RECORD_LOCAL", payload: recordId });
      setConfirmOpen(false);
      await queryClient.invalidateQueries({ queryKey: viewerQueryKeys.root() });
    } catch (error) {
      setDeleteError(error instanceof Error ? error.message : "Failed to delete record.");
    } finally {
      setDeletePending(false);
    }
  };

  return (
    <>
      <div className="px-1.5">
        <div
          className={cn(
            "group flex items-start gap-1 rounded-lg px-2.5 py-2 transition-colors duration-100",
            isSelected ? "bg-[rgba(33,96,180,0.08)]" : "hover:bg-[var(--surface-1)]",
            isMultiSelected && "ring-1 ring-[rgba(33,96,180,0.32)]",
          )}
        >
          <button
            type="button"
            onClick={handleClick}
            className="min-w-0 flex-1 rounded-sm text-left focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[rgba(33,96,180,0.18)]"
            title={summaryText}
          >
            <div className="flex items-start gap-2">
              {isMultiSelected ? (
                <span className="mt-0.5 flex size-4 shrink-0 items-center justify-center rounded-sm bg-[var(--accent)] text-white">
                  <Check className="size-3" />
                </span>
              ) : (
                <span className="mt-[5px] size-[6px] shrink-0 rounded-full bg-[var(--accent)]" />
              )}
              <div className="min-w-0 flex-1">
                <p
                  className={cn(
                    "break-words text-[11px] leading-[1.45]",
                    isSelected ? "font-medium text-[var(--text-primary)]" : "text-[var(--text-secondary)]",
                  )}
                >
                  {summaryText}
                </p>
                <div className="mt-1 flex flex-wrap items-center gap-x-1 gap-y-0.5 text-[9.5px] text-[var(--text-tertiary)]">
                  {effectiveRatingLabel ? (
                    <span className="inline-flex items-center gap-0.5 text-[var(--warning)]">
                      <Star className="size-3 fill-current" />
                      {effectiveRatingLabel}
                    </span>
                  ) : null}
                  {metadata.map((item, index) => (
                    <span key={`${recordId}-${item}`} className="flex items-center gap-x-1">
                      {index > 0 || effectiveRatingLabel ? (
                        <span className="text-[var(--border-strong)]">&middot;</span>
                      ) : null}
                      <span>{item}</span>
                    </span>
                  ))}
                </div>
              </div>
            </div>
          </button>

          <DropdownMenu open={menuOpen} onOpenChange={setMenuOpen}>
            <DropdownMenuTrigger asChild>
              <Button
                variant="ghost"
                size="sm"
                className="h-6 w-6 shrink-0 rounded-md p-0 opacity-0 transition-opacity group-hover:opacity-100 data-[state=open]:opacity-100"
                aria-label="Record actions"
              >
                <MoreVertical className="size-3.5" />
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end" className="min-w-40">
              <DropdownMenuItem onSelect={() => void handleCopyPath()}>
                <Copy className="mr-2 size-3.5" />
                {copyState === "copied" ? "Copied" : copyState === "error" ? "Copy failed" : "Copy path"}
              </DropdownMenuItem>
              <DropdownMenuItem onSelect={() => void handleOpenFolder()}>
                <FolderOpen className="mr-2 size-3.5" />
                {openState === "opened" ? "Opened" : openState === "error" ? "Open failed" : "Open folder"}
              </DropdownMenuItem>
              <DropdownMenuItem onSelect={handleDeleteIntent} className="text-[var(--destructive)]">
                <Trash2 className="mr-2 size-3.5" />
                Delete
              </DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </div>

      <AlertDialog open={confirmOpen} onOpenChange={setConfirmOpen}>
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Delete Record</AlertDialogTitle>
            <AlertDialogDescription>
              Delete {record?.title ?? recordId} from the local library.
            </AlertDialogDescription>
          </AlertDialogHeader>
          {deleteError ? (
            <p className="text-[12px] text-[var(--destructive)]">{deleteError}</p>
          ) : null}
          <AlertDialogFooter>
            <AlertDialogCancel onClick={handleDeleteCancel} disabled={deletePending}>
              Cancel
            </AlertDialogCancel>
            <AlertDialogAction onClick={() => void handleConfirmDelete()} disabled={deletePending}>
              {deletePending ? "Deleting..." : "Delete"}
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>

      <Tooltip open={copyState === "copied" || copyState === "error"}>
        <TooltipTrigger asChild>
          <span className="sr-only" />
        </TooltipTrigger>
        <TooltipContent>{copyState === "copied" ? "Copied" : "Copy failed"}</TooltipContent>
      </Tooltip>
    </>
  );
}

export const RecordListItem = memo(RecordListItemInner);
