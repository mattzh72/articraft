import { useEffect, useId, useRef, useState, type JSX, type MouseEvent as ReactMouseEvent } from "react";
import { Copy, FolderOpen, MoreVertical, Star, Trash2 } from "lucide-react";

import { Button } from "@/components/ui/button";
import { deleteRecord, fetchBootstrap, openRecordFolder } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import type { RecordSummary } from "@/lib/types";
import { cn } from "@/lib/utils";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

interface RecordListItemProps {
  record: RecordSummary;
}

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

function formatCost(value: number | null): string | null {
  if (value === null || Number.isNaN(value)) return null;
  if (value < 0.01) return `$${value.toFixed(3)}`;
  if (value < 0.1) return `$${value.toFixed(3)}`;
  return new Intl.NumberFormat(undefined, {
    style: "currency",
    currency: "USD",
    minimumFractionDigits: 2,
    maximumFractionDigits: 2,
  }).format(value);
}

function formatDate(value: string | null): string | null {
  if (!value) return null;
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return null;

  const now = new Date();
  const sameYear = date.getFullYear() === now.getFullYear();
  return new Intl.DateTimeFormat(undefined, {
    month: "short",
    day: "numeric",
    ...(sameYear ? {} : { year: "numeric" }),
  }).format(date);
}

export function RecordListItem({ record }: RecordListItemProps): JSX.Element {
  const { bootstrap, selectedRecordId } = useViewer();
  const dispatch = useViewerDispatch();
  const [menuOpen, setMenuOpen] = useState(false);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [deletePending, setDeletePending] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const menuRef = useRef<HTMLDivElement | null>(null);
  const titleId = useId();
  const descriptionId = useId();
  const isSelected = selectedRecordId === record.record_id;
  const summaryText = truncateWithEllipsis(record.prompt_preview || record.title);
  const metadata = [
    record.model_id,
    record.turn_count !== null
      ? `${record.turn_count} turn${record.turn_count === 1 ? "" : "s"}`
      : null,
    formatCost(record.total_cost_usd),
    formatDate(record.updated_at ?? record.created_at),
  ].filter((item): item is string => Boolean(item));

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
    if (copyState === "idle") {
      return;
    }

    const timeoutId = window.setTimeout(() => {
      setCopyState("idle");
    }, 1800);

    return () => {
      window.clearTimeout(timeoutId);
    };
  }, [copyState]);

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

  const handleSelect = () => {
    dispatch({ type: "SELECT_RECORD", payload: record.record_id });
  };

  const handleMenuToggle = (event: ReactMouseEvent<HTMLButtonElement>) => {
    event.stopPropagation();
    setMenuOpen((current) => !current);
  };

  const handleCopyRecordPath = async () => {
    if (!bootstrap) {
      setCopyState("error");
      return;
    }

    try {
      await copyTextToClipboard(buildRecordPath(bootstrap.repo_root, record.record_id));
      setCopyState("copied");
    } catch {
      setCopyState("error");
    }
  };

  const handleOpenRecordFolder = async () => {
    try {
      await openRecordFolder(record.record_id);
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
      await deleteRecord(record.record_id);
      dispatch({ type: "DELETE_RECORD_LOCAL", payload: record.record_id });
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
      setDeleteError(error instanceof Error ? error.message : "Failed to delete record.");
    } finally {
      setDeletePending(false);
    }
  };

  return (
    <>
      <div className="px-1.5" data-record-list-item={record.record_id}>
        <div
          className={cn(
            "group flex items-start gap-0.5 rounded-lg px-2.5 py-2 transition-colors duration-100",
            isSelected ? "bg-[var(--accent-soft)]" : "hover:bg-[var(--surface-1)]",
          )}
        >
          <button
            type="button"
            onClick={handleSelect}
            data-record-select-trigger="true"
            className="min-w-0 flex-1 rounded-sm py-0 text-left focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]"
            title={summaryText}
          >
            <p
              className={`break-words text-[11px] leading-[1.45] ${
                isSelected ? "font-medium text-[var(--text-primary)]" : "text-[var(--text-secondary)]"
              }`}
            >
              {summaryText}
            </p>

            {(metadata.length > 0 || record.rating) ? (
              <div className="mt-1 flex flex-wrap items-center gap-x-1 gap-y-0.5 text-[9.5px] text-[var(--text-tertiary)]">
                {record.rating ? (
                  <span className="flex items-center gap-x-0.5">
                    <Star className="size-[9px] fill-[#e0a100] text-[#e0a100]" />
                    <span className="text-[#c89400]">{record.rating}</span>
                  </span>
                ) : null}
                {metadata.map((item, index) => (
                  <span key={`${record.record_id}-${item}`} className="flex items-center gap-x-1">
                    {(index > 0 || record.rating) ? <span className="text-[var(--border-strong)]">·</span> : null}
                    <span>{item}</span>
                  </span>
                ))}
              </div>
            ) : null}
          </button>

          <div ref={menuRef} className="relative shrink-0 pt-px">
            <button
              type="button"
              aria-label={`Open actions for ${record.title}`}
              aria-haspopup="menu"
              aria-expanded={menuOpen}
              onClick={handleMenuToggle}
              className={cn(
                "flex size-6 items-center justify-center rounded-md text-[var(--text-tertiary)] opacity-0 transition-all duration-100 group-hover:opacity-100 focus-visible:opacity-100 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
                menuOpen
                  ? "bg-[var(--surface-0)] text-[var(--text-primary)] opacity-100 shadow-[0_1px_3px_rgba(0,0,0,0.06)]"
                  : "hover:bg-[var(--surface-0)] hover:text-[var(--text-primary)]",
              )}
            >
              <MoreVertical className="size-3" />
            </button>

            {menuOpen ? (
              <div
                role="menu"
                className="absolute right-0 top-full z-20 mt-1 min-w-[11rem] overflow-hidden rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 text-[var(--text-primary)] shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
              >
                <button
                  type="button"
                  role="menuitem"
                  onClick={handleCopyRecordPath}
                  className="flex w-full items-center gap-2 whitespace-nowrap rounded-md px-2.5 py-1.5 text-left font-sans text-[12px] font-medium text-[var(--text-secondary)] transition-colors hover:bg-[var(--surface-1)] hover:text-[var(--text-primary)]"
                >
                  <Copy className="size-3.5" />
                  <span>
                    {copyState === "copied"
                      ? "Copied object path"
                      : copyState === "error"
                        ? "Copy failed"
                        : "Copy object path"}
                  </span>
                </button>
                <button
                  type="button"
                  role="menuitem"
                  onClick={handleOpenRecordFolder}
                  className="flex w-full items-center gap-2 whitespace-nowrap rounded-md px-2.5 py-1.5 text-left font-sans text-[12px] font-medium text-[var(--text-secondary)] transition-colors hover:bg-[var(--surface-1)] hover:text-[var(--text-primary)]"
                >
                  <FolderOpen className="size-3.5" />
                  <span>
                    {openState === "opened"
                      ? "Opened object folder"
                      : openState === "error"
                        ? "Open failed"
                        : "Open object folder"}
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
                Delete record?
              </h2>
              <p id={descriptionId} className="text-[12px] leading-5 text-[var(--text-secondary)]">
                This will permanently remove <span className="font-medium text-[var(--text-primary)]">{record.title}</span> and
                its stored files from the viewer.
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
