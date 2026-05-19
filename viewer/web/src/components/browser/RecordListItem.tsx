import {
  memo,
  useEffect,
  useMemo,
  useRef,
  useState,
  type FormEvent,
  type JSX,
  type MouseEvent as ReactMouseEvent,
} from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";
import { ArrowUpRight, Check, ChevronDown, Copy, FolderOpen, GitFork, MoreVertical, Search, Star, Trash2 } from "lucide-react";

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
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Popover, PopoverContent, PopoverTrigger } from "@/components/ui/popover";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { deleteRecord, openRecordFolder, promoteRecordToDataset } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import type { CategoryOption, RecordSummary } from "@/lib/types";
import { cn } from "@/lib/utils";
import { categoriesQueryOptions, viewerQueryKeys } from "@/lib/viewer-queries";
import { useViewerDispatch } from "@/lib/viewer-context";

interface RecordListItemProps {
  recordId: string;
  record: RecordSummary | null;
  repoRoot: string | null;
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

  const truncated = withoutExistingEllipsis.slice(0, maxLength).trimEnd();
  return `${truncated.replace(TRAILING_PUNCTUATION, "")}...`;
}

function formatCost(value: number | null): string | null {
  if (value === null || Number.isNaN(value)) return null;
  if (value < 0.01) return `$${value.toFixed(3)}`;
  if (value < 0.1) return `$${value.toFixed(3)}`;
  return COST_FORMATTER.format(value);
}

function formatDate(value: string | null): string | null {
  if (!value) return null;
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return null;

  const now = new Date();
  const sameYear = date.getFullYear() === now.getFullYear();
  return sameYear
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
  return agent ?? "External";
}

function RecordListItemInner({
  recordId,
  record,
  repoRoot,
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
  const [promoteOpen, setPromoteOpen] = useState(false);
  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [deletePending, setDeletePending] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const [promotePending, setPromotePending] = useState(false);
  const [promoteError, setPromoteError] = useState<string | null>(null);
  const [promoteCategoryOpen, setPromoteCategoryOpen] = useState(false);
  const [promoteCategorySearch, setPromoteCategorySearch] = useState("");
  const [promoteCategorySlug, setPromoteCategorySlug] = useState("");
  const [promoteDatasetId, setPromoteDatasetId] = useState("");
  const promoteCategorySearchInputRef = useRef<HTMLInputElement | null>(null);
  const categoriesQuery = useQuery({
    ...categoriesQueryOptions(),
    enabled: promoteOpen,
  });
  const promoteCategoryOptions: CategoryOption[] = useMemo(
    () => categoriesQuery.data ?? [],
    [categoriesQuery.data],
  );
  const promoteCategoryLoading =
    categoriesQuery.isPending || (categoriesQuery.isFetching && categoriesQuery.data == null);
  const promoteCategoryLoadError =
    categoriesQuery.error instanceof Error ? categoriesQuery.error.message : null;
  const alreadyInDataset = record?.collections.includes("dataset") ?? false;
  const summaryText = truncateWithEllipsis(record?.prompt_preview || record?.title || recordId);
  const metadata = [
    record?.creator_mode === "external_agent"
      ? externalAgentLabel(record.external_agent)
      : null,
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
  const selectedCategoryOption = useMemo(
    () => promoteCategoryOptions.find((option) => option.slug === promoteCategorySlug) ?? null,
    [promoteCategoryOptions, promoteCategorySlug],
  );
  const filteredCategoryOptions = useMemo(() => {
    const query = promoteCategorySearch.trim().toLowerCase();
    if (!query) {
      return promoteCategoryOptions;
    }
    return promoteCategoryOptions.filter((option) =>
      option.title.toLowerCase().includes(query) || option.slug.toLowerCase().includes(query),
    );
  }, [promoteCategoryOptions, promoteCategorySearch]);

  useEffect(() => {
    if (!promoteOpen) {
      return;
    }

    if (categoriesQuery.data == null) {
      return;
    }

    if (
      record?.category_slug
      && categoriesQuery.data.some((option) => option.slug === record.category_slug)
    ) {
      setPromoteCategorySlug(record.category_slug);
      return;
    }

    setPromoteCategorySlug("");
  }, [categoriesQuery.data, promoteOpen, record?.category_slug]);

  useEffect(() => {
    if (!promoteCategoryOpen) {
      return;
    }

    requestAnimationFrame(() => promoteCategorySearchInputRef.current?.focus());
  }, [promoteCategoryOpen]);

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
    dispatch({ type: "CLEAR_MULTI_SELECT" });
    onSelect(recordId);
  };

  const handleRowClick = (event: ReactMouseEvent) => {
    if (event.shiftKey && multiSelectActive) {
      event.preventDefault();
      onMultiSelectToggle(recordId, true);
      return;
    }
    if (event.metaKey || event.ctrlKey) {
      event.preventDefault();
      onMultiSelectToggle(recordId, false);
      return;
    }
    handleSelect();
  };

  const handleCheckboxClick = (event: ReactMouseEvent) => {
    event.stopPropagation();
    onMultiSelectToggle(recordId, event.shiftKey);
  };

  const handleCopyRecordPath = async () => {
    if (!repoRoot) {
      setCopyState("error");
      return;
    }

    try {
      await copyTextToClipboard(buildRecordPath(repoRoot, recordId));
      setCopyState("copied");
    } catch {
      setCopyState("error");
    }
  };

  const handleOpenRecordFolder = async () => {
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

  const handlePromoteIntent = () => {
    if (!record) {
      return;
    }
    setMenuOpen(false);
    setPromoteError(null);
    setPromoteCategoryOpen(false);
    setPromoteCategorySearch("");
    setPromoteCategorySlug(record.category_slug ?? "");
    setPromoteDatasetId("");
    setPromoteOpen(true);
  };

  const handleDeleteCancel = () => {
    if (deletePending) {
      return;
    }
    setConfirmOpen(false);
    setDeleteError(null);
  };

  const handlePromoteCancel = () => {
    if (promotePending) {
      return;
    }
    setPromoteOpen(false);
    setPromoteCategoryOpen(false);
    setPromoteError(null);
  };

  const handleConfirmDelete = async () => {
    if (deletePending) {
      return;
    }

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

  const handleConfirmPromote = async (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (promotePending) {
      return;
    }

    if (!record) {
      setPromoteError("Record details are still loading.");
      return;
    }

    if (!selectedCategoryOption) {
      setPromoteError("Select a category.");
      return;
    }

    setPromotePending(true);
    setPromoteError(null);

    try {
      await promoteRecordToDataset(recordId, {
        categorySlug: selectedCategoryOption.slug,
        categoryTitle: selectedCategoryOption.title,
        datasetId: promoteDatasetId,
      });
      setPromoteOpen(false);
      await queryClient.invalidateQueries({ queryKey: viewerQueryKeys.root() });
      dispatch({ type: "SET_BROWSER_TAB", payload: "dataset" });
    } catch (error) {
      setPromoteError(error instanceof Error ? error.message : "Failed to promote record.");
    } finally {
      setPromotePending(false);
    }
  };

  if (!record) {
    return (
      <div className="px-1.5" data-record-list-item={recordId}>
        <div
          className={cn(
            "group flex h-[68px] items-start gap-0.5 rounded-lg px-2.5 py-2 transition-colors duration-100",
            isMultiSelected || isSelected ? "bg-[var(--accent-soft)]" : "hover:bg-[var(--surface-1)]",
          )}
          onClick={handleRowClick}
        >
          <button
            type="button"
            aria-label={isMultiSelected ? "Deselect record" : "Select record"}
            onClick={handleCheckboxClick}
            className={cn(
              "mr-1.5 mt-0.5 flex shrink-0 items-center justify-center rounded-sm p-0 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
              multiSelectActive ? "opacity-100" : "opacity-0 group-hover:opacity-100",
            )}
          >
            <span
              className={cn(
                "flex size-3.5 shrink-0 items-center justify-center rounded-sm border",
                isMultiSelected
                  ? "border-[var(--accent)] bg-[var(--accent-soft)] text-[var(--accent)]"
                  : "border-[var(--border-default)] bg-[var(--surface-1)] text-transparent",
              )}
            >
              <Check className="size-3" />
            </span>
          </button>

          <button
            type="button"
            onClick={(event) => {
              event.stopPropagation();
              handleRowClick(event);
            }}
            data-record-select-trigger="true"
            className="min-w-0 flex-1 rounded-sm py-0 text-left focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]"
            title={recordId}
          >
            <p className="line-clamp-2 break-words text-[11px] leading-[1.45] text-[var(--text-secondary)]">
              {recordId}
            </p>
            <p className="mt-1 text-[9.5px] text-[var(--text-quaternary)]">Loading summary…</p>
          </button>
        </div>
      </div>
    );
  }

  return (
    <>
      <div className="px-1.5" data-record-list-item={recordId}>
        <div
          className={cn(
            "group flex h-[68px] items-start gap-0.5 rounded-lg px-2.5 py-2 transition-colors duration-100",
            isMultiSelected
              ? "bg-[var(--accent-soft)]"
              : isSelected
                ? "bg-[var(--accent-soft)]"
                : "hover:bg-[var(--surface-1)]",
          )}
          onClick={handleRowClick}
        >
          {/* Checkbox: visible when multi-select is active, or on hover */}
          <button
            type="button"
            aria-label={isMultiSelected ? "Deselect record" : "Select record"}
            onClick={handleCheckboxClick}
            className={cn(
              "mr-1.5 mt-0.5 flex shrink-0 items-center justify-center rounded-sm p-0 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
              multiSelectActive
                ? "opacity-100"
                : "opacity-0 group-hover:opacity-100",
            )}
          >
            <span
              className={cn(
                "flex size-3.5 shrink-0 items-center justify-center rounded-sm border",
                isMultiSelected
                  ? "border-[var(--accent)] bg-[var(--accent-soft)] text-[var(--accent)]"
                  : "border-[var(--border-default)] bg-[var(--surface-1)] text-transparent",
              )}
            >
              <Check className="size-3" />
            </span>
          </button>

          <button
            type="button"
            onClick={(e) => { e.stopPropagation(); handleRowClick(e); }}
            data-record-select-trigger="true"
            className="min-w-0 flex-1 rounded-sm py-0 text-left focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]"
            title={summaryText}
          >
            <p
              className={`line-clamp-2 break-words text-[11px] leading-[1.45] ${
                isSelected ? "font-medium text-[var(--text-primary)]" : "text-[var(--text-secondary)]"
              }`}
            >
              {summaryText}
            </p>

            {(metadata.length > 0 || effectiveRating != null || record?.parent_record_id) ? (
              <div className="mt-1 flex flex-wrap items-center gap-x-1 gap-y-0.5 text-[9.5px] text-[var(--text-tertiary)]">
                {record?.parent_record_id ? (
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <span
                        className="flex items-center"
                        aria-label="Forked record"
                      >
                        <GitFork className="size-[9px] text-[var(--accent)]" />
                      </span>
                    </TooltipTrigger>
                    <TooltipContent side="top">Forked from {record.parent_record_id}</TooltipContent>
                  </Tooltip>
                ) : null}
                {effectiveRatingLabel ? (
                  <span className="flex items-center gap-x-0.5">
                    {record?.parent_record_id ? <span className="text-[var(--border-strong)]">·</span> : null}
                    <Star className="size-[9px] fill-[#e0a100] text-[#e0a100]" />
                    <span className="text-[#c89400]">{effectiveRatingLabel}</span>
                  </span>
                ) : null}
                {metadata.map((item, index) => (
                  <span key={`${recordId}-${item}`} className="flex items-center gap-x-1">
                    {(index > 0 || effectiveRatingLabel || record?.parent_record_id) ? <span className="text-[var(--border-strong)]">·</span> : null}
                    <span>{item}</span>
                  </span>
                ))}
              </div>
            ) : null}
          </button>

          <DropdownMenu open={menuOpen} onOpenChange={setMenuOpen}>
            <div className={cn("relative shrink-0 pt-px", multiSelectActive && "hidden")}>
              <Tooltip>
                <TooltipTrigger asChild>
                  <DropdownMenuTrigger asChild>
                    <button
                      type="button"
                      aria-label={`Open actions for ${record.title}`}
                      className={cn(
                        "flex size-6 items-center justify-center rounded-md text-[var(--text-tertiary)] opacity-0 transition-all duration-100 group-hover:opacity-100 focus-visible:opacity-100 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
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
                {!alreadyInDataset ? (
                  <DropdownMenuItem onSelect={handlePromoteIntent}>
                    <ArrowUpRight className="size-3.5" />
                    <span>Promote to dataset</span>
                  </DropdownMenuItem>
                ) : null}
                <DropdownMenuItem onSelect={() => void handleCopyRecordPath()}>
                  <Copy className="size-3.5" />
                  <span>
                    {copyState === "copied"
                      ? "Copied object path"
                      : copyState === "error"
                        ? "Copy failed"
                        : "Copy object path"}
                  </span>
                </DropdownMenuItem>
                <DropdownMenuItem onSelect={() => void handleOpenRecordFolder()}>
                  <FolderOpen className="size-3.5" />
                  <span>
                    {openState === "opened"
                      ? "Opened object folder"
                      : openState === "error"
                        ? "Open failed"
                        : "Open object folder"}
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
        </div>
      </div>

      <Dialog
        open={promoteOpen}
        onOpenChange={(open) => {
          if (!open) {
            handlePromoteCancel();
          } else {
            setPromoteOpen(true);
          }
        }}
      >
        <DialogContent className="w-full max-w-[420px] p-5">
          <form onSubmit={handleConfirmPromote}>
            <DialogHeader>
              <DialogTitle>Promote to dataset</DialogTitle>
              <DialogDescription>
                Assign a category and optional dataset ID for this record. Leave dataset ID empty
                to auto-generate one.
              </DialogDescription>
            </DialogHeader>

            <div className="mt-4 space-y-3">
              <div className="space-y-1.5">
                <Label className="text-[11px] text-[var(--text-secondary)]">Category</Label>
                <Popover open={promoteCategoryOpen} onOpenChange={setPromoteCategoryOpen}>
                  <PopoverTrigger asChild>
                    <button
                      type="button"
                      className={cn(
                        "flex h-8 w-full items-center justify-between gap-2 rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] px-2.5 py-1.5 text-left text-[12px] text-[var(--text-primary)] transition-all duration-150 outline-none",
                        "focus-visible:border-[var(--accent)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
                        !selectedCategoryOption && "text-[var(--text-quaternary)]",
                      )}
                      disabled={promotePending}
                    >
                      <span className="truncate">
                        {selectedCategoryOption
                          ? selectedCategoryOption.title
                          : promoteCategoryLoading
                            ? "Loading categories..."
                            : "Select a category"}
                      </span>
                      <ChevronDown
                        className={cn(
                          "size-3.5 shrink-0 text-[var(--text-tertiary)] transition-transform",
                          promoteCategoryOpen && "rotate-180",
                        )}
                      />
                    </button>
                  </PopoverTrigger>
                  <PopoverContent
                    align="start"
                    className="w-[var(--radix-popover-trigger-width)] p-1"
                    onOpenAutoFocus={(event) => {
                      event.preventDefault();
                      promoteCategorySearchInputRef.current?.focus();
                    }}
                  >
                    <div className="flex items-center gap-2 border-b border-[var(--border-subtle)] px-2 py-1.5">
                      <Search className="size-3 shrink-0 text-[var(--text-quaternary)]" />
                      <Input
                        ref={promoteCategorySearchInputRef}
                        value={promoteCategorySearch}
                        onChange={(event) => setPromoteCategorySearch(event.target.value)}
                        placeholder="Search categories..."
                        className="h-6 border-0 bg-transparent px-0 py-0 shadow-none focus-visible:ring-0"
                        disabled={promotePending}
                      />
                    </div>

                    {promoteCategoryLoadError ? (
                      <div className="px-2.5 py-2 text-[11px] text-[var(--destructive)]">
                        {promoteCategoryLoadError}
                      </div>
                    ) : promoteCategoryLoading ? (
                      <div className="px-2.5 py-2 text-[11px] text-[var(--text-quaternary)]">
                        Loading categories...
                      </div>
                    ) : filteredCategoryOptions.length === 0 ? (
                      <div className="px-2.5 py-2 text-[11px] text-[var(--text-quaternary)]">
                        No matching categories
                      </div>
                    ) : (
                      <div role="listbox" aria-label="Categories" className="max-h-56 overflow-y-auto py-1">
                        {filteredCategoryOptions.map((option) => {
                          const selected = option.slug === promoteCategorySlug;
                          return (
                            <button
                              key={option.slug}
                              type="button"
                              role="option"
                              aria-selected={selected}
                              onClick={() => {
                                setPromoteCategorySlug(option.slug);
                                setPromoteCategoryOpen(false);
                                setPromoteCategorySearch("");
                                setPromoteError(null);
                              }}
                              className="flex w-full items-center gap-2 rounded-md px-2.5 py-1.5 text-left text-[12px] text-[var(--text-primary)] outline-none transition-colors hover:bg-[var(--accent-soft)] focus-visible:bg-[var(--accent-soft)]"
                            >
                              <span
                                className={cn(
                                  "flex size-3.5 shrink-0 items-center justify-center rounded-sm border",
                                  selected
                                    ? "border-[var(--accent)] bg-[var(--accent-soft)] text-[var(--accent)]"
                                    : "border-[var(--border-default)] bg-[var(--surface-1)] text-transparent",
                                )}
                              >
                                <Check className="size-3" />
                              </span>
                              <span className="min-w-0 flex-1">
                                <span className="block truncate">{option.title}</span>
                                <span className="block truncate text-[10px] text-[var(--text-tertiary)]">
                                  {option.slug}
                                </span>
                              </span>
                            </button>
                          );
                        })}
                      </div>
                    )}
                  </PopoverContent>
                </Popover>
              </div>

              <div className="space-y-1.5">
                <Label htmlFor={`${record.record_id}-dataset-id`} className="text-[11px] text-[var(--text-secondary)]">
                  Dataset ID
                </Label>
                <Input
                  id={`${record.record_id}-dataset-id`}
                  value={promoteDatasetId}
                  onChange={(event) => setPromoteDatasetId(event.target.value)}
                  placeholder="Optional, e.g. ds_hinges_0007"
                  disabled={promotePending}
                />
              </div>
            </div>

            {promoteError ? (
              <div className="mt-3 rounded-lg border border-[rgba(209,52,21,0.1)] bg-[rgba(209,52,21,0.04)] px-3 py-2 text-[11px] text-[var(--destructive)]">
                {promoteError}
              </div>
            ) : null}

            <div className="mt-5 flex items-center justify-end gap-2">
              <Button type="button" variant="outline" onClick={handlePromoteCancel} disabled={promotePending}>
                Cancel
              </Button>
              <Button type="submit" disabled={promotePending}>
                {promotePending ? "Promoting..." : "Promote"}
              </Button>
            </div>
          </form>
        </DialogContent>
      </Dialog>

      <AlertDialog open={confirmOpen} onOpenChange={setConfirmOpen}>
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Delete record?</AlertDialogTitle>
            <AlertDialogDescription>
              This will permanently remove this record and its stored files from the viewer.
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

export const RecordListItem = memo(
  RecordListItemInner,
  (previous, next) =>
    previous.recordId === next.recordId
    && previous.record === next.record
    && previous.repoRoot === next.repoRoot
    && previous.isSelected === next.isSelected
    && previous.multiSelectActive === next.multiSelectActive
    && previous.isMultiSelected === next.isMultiSelected
    && previous.onSelect === next.onSelect
    && previous.onMultiSelectToggle === next.onMultiSelectToggle,
);
