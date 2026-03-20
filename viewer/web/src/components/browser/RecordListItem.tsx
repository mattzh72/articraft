import {
  useEffect,
  useId,
  useMemo,
  useRef,
  useState,
  type FormEvent,
  type JSX,
  type MouseEvent as ReactMouseEvent,
} from "react";
import { ArrowUpRight, Check, ChevronDown, Copy, FolderOpen, MoreVertical, Search, Star, Trash2 } from "lucide-react";

import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import { deleteRecord, fetchBootstrap, fetchCategories, openRecordFolder, promoteRecordToDataset } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import type { CategoryOption, RecordSummary } from "@/lib/types";
import { cn } from "@/lib/utils";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

interface RecordListItemProps {
  record: RecordSummary;
  multiSelectActive: boolean;
  isMultiSelected: boolean;
  onMultiSelectToggle: (recordId: string, shiftKey: boolean) => void;
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

export function RecordListItem({ record, multiSelectActive, isMultiSelected, onMultiSelectToggle }: RecordListItemProps): JSX.Element {
  const { bootstrap, selection } = useViewer();
  const dispatch = useViewerDispatch();
  const [menuOpen, setMenuOpen] = useState(false);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [promoteOpen, setPromoteOpen] = useState(false);
  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");
  const [deletePending, setDeletePending] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const [promotePending, setPromotePending] = useState(false);
  const [promoteError, setPromoteError] = useState<string | null>(null);
  const [promoteCategoryOptions, setPromoteCategoryOptions] = useState<CategoryOption[]>([]);
  const [promoteCategoryLoading, setPromoteCategoryLoading] = useState(false);
  const [promoteCategoryLoadError, setPromoteCategoryLoadError] = useState<string | null>(null);
  const [promoteCategoryOpen, setPromoteCategoryOpen] = useState(false);
  const [promoteCategorySearch, setPromoteCategorySearch] = useState("");
  const [promoteCategorySlug, setPromoteCategorySlug] = useState("");
  const [promoteDatasetId, setPromoteDatasetId] = useState("");
  const menuRef = useRef<HTMLDivElement | null>(null);
  const promoteCategoryPopoverRef = useRef<HTMLDivElement | null>(null);
  const promoteCategorySearchInputRef = useRef<HTMLInputElement | null>(null);
  const titleId = useId();
  const descriptionId = useId();
  const promoteTitleId = useId();
  const promoteDescriptionId = useId();
  const isSelected = selection?.kind === "record" && selection.recordId === record.record_id;
  const alreadyInDataset = record.collections.includes("dataset");
  const summaryText = truncateWithEllipsis(record.prompt_preview || record.title);
  const metadata = [
    record.model_id,
    record.turn_count !== null
      ? `${record.turn_count} turn${record.turn_count === 1 ? "" : "s"}`
      : null,
    formatCost(record.total_cost_usd),
    formatDate(record.updated_at ?? record.created_at),
  ].filter((item): item is string => Boolean(item));
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
    if (!promoteOpen) {
      return;
    }

    let cancelled = false;
    setPromoteCategoryLoading(true);
    setPromoteCategoryLoadError(null);

    fetchCategories()
      .then((categories) => {
        if (cancelled) {
          return;
        }
        setPromoteCategoryOptions(categories);
        setPromoteCategoryLoading(false);
        if (record.category_slug && categories.some((option) => option.slug === record.category_slug)) {
          setPromoteCategorySlug(record.category_slug);
        } else {
          setPromoteCategorySlug("");
        }
      })
      .catch((error) => {
        if (cancelled) {
          return;
        }
        setPromoteCategoryOptions([]);
        setPromoteCategoryLoading(false);
        setPromoteCategoryLoadError(error instanceof Error ? error.message : "Failed to load categories.");
      });

    return () => {
      cancelled = true;
    };
  }, [promoteOpen, record.category_slug]);

  useEffect(() => {
    if (!promoteOpen) {
      return;
    }

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape" && !promotePending) {
        setPromoteOpen(false);
        setPromoteError(null);
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [promoteOpen, promotePending]);

  useEffect(() => {
    if (!promoteCategoryOpen) {
      return;
    }

    promoteCategorySearchInputRef.current?.focus();

    const handlePointerDown = (event: MouseEvent) => {
      if (!promoteCategoryPopoverRef.current) {
        return;
      }
      if (event.target instanceof Node && !promoteCategoryPopoverRef.current.contains(event.target)) {
        setPromoteCategoryOpen(false);
      }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setPromoteCategoryOpen(false);
      }
    };

    document.addEventListener("mousedown", handlePointerDown);
    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("mousedown", handlePointerDown);
      document.removeEventListener("keydown", handleKeyDown);
    };
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
    dispatch({ type: "SELECT_ITEM", payload: { kind: "record", recordId: record.record_id } });
  };

  const handleRowClick = (event: ReactMouseEvent) => {
    if (event.shiftKey && multiSelectActive) {
      event.preventDefault();
      onMultiSelectToggle(record.record_id, true);
      return;
    }
    if (event.metaKey || event.ctrlKey) {
      event.preventDefault();
      onMultiSelectToggle(record.record_id, false);
      return;
    }
    handleSelect();
  };

  const handleCheckboxClick = (event: ReactMouseEvent) => {
    event.stopPropagation();
    onMultiSelectToggle(record.record_id, event.shiftKey);
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

  const handlePromoteIntent = () => {
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

  const handleConfirmPromote = async (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (promotePending) {
      return;
    }

    if (!selectedCategoryOption) {
      setPromoteError("Select a category.");
      return;
    }

    setPromotePending(true);
    setPromoteError(null);

    try {
      await promoteRecordToDataset(record.record_id, {
        categorySlug: selectedCategoryOption.slug,
        categoryTitle: selectedCategoryOption.title,
        datasetId: promoteDatasetId,
      });
      setPromoteOpen(false);

      try {
        const refreshedBootstrap = await fetchBootstrap();
        dispatch({ type: "SET_BOOTSTRAP", payload: refreshedBootstrap });
        dispatch({ type: "SET_BROWSER_TAB", payload: "dataset" });
      } catch (error) {
        dispatch({
          type: "SET_ERROR",
          payload: error instanceof Error ? error.message : "Promotion succeeded, but viewer refresh failed.",
        });
      }
    } catch (error) {
      setPromoteError(error instanceof Error ? error.message : "Failed to promote record.");
    } finally {
      setPromotePending(false);
    }
  };

  return (
    <>
      <div className="px-1.5" data-record-list-item={record.record_id}>
        <div
          className={cn(
            "group flex items-start gap-0.5 rounded-lg px-2.5 py-2 transition-colors duration-100",
            isMultiSelected
              ? "bg-[var(--accent-soft)]"
              : isSelected
                ? "border-l-2 border-l-[var(--accent)] bg-[var(--accent-soft)] pl-2"
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

          <div ref={menuRef} className={cn("relative shrink-0 pt-px", multiSelectActive && "hidden")}>
            <Tooltip>
              <TooltipTrigger asChild>
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
              </TooltipTrigger>
              {!menuOpen ? <TooltipContent side="right">Actions</TooltipContent> : null}
            </Tooltip>

            {menuOpen ? (
              <div
                role="menu"
                className="absolute right-0 top-full z-20 mt-1 min-w-[11rem] overflow-hidden rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 text-[var(--text-primary)] shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
              >
                {!alreadyInDataset ? (
                  <button
                    type="button"
                    role="menuitem"
                    onClick={handlePromoteIntent}
                    className="flex w-full items-center gap-2 whitespace-nowrap rounded-md px-2.5 py-1.5 text-left font-sans text-[12px] font-medium text-[var(--text-secondary)] transition-colors hover:bg-[var(--surface-1)] hover:text-[var(--text-primary)]"
                  >
                    <ArrowUpRight className="size-3.5" />
                    <span>Promote to dataset</span>
                  </button>
                ) : null}
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

      {promoteOpen ? (
        <div
          className="fixed inset-0 z-50 flex items-center justify-center bg-[rgba(0,0,0,0.15)] px-4 backdrop-blur-[2px]"
          onMouseDown={(event) => {
            if (event.target === event.currentTarget) {
              handlePromoteCancel();
            }
          }}
        >
          <form
            role="dialog"
            aria-modal="true"
            aria-labelledby={promoteTitleId}
            aria-describedby={promoteDescriptionId}
            className="w-full max-w-[420px] rounded-xl border border-[var(--border-default)] bg-[var(--surface-0)] p-5 shadow-[0_16px_48px_rgba(0,0,0,0.12)]"
            onMouseDown={(event) => event.stopPropagation()}
            onSubmit={handleConfirmPromote}
          >
            <div className="space-y-1.5">
              <h2 id={promoteTitleId} className="text-[14px] font-semibold text-[var(--text-primary)]">
                Promote to dataset
              </h2>
              <p id={promoteDescriptionId} className="text-[12px] leading-5 text-[var(--text-secondary)]">
                Assign a category and optional dataset ID for this record. Leave dataset ID empty to auto-generate one.
              </p>
            </div>

            <div className="mt-4 space-y-3">
              <div ref={promoteCategoryPopoverRef} className="relative space-y-1.5">
                <Label className="text-[11px] text-[var(--text-secondary)]">
                  Category
                </Label>
                <button
                  type="button"
                  aria-expanded={promoteCategoryOpen}
                  aria-haspopup="listbox"
                  onClick={() => setPromoteCategoryOpen((current) => !current)}
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
                  <ChevronDown className={cn("size-3.5 shrink-0 text-[var(--text-tertiary)] transition-transform", promoteCategoryOpen && "rotate-180")} />
                </button>

                {promoteCategoryOpen ? (
                  <div className="absolute left-0 top-[calc(100%+0.375rem)] z-10 w-full rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]">
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
                  </div>
                ) : null}
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
        </div>
      ) : null}

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
                This will permanently remove this record and its stored files from the viewer.
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
