import { useEffect, useLayoutEffect, useMemo, useRef, useState, type JSX } from "react";
import { createPortal } from "react-dom";
import { Check, ChevronDown, Search } from "lucide-react";

import type { CostFilter, RatingFilter, RatingFilterValue, RecordSummary, SupercategoryOption, TimeFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { cn, formatCategoryLabel } from "@/lib/utils";

import type { TimeFilterPoint } from "@/lib/types";

const TIME_STEPS: Array<{ value: TimeFilterPoint | null; label: string }> = [
  { value: null, label: "All time" },
  { value: "1y", label: "1 year" },
  { value: "180d", label: "6 months" },
  { value: "90d", label: "90 days" },
  { value: "60d", label: "60 days" },
  { value: "30d", label: "30 days" },
  { value: "14d", label: "14 days" },
  { value: "7d", label: "7 days" },
  { value: "3d", label: "3 days" },
  { value: "24h", label: "24 hours" },
  { value: "12h", label: "12 hours" },
  { value: "6h", label: "6 hours" },
  { value: "1h", label: "1 hour" },
  { value: null, label: "Now" },
];
const TIME_OLDEST_MAX = 0;
const TIME_NEWEST_MAX = TIME_STEPS.length - 1;
const TIME_STEP_INDEX = new Map(TIME_STEPS.map((s, i) => [s.value, i]));

const TIME_TICKS: Array<{ index: number; label: string }> = [
  { index: 0, label: "All" },
  { index: 3, label: "90d" },
  { index: 5, label: "30d" },
  { index: 7, label: "7d" },
  { index: 9, label: "24h" },
  { index: TIME_STEPS.length - 1, label: "Now" },
];

function timeFilterToSlider(filter: TimeFilter): [number, number] {
  const oldest = filter.oldest ? (TIME_STEP_INDEX.get(filter.oldest) ?? TIME_OLDEST_MAX) : TIME_OLDEST_MAX;
  const newest = filter.newest ? (TIME_STEP_INDEX.get(filter.newest) ?? TIME_NEWEST_MAX) : TIME_NEWEST_MAX;
  return [oldest, newest];
}

function sliderToTimeFilter(values: [number, number]): TimeFilter {
  const oldestStep = TIME_STEPS[values[0]];
  const newestStep = TIME_STEPS[values[1]];
  return {
    oldest: values[0] === TIME_OLDEST_MAX ? null : oldestStep?.value ?? null,
    newest: values[1] === TIME_NEWEST_MAX ? null : newestStep?.value ?? null,
  };
}

function timeFilterLabel(filter: TimeFilter): string {
  if (!filter.oldest && !filter.newest) return "Any time";
  const oldestLabel = filter.oldest ? (TIME_STEPS.find((s) => s.value === filter.oldest)?.label ?? filter.oldest) : null;
  const newestLabel = filter.newest ? (TIME_STEPS.find((s) => s.value === filter.newest)?.label ?? filter.newest) : null;
  if (oldestLabel && newestLabel) return `${newestLabel} – ${oldestLabel} ago`;
  if (oldestLabel) return `Last ${oldestLabel}`;
  if (newestLabel) return `Older than ${newestLabel}`;
  return "Any time";
}

const ratingOptions: Array<{ value: RatingFilterValue; label: string }> = [
  { value: "5", label: "5 stars" },
  { value: "4", label: "4 stars" },
  { value: "3", label: "3 stars" },
  { value: "2", label: "2 stars" },
  { value: "1", label: "1 star" },
  { value: "unrated", label: "Unrated" },
];

function uniqueRecords(records: Array<RecordSummary | null>): RecordSummary[] {
  const seen = new Map<string, RecordSummary>();

  for (const record of records) {
    if (!record) continue;
    if (!seen.has(record.record_id)) {
      seen.set(record.record_id, record);
    }
  }

  return Array.from(seen.values());
}

type CategoryOption = {
  value: string;
  label: string;
  group?: string;
};

type CostBounds = {
  min: number;
  max: number;
};

const costFormatter = new Intl.NumberFormat("en-US", {
  style: "currency",
  currency: "USD",
  minimumFractionDigits: 2,
  maximumFractionDigits: 3,
});

const COST_FILTER_EPSILON = 0.0005;

function categoryTriggerLabel(options: CategoryOption[], selectedValues: string[]): string {
  if (selectedValues.length === 0) {
    return "All categories";
  }
  if (selectedValues.length === 1) {
    return options.find((option) => option.value === selectedValues[0])?.label ?? selectedValues[0];
  }
  return `${selectedValues.length} categories`;
}

function filterTriggerClass(active: boolean, className?: string): string {
  return cn(
    "h-7 w-auto min-w-0 rounded-full px-2.5 text-[10px]",
    active
      ? "border-[var(--accent)] bg-[var(--accent-soft)] text-[var(--accent)] hover:border-[var(--accent)] hover:bg-[var(--accent-hover)] focus-visible:border-[var(--accent)] data-[placeholder]:text-[var(--accent)]/80 [&_svg]:text-[var(--accent)]"
      : "border-[var(--border-default)] bg-[var(--surface-1)] text-[var(--text-primary)]",
    className,
  );
}

function MultiSelectFilter({
  options,
  selectedValues,
  onChange,
  title,
  searchPlaceholder,
  noMatchLabel,
  triggerLabel,
}: {
  options: CategoryOption[];
  selectedValues: string[];
  onChange: (nextValues: string[]) => void;
  title: string;
  searchPlaceholder: string;
  noMatchLabel: string;
  triggerLabel: (options: CategoryOption[], selectedValues: string[]) => string;
}): JSX.Element {
  const [open, setOpen] = useState(false);
  const [search, setSearch] = useState("");
  const containerRef = useRef<HTMLDivElement | null>(null);
  const menuRef = useRef<HTMLDivElement | null>(null);
  const searchRef = useRef<HTMLInputElement | null>(null);
  const [menuOffsetLeft, setMenuOffsetLeft] = useState(0);
  const hasActiveFilter = selectedValues.length > 0;

  useLayoutEffect(() => {
    if (!open || !containerRef.current || !menuRef.current) {
      setMenuOffsetLeft(0);
      return;
    }

    const updateOffset = () => {
      if (!containerRef.current || !menuRef.current) {
        return;
      }

      const menuRect = menuRef.current.getBoundingClientRect();
      const sidebarRect = containerRef.current.closest("aside")?.getBoundingClientRect();
      if (!sidebarRect) {
        setMenuOffsetLeft(0);
        return;
      }

      const maxRight = sidebarRect.right - 12;
      const overflowRight = menuRect.right - maxRight;
      setMenuOffsetLeft(overflowRight > 0 ? -overflowRight : 0);
    };

    updateOffset();
    window.addEventListener("resize", updateOffset);
    window.addEventListener("scroll", updateOffset, true);
    return () => {
      window.removeEventListener("resize", updateOffset);
      window.removeEventListener("scroll", updateOffset, true);
    };
  }, [open]);

  useEffect(() => {
    if (!open) {
      return;
    }

    const handlePointerDown = (event: MouseEvent) => {
      if (!containerRef.current) {
        return;
      }
      if (event.target instanceof Node && !containerRef.current.contains(event.target)) {
        setOpen(false);
      }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setOpen(false);
      }
    };

    document.addEventListener("mousedown", handlePointerDown);
    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("mousedown", handlePointerDown);
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [open]);

  useEffect(() => {
    if (open) {
      requestAnimationFrame(() => searchRef.current?.focus());
    } else {
      setSearch("");
    }
  }, [open]);

  const selectedSet = useMemo(() => new Set(selectedValues), [selectedValues]);
  const resolvedTriggerLabel = triggerLabel(options, selectedValues);

  const filteredOptions = useMemo(() => {
    const query = search.trim().toLowerCase();
    if (!query) return options;
    return options.filter((o) => o.label.toLowerCase().includes(query) || o.value.toLowerCase().includes(query));
  }, [options, search]);

  const toggleValue = (value: string) => {
    if (selectedSet.has(value)) {
      onChange(selectedValues.filter((item) => item !== value));
      return;
    }

    onChange([...selectedValues, value]);
  };

  return (
    <div ref={containerRef} className="relative">
      <button
        type="button"
        aria-expanded={open}
        aria-haspopup="listbox"
        onClick={() => setOpen((current) => !current)}
        className={cn(
          "flex max-w-[14rem] items-center justify-between gap-2 border outline-none transition-all duration-150 hover:border-[var(--border-strong)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
          filterTriggerClass(hasActiveFilter),
          !hasActiveFilter && "focus-visible:border-[var(--accent)]",
        )}
      >
        <span className="truncate">{resolvedTriggerLabel}</span>
        <ChevronDown
          className={cn(
            "size-3 transition-transform duration-150",
            hasActiveFilter ? "text-[var(--accent)]" : "text-[var(--text-tertiary)]",
            open && "rotate-180",
          )}
        />
      </button>

      {open ? (
        <div
          ref={menuRef}
          className="absolute left-0 top-[calc(100%+0.375rem)] z-50 w-64 rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
          style={menuOffsetLeft === 0 ? undefined : { left: menuOffsetLeft }}
        >
          <div className="flex items-center justify-between px-2 py-1">
            <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-tertiary)]">{title}</span>
            {selectedValues.length > 0 ? (
              <button
                type="button"
                onClick={() => onChange([])}
                className="text-[10px] text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-secondary)]"
              >
                Clear
              </button>
            ) : null}
          </div>

          {options.length > 6 ? (
            <div className="relative px-1.5 pb-1">
              <Search className="pointer-events-none absolute left-3.5 top-1/2 size-3 -translate-y-1/2 text-[var(--text-tertiary)]" />
              <input
                ref={searchRef}
                type="text"
                value={search}
                onChange={(e) => setSearch(e.target.value)}
                placeholder={searchPlaceholder}
                className="w-full rounded-md border border-[var(--border-default)] bg-[var(--surface-1)] py-1 pl-7 pr-2 text-[12px] text-[var(--text-primary)] placeholder:text-[var(--text-tertiary)] outline-none transition-colors focus:border-[var(--accent)]"
              />
            </div>
          ) : null}

          <div role="listbox" aria-multiselectable="true" className="max-h-64 overflow-y-auto">
            {filteredOptions.length === 0 ? (
              <div className="px-2.5 py-3 text-center text-[11px] text-[var(--text-tertiary)]">
                {noMatchLabel}
              </div>
            ) : null}
            {filteredOptions.map((option, index) => {
              const selected = selectedSet.has(option.value);
              const showGroupHeader =
                option.group &&
                (index === 0 || filteredOptions[index - 1]?.group !== option.group);
              return (
                <div key={option.value}>
                  {showGroupHeader ? (
                    <div className="px-2.5 pb-0.5 pt-2 text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-quaternary)]">
                      {option.group}
                    </div>
                  ) : null}
                  <button
                    type="button"
                    role="option"
                    aria-selected={selected}
                    onClick={() => toggleValue(option.value)}
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
                    <span className="truncate">{option.label}</span>
                  </button>
                </div>
              );
            })}
          </div>
        </div>
      ) : null}
    </div>
  );
}

function ratingTriggerLabel(options: CategoryOption[], selectedValues: string[]): string {
  if (selectedValues.length === 0) {
    return "Any rating";
  }
  if (selectedValues.length === 1) {
    return options.find((option) => option.value === selectedValues[0])?.label ?? selectedValues[0];
  }
  return `${selectedValues.length} ratings`;
}

function authorTriggerLabel(options: CategoryOption[], selectedValues: string[]): string {
  if (selectedValues.length === 0) {
    return "All authors";
  }
  if (selectedValues.length === 1) {
    return options.find((option) => option.value === selectedValues[0])?.label ?? selectedValues[0];
  }
  return `${selectedValues.length} authors`;
}

function formatCostValue(value: number): string {
  return costFormatter.format(value);
}

function getCostBounds(records: RecordSummary[], selectedRunId: string | null): CostBounds | null {
  const costs = records
    .filter((record) => !selectedRunId || record.run_id === selectedRunId)
    .map((record) => record.total_cost_usd)
    .filter((value): value is number => Number.isFinite(value));

  if (costs.length === 0) {
    return null;
  }

  return {
    min: Math.min(...costs),
    max: Math.max(...costs),
  };
}

function sameCostFilter(left: CostFilter, right: CostFilter): boolean {
  return left.min === right.min && left.max === right.max;
}

function normalizeCostFilterForBounds(costFilter: CostFilter, bounds: CostBounds): CostFilter {
  const clampedMin = costFilter.min == null ? null : Math.min(Math.max(costFilter.min, bounds.min), bounds.max);
  const clampedMax = costFilter.max == null ? null : Math.max(bounds.min, Math.min(costFilter.max, bounds.max));

  const normalized =
    clampedMin != null && clampedMax != null && clampedMin > clampedMax
      ? { min: clampedMax, max: clampedMin }
      : { min: clampedMin, max: clampedMax };

  return {
    min:
      normalized.min != null && normalized.min <= bounds.min + COST_FILTER_EPSILON
        ? null
        : normalized.min,
    max:
      normalized.max != null && normalized.max >= bounds.max - COST_FILTER_EPSILON
        ? null
        : normalized.max,
  };
}

function buildCostFilterFromSlider(values: number[], bounds: CostBounds): CostFilter {
  const [nextMin = bounds.min, nextMax = bounds.max] = values;
  return normalizeCostFilterForBounds({ min: nextMin, max: nextMax }, bounds);
}

function costTriggerLabel(bounds: CostBounds | null, costFilter: CostFilter): string {
  if (!bounds) {
    return "No cost data";
  }

  const min = costFilter.min ?? bounds.min;
  const max = costFilter.max ?? bounds.max;
  if (costFilter.min == null && costFilter.max == null) {
    return "Any cost";
  }

  return `${formatCostValue(min)} to ${formatCostValue(max)}`;
}

function TimeRangeFilter({
  value,
  onChange,
}: {
  value: TimeFilter;
  onChange: (nextValue: TimeFilter) => void;
}): JSX.Element {
  const [open, setOpen] = useState(false);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const triggerRef = useRef<HTMLButtonElement | null>(null);
  const [popoverPosition, setPopoverPosition] = useState<{ top: number; left: number; width: number } | null>(null);

  const hasActiveFilter = value.oldest != null || value.newest != null;
  const sliderValue = timeFilterToSlider(value);

  useEffect(() => {
    if (!open || !triggerRef.current) {
      return;
    }

    const updatePosition = () => {
      if (!triggerRef.current) {
        return;
      }
      const rect = triggerRef.current.getBoundingClientRect();
      setPopoverPosition({
        top: rect.bottom + 6,
        left: rect.left,
        width: Math.max(rect.width, 288),
      });
    };

    updatePosition();
    window.addEventListener("resize", updatePosition);
    window.addEventListener("scroll", updatePosition, true);
    return () => {
      window.removeEventListener("resize", updatePosition);
      window.removeEventListener("scroll", updatePosition, true);
    };
  }, [open]);

  useEffect(() => {
    if (!open) {
      return;
    }

    const handlePointerDown = (event: MouseEvent) => {
      if (!containerRef.current) {
        return;
      }
      if (event.target instanceof Node && !containerRef.current.contains(event.target)) {
        setOpen(false);
      }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setOpen(false);
      }
    };

    document.addEventListener("mousedown", handlePointerDown);
    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("mousedown", handlePointerDown);
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [open]);

  return (
    <div ref={containerRef} className="relative">
      <button
        ref={triggerRef}
        type="button"
        aria-expanded={open}
        aria-haspopup="dialog"
        onClick={() => setOpen((current) => !current)}
        className={cn(
          "flex max-w-[16rem] items-center justify-between gap-2 border outline-none transition-all duration-150 hover:border-[var(--border-strong)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
          filterTriggerClass(hasActiveFilter),
          !hasActiveFilter && "focus-visible:border-[var(--accent)]",
        )}
      >
        <span className="truncate">{timeFilterLabel(value)}</span>
        <ChevronDown
          className={cn(
            "size-3 transition-transform duration-150",
            hasActiveFilter ? "text-[var(--accent)]" : "text-[var(--text-tertiary)]",
            open && "rotate-180",
          )}
        />
      </button>

      {open && popoverPosition
        ? createPortal(
            <div
              className="fixed z-[120] rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
              style={{
                top: popoverPosition.top,
                left: popoverPosition.left,
                width: popoverPosition.width,
              }}
            >
              <div className="flex items-center justify-between px-2 py-1">
                <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-tertiary)]">Time range</span>
                {hasActiveFilter ? (
                  <button
                    type="button"
                    onClick={() => onChange({ oldest: null, newest: null })}
                    className="text-[10px] text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-secondary)]"
                  >
                    Clear
                  </button>
                ) : null}
              </div>

              <div className="space-y-2 px-2.5 py-2">
                <div className="text-[10px] text-[var(--text-secondary)]">
                  {timeFilterLabel(value)}
                </div>

                <Slider
                  min={0}
                  max={TIME_STEPS.length - 1}
                  step={1}
                  minStepsBetweenThumbs={0}
                  value={sliderValue}
                  onValueChange={(values) => {
                    if (values.length === 2) {
                      onChange(sliderToTimeFilter([values[0], values[1]]));
                    }
                  }}
                />

                <div className="relative h-3 text-[10px] text-[var(--text-quaternary)]">
                  {TIME_TICKS.map((tick) => (
                    <span
                      key={tick.index}
                      className="absolute -translate-x-1/2"
                      style={{ left: `${(tick.index / (TIME_STEPS.length - 1)) * 100}%` }}
                    >
                      {tick.label}
                    </span>
                  ))}
                </div>
              </div>
            </div>,
            document.body,
          )
        : null}
    </div>
  );
}

function CostRangeFilter({
  bounds,
  costFilter,
  onChange,
}: {
  bounds: CostBounds | null;
  costFilter: CostFilter;
  onChange: (nextValue: CostFilter) => void;
}): JSX.Element {
  const [open, setOpen] = useState(false);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const triggerRef = useRef<HTMLButtonElement | null>(null);
  const [popoverPosition, setPopoverPosition] = useState<{ top: number; left: number; width: number } | null>(null);
  const sliderValue = useMemo(() => {
    if (!bounds) {
      return [0, 0];
    }

    return [costFilter.min ?? bounds.min, costFilter.max ?? bounds.max];
  }, [bounds, costFilter.max, costFilter.min]);

  const hasActiveFilter = costFilter.min != null || costFilter.max != null;
  const disabled = !bounds || Math.abs(bounds.max - bounds.min) <= COST_FILTER_EPSILON;
  const step = bounds && bounds.max - bounds.min > 0.5 ? 0.01 : 0.001;

  useEffect(() => {
    if (!open || !triggerRef.current) {
      return;
    }

    const updatePosition = () => {
      if (!triggerRef.current) {
        return;
      }
      const rect = triggerRef.current.getBoundingClientRect();
      setPopoverPosition({
        top: rect.bottom + 6,
        left: rect.left,
        width: Math.max(rect.width, 288),
      });
    };

    updatePosition();
    window.addEventListener("resize", updatePosition);
    window.addEventListener("scroll", updatePosition, true);
    return () => {
      window.removeEventListener("resize", updatePosition);
      window.removeEventListener("scroll", updatePosition, true);
    };
  }, [open]);

  useEffect(() => {
    if (!open) {
      return;
    }

    const handlePointerDown = (event: MouseEvent) => {
      if (!containerRef.current) {
        return;
      }
      if (event.target instanceof Node && !containerRef.current.contains(event.target)) {
        setOpen(false);
      }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setOpen(false);
      }
    };

    document.addEventListener("mousedown", handlePointerDown);
    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("mousedown", handlePointerDown);
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [open]);

  return (
    <div ref={containerRef} className="relative">
      <button
        ref={triggerRef}
        type="button"
        aria-expanded={open}
        aria-haspopup="dialog"
        disabled={!bounds}
        onClick={() => {
          if (!bounds) {
            return;
          }
          setOpen((current) => !current);
        }}
        className={cn(
          "flex max-w-[16rem] items-center justify-between gap-2 border outline-none transition-all duration-150",
          filterTriggerClass(hasActiveFilter),
          bounds
            ? "focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]"
            : "cursor-not-allowed opacity-60",
          !hasActiveFilter && bounds && "hover:border-[var(--border-strong)] focus-visible:border-[var(--accent)]",
        )}
      >
        <span className="truncate">{costTriggerLabel(bounds, costFilter)}</span>
        <ChevronDown
          className={cn(
            "size-3 transition-transform duration-150",
            hasActiveFilter ? "text-[var(--accent)]" : "text-[var(--text-tertiary)]",
            open && "rotate-180",
          )}
        />
      </button>

      {open && bounds && popoverPosition
        ? createPortal(
            <div
              className="fixed z-[120] rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
              style={{
                top: popoverPosition.top,
                left: popoverPosition.left,
                width: popoverPosition.width,
              }}
            >
              <div className="flex items-center justify-between px-2 py-1">
                <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-tertiary)]">Cost range</span>
                {hasActiveFilter ? (
                  <button
                    type="button"
                    onClick={() => onChange({ min: null, max: null })}
                    className="text-[10px] text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-secondary)]"
                  >
                    Clear
                  </button>
                ) : null}
              </div>

              <div className="space-y-2 px-2.5 py-2">
                <div className="font-mono text-[10px] text-[var(--text-secondary)]">
                  {hasActiveFilter ? costTriggerLabel(bounds, costFilter) : "Any cost"}
                </div>

                <Slider
                  min={bounds.min}
                  max={bounds.max}
                  step={step}
                  minStepsBetweenThumbs={0}
                  value={sliderValue}
                  disabled={disabled}
                  onValueChange={(values) => {
                    if (values.length !== 2) {
                      return;
                    }
                    onChange(buildCostFilterFromSlider(values, bounds));
                  }}
                />

                <div className="flex items-center justify-between font-mono text-[10px] text-[var(--text-quaternary)]">
                  <span>{formatCostValue(bounds.min)}</span>
                  <span>{formatCostValue(bounds.max)}</span>
                </div>
              </div>
            </div>,
            document.body,
          )
        : null}
    </div>
  );
}

export function ExplorerFilters(): JSX.Element | null {
  const { bootstrap, sourceFilter, timeFilter, modelFilter, sdkFilter, authorFilters, categoryFilters, costFilter, ratingFilter, secondaryRatingFilter, selectedRunId } =
    useViewer();
  const dispatch = useViewerDispatch();

  const sourceRecords = useMemo(() => {
    if (!bootstrap) return [];

    return uniqueRecords(
      sourceFilter === "dataset"
        ? bootstrap.dataset_entries.map((entry) => entry.record)
        : bootstrap.workbench_entries.map((entry) => entry.record),
    );
  }, [bootstrap, sourceFilter]);

  const availableModels = useMemo(() => {
    return Array.from(
      new Set(sourceRecords.map((record) => record.model_id).filter((value): value is string => Boolean(value))),
    ).sort((left, right) => left.localeCompare(right));
  }, [sourceRecords]);

  const availableSdks = useMemo(() => {
    return Array.from(
      new Set(sourceRecords.map((record) => record.sdk_package).filter((value): value is string => Boolean(value))),
    ).sort((left, right) => left.localeCompare(right));
  }, [sourceRecords]);

  const availableAuthors = useMemo(() => {
    return Array.from(
      new Set(sourceRecords.map((record) => record.author).filter((value): value is string => Boolean(value))),
    ).sort((left, right) => left.localeCompare(right));
  }, [sourceRecords]);

  const availableCostBounds = useMemo(
    () => getCostBounds(sourceRecords, selectedRunId),
    [selectedRunId, sourceRecords],
  );

  const costFilterActive = costFilter.min != null || costFilter.max != null;
  const timeFilterActive = timeFilter.oldest != null || timeFilter.newest != null;
  const ratingFilterActive = ratingFilter.length > 0;
  const secondaryRatingFilterActive = secondaryRatingFilter.length > 0;
  const modelFilterActive = modelFilter !== null;
  const sdkFilterActive = sdkFilter !== null;
  const authorFilterActive = sourceFilter === "dataset" && authorFilters.length > 0;
  const categoryFilterActive = sourceFilter === "dataset" && categoryFilters.length > 0;

  const availableCategories = useMemo(() => {
    if (!bootstrap) return [];

    const slugs = Array.from(
      new Set(
        bootstrap.dataset_entries
          .map((entry) => entry.category_slug.trim())
          .filter((value) => value.length > 0),
      ),
    );

    const supercategories: SupercategoryOption[] = bootstrap.supercategories ?? [];
    if (supercategories.length === 0) {
      return slugs
        .map((value) => ({ value, label: formatCategoryLabel(value) }))
        .sort((left, right) => left.label.localeCompare(right.label));
    }

    const catToSuper = new Map<string, string>();
    const superOrder = new Map<string, number>();
    for (let i = 0; i < supercategories.length; i++) {
      const sc = supercategories[i];
      superOrder.set(sc.slug, i);
      for (const cat of sc.category_slugs) {
        catToSuper.set(cat, sc.title);
      }
    }

    return slugs
      .map((value) => ({
        value,
        label: formatCategoryLabel(value),
        group: catToSuper.get(value),
      }))
      .sort((left, right) => {
        const leftGroup = left.group ?? "\uffff";
        const rightGroup = right.group ?? "\uffff";
        if (leftGroup !== rightGroup) {
          const leftSuper = supercategories.find((sc) => sc.title === leftGroup);
          const rightSuper = supercategories.find((sc) => sc.title === rightGroup);
          const leftIdx = leftSuper ? (superOrder.get(leftSuper.slug) ?? Infinity) : Infinity;
          const rightIdx = rightSuper ? (superOrder.get(rightSuper.slug) ?? Infinity) : Infinity;
          return leftIdx - rightIdx;
        }
        return left.label.localeCompare(right.label);
      });
  }, [bootstrap]);

  useEffect(() => {
    if (!availableCostBounds) {
      if (costFilter.min != null || costFilter.max != null) {
        dispatch({ type: "SET_COST_FILTER", payload: { min: null, max: null } });
      }
      return;
    }

    const normalizedCostFilter = normalizeCostFilterForBounds(costFilter, availableCostBounds);
    if (!sameCostFilter(costFilter, normalizedCostFilter)) {
      dispatch({ type: "SET_COST_FILTER", payload: normalizedCostFilter });
    }
  }, [availableCostBounds, costFilter, dispatch]);

  if (!bootstrap) return null;

  const filtersActive =
    timeFilterActive ||
    costFilterActive ||
    ratingFilterActive ||
    secondaryRatingFilterActive ||
    modelFilterActive ||
    sdkFilterActive ||
    authorFilterActive ||
    categoryFilterActive;

  return (
    <div className="space-y-2">
      <div className="flex items-center justify-between">
        <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-tertiary)]">Filters</span>
        {filtersActive ? (
          <Button
            type="button"
            variant="ghost"
            size="sm"
            onClick={() => {
              dispatch({ type: "SET_TIME_FILTER", payload: { oldest: null, newest: null } });
              dispatch({ type: "SET_COST_FILTER", payload: { min: null, max: null } });
              dispatch({ type: "SET_RATING_FILTER", payload: [] });
              dispatch({ type: "SET_SECONDARY_RATING_FILTER", payload: [] });
              dispatch({ type: "SET_MODEL_FILTER", payload: null });
              dispatch({ type: "SET_SDK_FILTER", payload: null });
              dispatch({ type: "SET_AUTHOR_FILTERS", payload: [] });
              dispatch({ type: "SET_CATEGORY_FILTERS", payload: [] });
            }}
            className="h-5 px-1.5 text-[10px] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
          >
            Clear
          </Button>
        ) : null}
      </div>

      <div className="flex flex-wrap gap-1.5">
        <TimeRangeFilter
          value={timeFilter}
          onChange={(nextValue) => dispatch({ type: "SET_TIME_FILTER", payload: nextValue })}
        />

        <CostRangeFilter
          bounds={availableCostBounds}
          costFilter={costFilter}
          onChange={(nextValue) => dispatch({ type: "SET_COST_FILTER", payload: nextValue })}
        />

        <MultiSelectFilter
          options={ratingOptions}
          selectedValues={ratingFilter}
          onChange={(nextValues) => dispatch({ type: "SET_RATING_FILTER", payload: nextValues as RatingFilter })}
          title="Ratings"
          searchPlaceholder="Search ratings…"
          noMatchLabel="No ratings match"
          triggerLabel={ratingTriggerLabel}
        />

        <MultiSelectFilter
          options={ratingOptions}
          selectedValues={secondaryRatingFilter}
          onChange={(nextValues) => dispatch({ type: "SET_SECONDARY_RATING_FILTER", payload: nextValues as RatingFilter })}
          title="Secondary Ratings"
          searchPlaceholder="Search ratings…"
          noMatchLabel="No ratings match"
          triggerLabel={(options, selectedValues) =>
            selectedValues.length === 0
              ? "Any secondary rating"
              : ratingTriggerLabel(options, selectedValues).replace("ratings", "secondary ratings")
          }
        />

        <Select
          value={modelFilter ?? "all"}
          onValueChange={(value) =>
            dispatch({
              type: "SET_MODEL_FILTER",
              payload: value === "all" ? null : value,
            })
          }
        >
          <SelectTrigger size="sm" className={filterTriggerClass(modelFilterActive, "max-w-full")}>
            <SelectValue placeholder="All models" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="all">All models</SelectItem>
            {availableModels.map((model) => (
              <SelectItem key={model} value={model}>
                <span className="truncate font-mono text-[11px]">{model}</span>
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        {availableSdks.length > 1 ? (
          <Select
            value={sdkFilter ?? "all"}
            onValueChange={(value) =>
              dispatch({
                type: "SET_SDK_FILTER",
                payload: value === "all" ? null : value,
              })
            }
          >
            <SelectTrigger size="sm" className={filterTriggerClass(sdkFilterActive)}>
              <SelectValue placeholder="All SDKs" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All SDKs</SelectItem>
              {availableSdks.map((sdk) => (
                <SelectItem key={sdk} value={sdk}>
                  <span className="truncate font-mono text-[11px]">{sdk}</span>
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        ) : null}

        {sourceFilter === "dataset" && availableAuthors.length > 0 ? (
          <MultiSelectFilter
            options={availableAuthors.map((author) => ({ value: author, label: author }))}
            selectedValues={authorFilters}
            onChange={(nextValues) => dispatch({ type: "SET_AUTHOR_FILTERS", payload: nextValues })}
            title="Authors"
            searchPlaceholder="Search authors…"
            noMatchLabel="No authors match"
            triggerLabel={authorTriggerLabel}
          />
        ) : null}

        {sourceFilter === "dataset" && availableCategories.length > 0 ? (
          <MultiSelectFilter
            options={availableCategories}
            selectedValues={categoryFilters}
            onChange={(nextValues) => dispatch({ type: "SET_CATEGORY_FILTERS", payload: nextValues })}
            title="Categories"
            searchPlaceholder="Search categories…"
            noMatchLabel="No categories match"
            triggerLabel={categoryTriggerLabel}
          />
        ) : null}
      </div>
    </div>
  );
}
