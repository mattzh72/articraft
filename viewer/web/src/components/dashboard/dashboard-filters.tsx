import { useEffect, useMemo, useRef, useState, type JSX } from "react";
import { createPortal } from "react-dom";
import { Check, ChevronDown, Search } from "lucide-react";

import { cn, formatCategoryLabel } from "@/lib/utils";
import type { TimeFilter, TimeFilterPoint } from "@/lib/types";
import { Slider } from "@/components/ui/slider";

/* ---------------------------------------------------------------------------
 * Shared styling
 * --------------------------------------------------------------------------- */

const FILTER_PILL = cn(
  "h-6 w-auto min-w-0 rounded-full px-2 text-[10px] border outline-none transition-all duration-150",
);

export function pillClass(active: boolean): string {
  return cn(
    FILTER_PILL,
    active
      ? "border-[var(--accent)] bg-[var(--accent-soft)] text-[var(--accent)] hover:border-[var(--accent)] hover:bg-[var(--accent-hover)] [&_svg]:text-[var(--accent)]"
      : "border-[var(--border-default)] bg-[var(--surface-1)] text-[var(--text-secondary)] hover:border-[var(--border-strong)]",
  );
}

/* ---------------------------------------------------------------------------
 * Time filter constants & helpers
 * --------------------------------------------------------------------------- */

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

export const TIME_DURATIONS: Record<string, number> = {
  "1h": 1 * 60 * 60 * 1000,
  "6h": 6 * 60 * 60 * 1000,
  "12h": 12 * 60 * 60 * 1000,
  "24h": 24 * 60 * 60 * 1000,
  "3d": 3 * 24 * 60 * 60 * 1000,
  "7d": 7 * 24 * 60 * 60 * 1000,
  "14d": 14 * 24 * 60 * 60 * 1000,
  "30d": 30 * 24 * 60 * 60 * 1000,
  "60d": 60 * 24 * 60 * 60 * 1000,
  "90d": 90 * 24 * 60 * 60 * 1000,
  "180d": 180 * 24 * 60 * 60 * 1000,
  "1y": 365 * 24 * 60 * 60 * 1000,
};

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

function timeLabel(filter: TimeFilter): string {
  if (!filter.oldest && !filter.newest) return "Any time";
  const oldestLabel = filter.oldest ? (TIME_STEPS.find((s) => s.value === filter.oldest)?.label ?? filter.oldest) : null;
  const newestLabel = filter.newest ? (TIME_STEPS.find((s) => s.value === filter.newest)?.label ?? filter.newest) : null;
  if (oldestLabel && newestLabel) return `${newestLabel} – ${oldestLabel} ago`;
  if (oldestLabel) return `Last ${oldestLabel}`;
  if (newestLabel) return `Older than ${newestLabel}`;
  return "Any time";
}

/* ---------------------------------------------------------------------------
 * Stars / Cost constants
 * --------------------------------------------------------------------------- */

export const STAR_SLIDER_MIN = 0;
export const STAR_SLIDER_MAX = 5;

const COST_EPSILON = 0.0005;
const costFmt = new Intl.NumberFormat("en-US", { style: "currency", currency: "USD", minimumFractionDigits: 2, maximumFractionDigits: 3 });

export type CostBounds = { min: number; max: number };

type DashboardMultiSelectFilterProps = {
  title: string;
  emptyLabel: string;
  summaryNoun: string;
  options: string[];
  value: string[];
  onChange: (value: string[]) => void;
  formatOptionLabel?: (value: string) => string;
  searchPlaceholder: string;
  noResultsLabel: string;
};

function buildMultiSelectLabel({
  value,
  options,
  emptyLabel,
  summaryNoun,
  formatOptionLabel,
}: Omit<DashboardMultiSelectFilterProps, "title" | "onChange" | "searchPlaceholder" | "noResultsLabel">): string {
  if (value.length === 0) {
    return emptyLabel;
  }
  if (value.length === 1) {
    const selected = options.find((option) => option === value[0]) ?? value[0];
    return formatOptionLabel ? formatOptionLabel(selected) : selected;
  }
  return `${value.length} ${summaryNoun}`;
}

/* ---------------------------------------------------------------------------
 * SliderPopover
 * --------------------------------------------------------------------------- */

export function SliderPopover({
  open,
  triggerRef,
  containerRef,
  onClose,
  title,
  hasActive,
  onClear,
  minWidth,
  children,
}: {
  open: boolean;
  triggerRef: React.RefObject<HTMLButtonElement | null>;
  containerRef: React.RefObject<HTMLDivElement | null>;
  onClose: () => void;
  title: string;
  hasActive: boolean;
  onClear: () => void;
  minWidth?: number;
  children: React.ReactNode;
}): JSX.Element | null {
  const [pos, setPos] = useState<{ top: number; left: number; width: number } | null>(null);
  const popoverRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (!open || !triggerRef.current) return;
    const update = () => {
      if (!triggerRef.current) return;
      const r = triggerRef.current.getBoundingClientRect();
      setPos({ top: r.bottom + 6, left: r.left, width: Math.max(r.width, minWidth ?? 248) });
    };
    update();
    window.addEventListener("resize", update);
    window.addEventListener("scroll", update, true);
    return () => {
      window.removeEventListener("resize", update);
      window.removeEventListener("scroll", update, true);
    };
  }, [open, triggerRef, minWidth]);

  useEffect(() => {
    if (!open) return;
    const onMouse = (e: MouseEvent) => {
      if (!(e.target instanceof Node)) return;
      if (containerRef.current?.contains(e.target)) return;
      if (popoverRef.current?.contains(e.target)) return;
      onClose();
    };
    const onKey = (e: KeyboardEvent) => { if (e.key === "Escape") onClose(); };
    document.addEventListener("mousedown", onMouse);
    document.addEventListener("keydown", onKey);
    return () => { document.removeEventListener("mousedown", onMouse); document.removeEventListener("keydown", onKey); };
  }, [open, containerRef, onClose]);

  if (!open || !pos) return null;

  return createPortal(
    <div
      ref={popoverRef}
      className="fixed z-[120] rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-1 shadow-[0_4px_16px_rgba(0,0,0,0.08),0_0_0_1px_rgba(0,0,0,0.02)]"
      style={{ top: pos.top, left: pos.left, width: pos.width }}
    >
      <div className="flex items-center justify-between px-2 py-1">
        <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-tertiary)]">{title}</span>
        {hasActive ? (
          <button type="button" onClick={onClear} className="text-[10px] text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-secondary)]">Clear</button>
        ) : null}
      </div>
      <div className="space-y-2 px-2.5 py-2">{children}</div>
    </div>,
    document.body,
  );
}

/* ---------------------------------------------------------------------------
 * DashboardTimeFilter
 * --------------------------------------------------------------------------- */

export function DashboardTimeFilter({
  value,
  onChange,
}: {
  value: TimeFilter;
  onChange: (v: TimeFilter) => void;
}): JSX.Element {
  const [open, setOpen] = useState(false);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const triggerRef = useRef<HTMLButtonElement | null>(null);
  const hasActive = value.oldest != null || value.newest != null;
  const sliderValue = timeFilterToSlider(value);

  return (
    <div ref={containerRef} className="relative">
      <button ref={triggerRef} type="button" onClick={() => setOpen((c) => !c)} className={cn("flex items-center gap-1.5", pillClass(hasActive))}>
        <span className="truncate">{timeLabel(value)}</span>
        <ChevronDown className={cn("size-3 transition-transform duration-150", open && "rotate-180")} />
      </button>
      <SliderPopover open={open} triggerRef={triggerRef} containerRef={containerRef} onClose={() => setOpen(false)} title="Time range" hasActive={hasActive} onClear={() => onChange({ oldest: null, newest: null })} minWidth={288}>
        <div className="text-[10px] text-[var(--text-secondary)]">{timeLabel(value)}</div>
        <Slider min={0} max={TIME_STEPS.length - 1} step={1} minStepsBetweenThumbs={0} value={sliderValue} onValueChange={(v) => { if (v.length === 2) onChange(sliderToTimeFilter([v[0], v[1]])); }} />
        <div className="relative h-3 text-[10px] text-[var(--text-quaternary)]">
          {TIME_TICKS.map((tick) => (
            <span key={tick.index} className="absolute -translate-x-1/2" style={{ left: `${(tick.index / (TIME_STEPS.length - 1)) * 100}%` }}>{tick.label}</span>
          ))}
        </div>
      </SliderPopover>
    </div>
  );
}

/* ---------------------------------------------------------------------------
 * DashboardStarsFilter
 * --------------------------------------------------------------------------- */

export function DashboardStarsFilter({
  value,
  onChange,
}: {
  value: [number, number];
  onChange: (v: [number, number]) => void;
}): JSX.Element {
  const [open, setOpen] = useState(false);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const triggerRef = useRef<HTMLButtonElement | null>(null);
  const hasActive = value[0] !== STAR_SLIDER_MIN || value[1] !== STAR_SLIDER_MAX;

  const label = hasActive
    ? value[0] === value[1]
      ? `${value[0]} rating`
      : `${value[0]} – ${value[1]} rating`
    : "Any average rating";

  return (
    <div ref={containerRef} className="relative">
      <button ref={triggerRef} type="button" onClick={() => setOpen((c) => !c)} className={cn("flex items-center gap-1.5", pillClass(hasActive))}>
        <span className="truncate">{label}</span>
        <ChevronDown className={cn("size-3 transition-transform duration-150", open && "rotate-180")} />
      </button>
      <SliderPopover open={open} triggerRef={triggerRef} containerRef={containerRef} onClose={() => setOpen(false)} title="Average rating" hasActive={hasActive} onClear={() => onChange([STAR_SLIDER_MIN, STAR_SLIDER_MAX])}>
        <div className="text-[10px] text-[var(--text-secondary)]">{label}</div>
        <Slider min={STAR_SLIDER_MIN} max={STAR_SLIDER_MAX} step={0.5} value={value} onValueChange={(v) => { if (v.length === 2) onChange([v[0], v[1]]); }} />
        <div className="flex items-center justify-between font-mono text-[10px] text-[var(--text-quaternary)]">
          <span>{STAR_SLIDER_MIN}</span>
          <span>{STAR_SLIDER_MAX}</span>
        </div>
      </SliderPopover>
    </div>
  );
}

function DashboardMultiSelectFilter({
  title,
  emptyLabel,
  summaryNoun,
  options,
  value,
  onChange,
  formatOptionLabel,
  searchPlaceholder,
  noResultsLabel,
}: DashboardMultiSelectFilterProps): JSX.Element {
  const [open, setOpen] = useState(false);
  const [search, setSearch] = useState("");
  const containerRef = useRef<HTMLDivElement | null>(null);
  const triggerRef = useRef<HTMLButtonElement | null>(null);
  const hasActive = value.length > 0;
  const selectedSet = useMemo(() => new Set(value), [value]);
  const label = buildMultiSelectLabel({
    value,
    options,
    emptyLabel,
    summaryNoun,
    formatOptionLabel,
  });

  const filteredOptions = useMemo(() => {
    const query = search.trim().toLowerCase();
    if (!query) {
      return options;
    }
    return options.filter((option) => {
      const optionLabel = formatOptionLabel ? formatOptionLabel(option) : option;
      return (
        option.toLowerCase().includes(query) ||
        optionLabel.toLowerCase().includes(query)
      );
    });
  }, [formatOptionLabel, options, search]);

  function toggleOption(option: string): void {
    if (selectedSet.has(option)) {
      onChange(value.filter((item) => item !== option));
      return;
    }
    onChange([...value, option]);
  }

  return (
    <div ref={containerRef} className="relative">
      <button
        ref={triggerRef}
        type="button"
        onClick={() => setOpen((current) => !current)}
        className={cn("flex items-center gap-1.5", pillClass(hasActive))}
      >
        <span className="truncate">{label}</span>
        <ChevronDown className={cn("size-3 transition-transform duration-150", open && "rotate-180")} />
      </button>
      <SliderPopover
        open={open}
        triggerRef={triggerRef}
        containerRef={containerRef}
        onClose={() => setOpen(false)}
        title={title}
        hasActive={hasActive}
        onClear={() => onChange([])}
        minWidth={300}
      >
        <label className="flex items-center gap-2 rounded-md border border-[var(--border-default)] bg-[var(--surface-1)] px-2 py-1">
          <Search className="size-3 shrink-0 text-[var(--text-quaternary)]" />
          <input
            type="text"
            value={search}
            onChange={(event) => setSearch(event.target.value)}
            placeholder={searchPlaceholder}
            className="w-full bg-transparent text-[10px] text-[var(--text-primary)] placeholder:text-[var(--text-quaternary)] focus:outline-none"
          />
        </label>
        <div className="max-h-56 overflow-y-auto pr-1">
          {filteredOptions.length === 0 ? (
            <div className="py-1 text-[10px] text-[var(--text-quaternary)]">{noResultsLabel}</div>
          ) : (
            <div className="space-y-1">
              {filteredOptions.map((option) => {
                const checked = selectedSet.has(option);
                const optionLabel = formatOptionLabel ? formatOptionLabel(option) : option;
                return (
                  <button
                    key={option}
                    type="button"
                    onClick={() => toggleOption(option)}
                    className={cn(
                      "flex w-full items-center justify-between gap-3 rounded-md px-2 py-1.5 text-left text-[10px] transition-colors",
                      checked
                        ? "bg-[var(--accent-soft)] text-[var(--accent)]"
                        : "text-[var(--text-secondary)] hover:bg-[var(--surface-1)]",
                    )}
                  >
                    <span className="truncate">{optionLabel}</span>
                    <span
                      className={cn(
                        "flex size-3.5 shrink-0 items-center justify-center rounded-sm border",
                        checked
                          ? "border-[var(--accent)] bg-[var(--accent)] text-white"
                          : "border-[var(--border-default)] text-transparent",
                      )}
                    >
                      <Check className="size-2.5" />
                    </span>
                  </button>
                );
              })}
            </div>
          )}
        </div>
      </SliderPopover>
    </div>
  );
}

export function DashboardCategoryFilter({
  options,
  value,
  onChange,
}: {
  options: string[];
  value: string[];
  onChange: (value: string[]) => void;
}): JSX.Element {
  return (
    <DashboardMultiSelectFilter
      title="Categories"
      emptyLabel="All categories"
      summaryNoun="categories"
      options={options}
      value={value}
      onChange={onChange}
      formatOptionLabel={formatCategoryLabel}
      searchPlaceholder="Filter categories..."
      noResultsLabel="No matching categories"
    />
  );
}

export function DashboardAuthorFilter({
  options,
  value,
  onChange,
}: {
  options: string[];
  value: string[];
  onChange: (value: string[]) => void;
}): JSX.Element {
  return (
    <DashboardMultiSelectFilter
      title="Authors"
      emptyLabel="All authors"
      summaryNoun="authors"
      options={options}
      value={value}
      onChange={onChange}
      searchPlaceholder="Filter authors..."
      noResultsLabel="No matching authors"
    />
  );
}

/* ---------------------------------------------------------------------------
 * DashboardCostFilter
 * --------------------------------------------------------------------------- */

export function DashboardCostFilter({
  bounds,
  value,
  onChange,
}: {
  bounds: CostBounds | null;
  value: { min: number | null; max: number | null };
  onChange: (v: { min: number | null; max: number | null }) => void;
}): JSX.Element {
  const [open, setOpen] = useState(false);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const triggerRef = useRef<HTMLButtonElement | null>(null);
  const hasActive = value.min != null || value.max != null;
  const disabled = !bounds || Math.abs(bounds.max - bounds.min) <= COST_EPSILON;
  const sliderVal = bounds ? [value.min ?? bounds.min, value.max ?? bounds.max] : [0, 0];
  const step = bounds && bounds.max - bounds.min > 0.5 ? 0.01 : 0.001;

  const label = !bounds
    ? "No cost data"
    : hasActive
      ? `${costFmt.format(value.min ?? bounds.min)} – ${costFmt.format(value.max ?? bounds.max)}`
      : "Any cost";

  return (
    <div ref={containerRef} className="relative">
      <button ref={triggerRef} type="button" disabled={!bounds} onClick={() => { if (bounds) setOpen((c) => !c); }} className={cn("flex items-center gap-1.5", pillClass(hasActive), !bounds && "cursor-not-allowed opacity-60")}>
        <span className="truncate">{label}</span>
        <ChevronDown className={cn("size-3 transition-transform duration-150", open && "rotate-180")} />
      </button>
      <SliderPopover open={open} triggerRef={triggerRef} containerRef={containerRef} onClose={() => setOpen(false)} title="Cost range" hasActive={hasActive} onClear={() => onChange({ min: null, max: null })} minWidth={288}>
        <div className="font-mono text-[10px] text-[var(--text-secondary)]">{hasActive ? label : "Any cost"}</div>
        {bounds ? (
          <>
            <Slider
              min={bounds.min}
              max={bounds.max}
              step={step}
              minStepsBetweenThumbs={0}
              value={sliderVal}
              disabled={disabled}
              onValueChange={(v) => {
                if (v.length !== 2 || !bounds) return;
                const [lo, hi] = v;
                const nextMin = lo != null && lo <= bounds.min + COST_EPSILON ? null : lo ?? null;
                const nextMax = hi != null && hi >= bounds.max - COST_EPSILON ? null : hi ?? null;
                onChange({ min: nextMin, max: nextMax });
              }}
            />
            <div className="flex items-center justify-between font-mono text-[10px] text-[var(--text-quaternary)]">
              <span>{costFmt.format(bounds.min)}</span>
              <span>{costFmt.format(bounds.max)}</span>
            </div>
          </>
        ) : null}
      </SliderPopover>
    </div>
  );
}
