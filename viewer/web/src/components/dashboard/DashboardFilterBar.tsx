import type { JSX } from "react";

import type { CostFilter, TimeFilter } from "@/lib/types";
import { cn } from "@/lib/utils";
import {
  DashboardAuthorFilter,
  DashboardCategoryFilter,
  pillClass,
  STAR_SLIDER_MAX,
  STAR_SLIDER_MIN,
  type CostBounds,
  DashboardCostFilter,
  DashboardStarsFilter,
  DashboardTimeFilter,
} from "@/components/dashboard/dashboard-filters";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

type DashboardFilterBarProps = {
  timeFilter: TimeFilter;
  onTimeFilterChange: (value: TimeFilter) => void;
  starsFilter: [number, number];
  onStarsFilterChange: (value: [number, number]) => void;
  costFilter: CostFilter;
  onCostFilterChange: (value: CostFilter) => void;
  sdkFilter: string | null;
  onSdkFilterChange: (value: string | null) => void;
  authorFilters: string[];
  onAuthorFiltersChange: (value: string[]) => void;
  categoryFilters: string[];
  onCategoryFiltersChange: (value: string[]) => void;
  availableSdks: string[];
  availableAuthors: string[];
  availableCategories: string[];
  costBounds: CostBounds | null;
  recordCount: number;
  categoryCount: number;
  loading: boolean;
};

export function DashboardFilterBar({
  timeFilter,
  onTimeFilterChange,
  starsFilter,
  onStarsFilterChange,
  costFilter,
  onCostFilterChange,
  sdkFilter,
  onSdkFilterChange,
  authorFilters,
  onAuthorFiltersChange,
  categoryFilters,
  onCategoryFiltersChange,
  availableSdks,
  availableAuthors,
  availableCategories,
  costBounds,
  recordCount,
  categoryCount,
  loading,
}: DashboardFilterBarProps): JSX.Element {
  const timeFilterActive = timeFilter.oldest != null || timeFilter.newest != null;
  const starsFilterActive =
    starsFilter[0] !== STAR_SLIDER_MIN || starsFilter[1] !== STAR_SLIDER_MAX;
  const costFilterActive = costFilter.min != null || costFilter.max != null;
  const sdkFilterActive = sdkFilter != null;
  const authorFilterActive = authorFilters.length > 0;
  const categoryFilterActive = categoryFilters.length > 0;
  const anyFilterActive =
    timeFilterActive ||
    starsFilterActive ||
    costFilterActive ||
    sdkFilterActive ||
    authorFilterActive ||
    categoryFilterActive;

  return (
    <section>
      <div className="flex items-center justify-between gap-3">
        <div className="flex flex-wrap items-center gap-1.5">
          {availableCategories.length > 0 ? (
            <DashboardCategoryFilter
              options={availableCategories}
              value={categoryFilters}
              onChange={onCategoryFiltersChange}
            />
          ) : null}
          {availableAuthors.length > 0 ? (
            <DashboardAuthorFilter
              options={availableAuthors}
              value={authorFilters}
              onChange={onAuthorFiltersChange}
            />
          ) : null}
          <DashboardStarsFilter value={starsFilter} onChange={onStarsFilterChange} />
          <DashboardTimeFilter value={timeFilter} onChange={onTimeFilterChange} />
          <DashboardCostFilter
            bounds={costBounds}
            value={costFilter}
            onChange={onCostFilterChange}
          />

          {availableSdks.length > 1 ? (
            <Select
              value={sdkFilter ?? "all"}
              onValueChange={(value) => onSdkFilterChange(value === "all" ? null : value)}
            >
              <SelectTrigger size="sm" className={cn(pillClass(sdkFilterActive), "h-6")}>
                <SelectValue placeholder="All SDKs" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All SDKs</SelectItem>
                {availableSdks.map((sdk) => (
                  <SelectItem key={sdk} value={sdk}>
                    <span className="truncate font-mono text-[10px]">{sdk}</span>
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          ) : null}

          {anyFilterActive ? (
            <button
              type="button"
              onClick={() => {
                onTimeFilterChange({ oldest: null, newest: null });
                onStarsFilterChange([STAR_SLIDER_MIN, STAR_SLIDER_MAX]);
                onCostFilterChange({ min: null, max: null });
                onSdkFilterChange(null);
                onAuthorFiltersChange([]);
                onCategoryFiltersChange([]);
              }}
              className="h-6 px-2 text-[10px] text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-secondary)]"
            >
              Clear all
            </button>
          ) : null}
        </div>

        <div className="flex shrink-0 items-center gap-3">
          {loading ? (
            <span
              aria-live="polite"
              className="inline-flex items-center gap-1.5 rounded-full border border-[var(--accent)]/30 bg-[var(--accent-soft)] px-2 py-1 text-[10px] text-[var(--accent)]"
            >
              <span className="size-1.5 animate-pulse rounded-full bg-current" />
              Applying filters…
            </span>
          ) : null}
          <span className="text-[10px] tabular-nums text-[var(--text-quaternary)]">
            {recordCount} records · {categoryCount} categories
          </span>
        </div>
      </div>
      {loading ? (
        <p className="mt-2 text-[10px] text-[var(--text-quaternary)]">
          Filtered views can take a few seconds to recompute, especially tighter rating ranges.
        </p>
      ) : null}
    </section>
  );
}
