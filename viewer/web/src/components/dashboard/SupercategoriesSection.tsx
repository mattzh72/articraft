import { useMemo, useState, type JSX } from "react";
import {
  Cell,
  Label,
  Pie,
  PieChart,
  Sector,
  Tooltip,
  type PieSectorShapeProps,
} from "recharts";

import { formatCost } from "@/lib/dashboard-stats";
import type { DatasetEntry, SupercategoryOption, TimeFilter } from "@/lib/types";
import { cn } from "@/lib/utils";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  ChartContainer,
  type ChartConfig,
} from "@/components/ui/chart";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import {
  pillClass,
  TIME_DURATIONS,
  STAR_SLIDER_MIN,
  STAR_SLIDER_MAX,
  type CostBounds,
  DashboardTimeFilter,
  DashboardStarsFilter,
  DashboardCostFilter,
} from "@/components/dashboard/dashboard-filters";

type CategoryStats = {
  count: number;
  sdk_package: string | null;
  average_rating: number | null;
  average_cost_usd: number | null;
};

type SupercategoriesSectionProps = {
  categoryStats: Record<string, CategoryStats>;
  supercategories: SupercategoryOption[];
  datasetEntries?: DatasetEntry[];
};

const HEAD_CLASSES = "h-8 px-3 text-[10px] font-medium uppercase tracking-[0.06em] text-[var(--text-quaternary)]";

const PALETTE = [
  "#0ea5e9",
  "#f59e0b",
  "#10b981",
  "#f97316",
  "#6366f1",
  "#ec4899",
  "#06b6d4",
  "#ef4444",
  "#84cc16",
  "#8b5cf6",
  "#14b8a6",
  "#f43f5e",
  "#22c55e",
  "#2563eb",
];

/* ---------------------------------------------------------------------------
 * Data types & computation
 * --------------------------------------------------------------------------- */

type SupercategoryRow = {
  slug: string;
  title: string;
  color: string;
  categoryCount: number;
  recordCount: number;
  avgRating: number | null;
  avgCost: number | null;
};

type DisplayRow = SupercategoryRow & {
  share: number;
};

function formatRecordCount(value: number | null | undefined): string {
  return typeof value === "number" && Number.isFinite(value) ? value.toLocaleString() : "--";
}

function isDisplayRow(value: unknown): value is DisplayRow {
  if (typeof value !== "object" || value == null) return false;
  const row = value as Partial<DisplayRow>;
  return (
    typeof row.slug === "string"
    && typeof row.title === "string"
    && typeof row.color === "string"
    && typeof row.categoryCount === "number"
    && typeof row.recordCount === "number"
    && typeof row.share === "number"
  );
}

function computeRows(
  supercategories: SupercategoryOption[],
  categoryStats: Record<string, CategoryStats>,
): SupercategoryRow[] {
  return supercategories.map((sc, i) => {
    let recordCount = 0;
    let ratingSum = 0;
    let ratingWeights = 0;
    let costSum = 0;
    let costWeights = 0;
    let categoryCount = 0;

    for (const slug of sc.category_slugs) {
      const stats = categoryStats[slug];
      if (!stats) continue;
      categoryCount++;
      recordCount += stats.count;
      if (stats.average_rating != null) {
        ratingSum += stats.average_rating * stats.count;
        ratingWeights += stats.count;
      }
      if (stats.average_cost_usd != null) {
        costSum += stats.average_cost_usd * stats.count;
        costWeights += stats.count;
      }
    }

    return {
      slug: sc.slug,
      title: sc.title,
      color: PALETTE[i % PALETTE.length],
      categoryCount,
      recordCount,
      avgRating: ratingWeights > 0 ? Math.round((ratingSum / ratingWeights) * 10) / 10 : null,
      avgCost: costWeights > 0 ? Math.round((costSum / costWeights) * 10000) / 10000 : null,
    };
  });
}

function formatRating(value: number | null): string {
  return value != null ? `${value.toFixed(1)}` : "--";
}

function formatShare(share: number): string {
  const percentage = share * 100;
  return `${percentage >= 10 ? percentage.toFixed(0) : percentage.toFixed(1)}%`;
}

function hexToRgba(hex: string, alpha: number): string {
  const normalized = hex.replace("#", "");
  const expanded = normalized.length === 3
    ? normalized
      .split("")
      .map((char) => `${char}${char}`)
      .join("")
    : normalized;

  if (expanded.length !== 6) {
    return `rgba(17, 17, 17, ${alpha})`;
  }

  const value = Number.parseInt(expanded, 16);
  const r = (value >> 16) & 255;
  const g = (value >> 8) & 255;
  const b = value & 255;
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

function renderSliceShape(
  props: PieSectorShapeProps,
  activeSlug: string | null,
): JSX.Element {
  const { outerRadius, payload } = props;
  const isActive = activeSlug != null && payload?.slug === activeSlug;
  const isDimmed = activeSlug != null && !isActive;

  return (
    <Sector
      {...props}
      outerRadius={typeof outerRadius === "number" && isActive ? outerRadius + 4 : outerRadius}
      cornerRadius={5}
      opacity={isDimmed ? 0.34 : 1}
      stroke="var(--surface-1)"
      strokeWidth={isActive ? 2 : 1}
    />
  );
}

function CustomTooltip({
  active,
  payload,
}: {
  active?: boolean;
  payload?: Array<{ payload: DisplayRow }>;
}): JSX.Element | null {
  if (!active || !payload || payload.length === 0) return null;
  const row = payload
    .map((entry) => entry?.payload)
    .find((entry): entry is DisplayRow => isDisplayRow(entry));
  if (!row) return null;

  return (
    <div className="rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] px-3 py-2 shadow-md">
      <div className="flex items-center gap-1.5 text-[11px] font-medium text-[var(--text-primary)]">
        <span
          className="inline-block size-2 rounded-full"
          style={{ backgroundColor: row.color }}
        />
        <span>{row.title}</span>
        <span className="ml-1 font-mono text-[10px] text-[var(--text-quaternary)]">
          {formatShare(row.share)}
        </span>
      </div>
      <div className="mt-1 space-y-0.5 text-[10px] text-[var(--text-tertiary)]">
        <div>
          {formatRecordCount(row.recordCount)} records across {row.categoryCount} categories
        </div>
        <div>
          Avg stars: {formatRating(row.avgRating)} · Avg cost: {formatCost(row.avgCost)}
        </div>
      </div>
    </div>
  );
}

export function SupercategoriesSection({
  categoryStats,
  supercategories,
  datasetEntries,
}: SupercategoriesSectionProps): JSX.Element | null {
  const [activeSlug, setActiveSlug] = useState<string | null>(null);

  // --- local filter state ---
  const [timeFilter, setTimeFilter] = useState<TimeFilter>({ oldest: null, newest: null });
  const [starsFilter, setStarsFilter] = useState<[number, number]>([STAR_SLIDER_MIN, STAR_SLIDER_MAX]);
  const [costFilter, setCostFilter] = useState<{ min: number | null; max: number | null }>({ min: null, max: null });
  const [sdkFilter, setSdkFilter] = useState<string | null>(null);

  const timeFilterActive = timeFilter.oldest != null || timeFilter.newest != null;
  const starsFilterActive = starsFilter[0] !== STAR_SLIDER_MIN || starsFilter[1] !== STAR_SLIDER_MAX;
  const costFilterActive = costFilter.min != null || costFilter.max != null;
  const sdkFilterActive = sdkFilter !== null;
  const anyFilterActive = timeFilterActive || starsFilterActive || costFilterActive || sdkFilterActive;

  // Compute which categories pass the time filter using dataset_entries
  const timeSlugs = useMemo<Set<string> | null>(() => {
    if (!timeFilterActive || !datasetEntries) return null;
    const now = Date.now();
    const maxAge = timeFilter.oldest ? TIME_DURATIONS[timeFilter.oldest] : null;
    const minAge = timeFilter.newest ? TIME_DURATIONS[timeFilter.newest] : null;
    const slugs = new Set<string>();
    for (const entry of datasetEntries) {
      const ts = entry.record?.created_at ? new Date(entry.record.created_at).getTime() : 0;
      if (!ts) continue;
      const age = now - ts;
      if (maxAge != null && age > maxAge) continue;
      if (minAge != null && age < minAge) continue;
      slugs.add(entry.category_slug);
    }
    return slugs;
  }, [timeFilterActive, timeFilter, datasetEntries]);

  // Available SDK values from category stats
  const availableSdks = useMemo(() => {
    return Array.from(
      new Set(
        Object.values(categoryStats)
          .map((s) => s.sdk_package)
          .filter((v): v is string => Boolean(v)),
      ),
    ).sort((a, b) => a.localeCompare(b));
  }, [categoryStats]);

  // Cost bounds across all categories
  const categoryCostBounds = useMemo<CostBounds | null>(() => {
    const costs = Object.values(categoryStats)
      .map((s) => s.average_cost_usd)
      .filter((v): v is number => v != null && Number.isFinite(v));
    if (costs.length === 0) return null;
    return { min: Math.min(...costs), max: Math.max(...costs) };
  }, [categoryStats]);

  // Apply filters to category stats
  const filteredCategoryStats = useMemo(() => {
    const result: Record<string, CategoryStats> = {};
    for (const [slug, stats] of Object.entries(categoryStats)) {
      if (timeSlugs && !timeSlugs.has(slug)) continue;
      if (sdkFilter && stats.sdk_package !== sdkFilter) continue;
      if (starsFilterActive) {
        if (stats.average_rating == null) continue;
        if (stats.average_rating < starsFilter[0] || stats.average_rating > starsFilter[1]) continue;
      }
      if (costFilterActive) {
        if (stats.average_cost_usd == null) continue;
        if (costFilter.min != null && stats.average_cost_usd < costFilter.min) continue;
        if (costFilter.max != null && stats.average_cost_usd > costFilter.max) continue;
      }
      result[slug] = stats;
    }
    return result;
  }, [categoryStats, timeSlugs, sdkFilter, starsFilter, starsFilterActive, costFilter, costFilterActive]);

  const rows = useMemo(
    () => computeRows(supercategories, filteredCategoryStats),
    [supercategories, filteredCategoryStats],
  );

  const totalRecords = useMemo(
    () => rows.reduce((sum, row) => sum + row.recordCount, 0),
    [rows],
  );

  const displayRows = useMemo<DisplayRow[]>(
    () =>
      rows
        .filter((row) => row.recordCount > 0)
        .map((row) => ({
          ...row,
          share: totalRecords > 0 ? row.recordCount / totalRecords : 0,
        })),
    [rows, totalRecords],
  );

  const chartConfig = useMemo(() => {
    const config: ChartConfig = {};
    for (const row of displayRows) {
      config[row.slug] = { label: row.title, color: row.color };
    }
    return config;
  }, [displayRows]);

  const activeRow = useMemo(
    () => displayRows.find((row) => row.slug === activeSlug) ?? null,
    [displayRows, activeSlug],
  );

  // Only bail if the unfiltered data has no supercategories at all
  const unfilteredRows = useMemo(
    () => computeRows(supercategories, categoryStats),
    [supercategories, categoryStats],
  );
  const hasAnyData = unfilteredRows.some((row) => row.recordCount > 0);
  if (!hasAnyData) return null;

  return (
    <section>
      <div className="mb-2 flex items-center justify-between gap-3">
        <h2 className="text-[10px] font-medium uppercase tracking-[0.06em] text-[var(--text-tertiary)]">
          Supercategories
        </h2>
        <span className="text-[10px] text-[var(--text-quaternary)]">
          Hover a group to highlight it
        </span>
      </div>

      <div className="rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] p-4">
        {/* --- filter bar --- */}
        <div className="mb-3 flex flex-wrap items-center gap-1.5">
          <DashboardStarsFilter value={starsFilter} onChange={setStarsFilter} />
          <DashboardTimeFilter value={timeFilter} onChange={setTimeFilter} />
          <DashboardCostFilter bounds={categoryCostBounds} value={costFilter} onChange={setCostFilter} />

          {availableSdks.length > 1 ? (
            <Select value={sdkFilter ?? "all"} onValueChange={(v) => setSdkFilter(v === "all" ? null : v)}>
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
                setTimeFilter({ oldest: null, newest: null });
                setStarsFilter([STAR_SLIDER_MIN, STAR_SLIDER_MAX]);
                setCostFilter({ min: null, max: null });
                setSdkFilter(null);
              }}
              className="h-6 px-2 text-[10px] text-[var(--text-tertiary)] transition-colors hover:text-[var(--text-secondary)]"
            >
              Clear all
            </button>
          ) : null}
        </div>
        {displayRows.length === 0 ? (
          <div className="py-8 text-center text-[11px] text-[var(--text-quaternary)]">
            No supercategories match the current filters
          </div>
        ) : (
        <div
          className="flex flex-col gap-4 md:flex-row"
          onMouseLeave={() => setActiveSlug(null)}
        >
          <div className="flex shrink-0 flex-col items-center justify-center md:w-[248px]">
            <ChartContainer config={chartConfig} className="aspect-square h-[236px] w-[236px]">
              <PieChart>
                <Tooltip cursor={false} content={<CustomTooltip />} />

                <Pie
                  data={[{ value: totalRecords }]}
                  dataKey="value"
                  innerRadius={70}
                  outerRadius={102}
                  strokeWidth={0}
                  isAnimationActive={false}
                >
                  <Cell fill="var(--surface-2)" />
                </Pie>

                <Pie
                  data={displayRows}
                  dataKey="recordCount"
                  nameKey="title"
                  innerRadius={70}
                  outerRadius={102}
                  paddingAngle={2}
                  cornerRadius={5}
                  shape={(props) => renderSliceShape(props, activeSlug)}
                  onMouseEnter={(_, index) => {
                    const hoveredRow = displayRows[index];
                    setActiveSlug(hoveredRow?.slug ?? null);
                  }}
                >
                  {displayRows.map((row) => (
                    <Cell key={row.slug} fill={row.color} />
                  ))}
                  <Label
                    content={({ viewBox }) => {
                      if (!viewBox || !("cx" in viewBox) || !("cy" in viewBox)) {
                        return null;
                      }

                      const centerValue = activeRow
                        ? formatRecordCount(activeRow.recordCount)
                        : formatRecordCount(totalRecords);
                      const centerLabel = activeRow
                        ? formatShare(activeRow.share)
                        : "records";

                      return (
                        <text x={viewBox.cx} y={viewBox.cy} textAnchor="middle" dominantBaseline="middle">
                          <tspan
                            x={viewBox.cx}
                            y={(viewBox.cy ?? 0) - 2}
                            className="fill-[var(--text-primary)] text-[24px] font-semibold"
                            style={{ fontFamily: "var(--font-mono, ui-monospace, monospace)" }}
                          >
                            {centerValue}
                          </tspan>
                          <tspan
                            x={viewBox.cx}
                            y={(viewBox.cy ?? 0) + 18}
                            className="fill-[var(--text-quaternary)] text-[11px]"
                          >
                            {centerLabel}
                          </tspan>
                        </text>
                      );
                    }}
                  />
                </Pie>
              </PieChart>
            </ChartContainer>

            <p className="mt-1 text-center text-[10px] text-[var(--text-quaternary)]">
              {activeRow
                ? `${activeRow.title} · ${activeRow.categoryCount} categories`
                : "Distribution by supercategory"}
            </p>
          </div>

          <div className="min-w-0 flex-1">
            <Table className="text-[11px]">
              <TableHeader>
                <TableRow className="border-b border-[var(--border-default)] hover:bg-transparent">
                  <TableHead className={`${HEAD_CLASSES} w-full`}>Group</TableHead>
                  <TableHead className={`${HEAD_CLASSES} text-right`}>Cat</TableHead>
                  <TableHead className={`${HEAD_CLASSES} text-right`}>Rec</TableHead>
                  <TableHead className={`${HEAD_CLASSES} text-right`}>Stars</TableHead>
                  <TableHead className={`${HEAD_CLASSES} text-right`}>Cost</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {displayRows.map((row) => {
                  const isActive = row.slug === activeSlug;

                  return (
                    <TableRow
                      key={row.slug}
                      className={cn(
                        "border-b border-[var(--border-subtle)]",
                        isActive ? "" : "hover:bg-[var(--surface-1)]",
                      )}
                      onMouseEnter={() => setActiveSlug(row.slug)}
                      style={isActive
                        ? {
                            backgroundColor: hexToRgba(row.color, 0.06),
                            boxShadow: `inset 2px 0 0 ${row.color}`,
                          }
                        : undefined}
                    >
                      <TableCell className="w-full px-3 py-[6px] text-[var(--text-secondary)]">
                        <button
                          type="button"
                          onFocus={() => setActiveSlug(row.slug)}
                          onBlur={() => {
                            setActiveSlug((current) => (current === row.slug ? null : current));
                          }}
                          className="flex w-full items-center gap-2 text-left focus:outline-none"
                        >
                          <span
                            className="inline-block size-2 shrink-0 rounded-full"
                            style={{ backgroundColor: row.color }}
                          />
                          <span className="truncate">{row.title}</span>
                          <span className="ml-auto shrink-0 font-mono text-[10px] text-[var(--text-quaternary)]">
                            {formatShare(row.share)}
                          </span>
                        </button>
                      </TableCell>
                      <TableCell className="whitespace-nowrap px-3 py-[6px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
                        {row.categoryCount}
                      </TableCell>
                      <TableCell className="whitespace-nowrap px-3 py-[6px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
                        {row.recordCount}
                      </TableCell>
                      <TableCell className="whitespace-nowrap px-3 py-[6px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
                        {formatRating(row.avgRating)}
                      </TableCell>
                      <TableCell className="whitespace-nowrap px-3 py-[6px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
                        {formatCost(row.avgCost)}
                      </TableCell>
                    </TableRow>
                  );
                })}
              </TableBody>
            </Table>
          </div>
        </div>
        )}
      </div>
    </section>
  );
}
