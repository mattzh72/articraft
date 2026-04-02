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

import { formatCost, formatTokenCount } from "@/lib/dashboard-stats";
import type { SupercategoryOption } from "@/lib/types";
import { cn } from "@/lib/utils";
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

type CategoryStats = {
  count: number;
  sdk_package: string | null;
  average_rating: number | null;
  average_cost_usd: number | null;
  average_input_tokens: number | null;
  average_output_tokens: number | null;
  input_token_sample_count: number;
  output_token_sample_count: number;
};

type SupercategoriesSectionProps = {
  categoryStats: Record<string, CategoryStats>;
  supercategories: SupercategoryOption[];
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
  avgInputTokens: number | null;
  avgOutputTokens: number | null;
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
    let inputTokenSum = 0;
    let inputTokenWeights = 0;
    let outputTokenSum = 0;
    let outputTokenWeights = 0;
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
      if (stats.average_input_tokens != null && stats.input_token_sample_count > 0) {
        inputTokenSum += stats.average_input_tokens * stats.input_token_sample_count;
        inputTokenWeights += stats.input_token_sample_count;
      }
      if (stats.average_output_tokens != null && stats.output_token_sample_count > 0) {
        outputTokenSum += stats.average_output_tokens * stats.output_token_sample_count;
        outputTokenWeights += stats.output_token_sample_count;
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
      avgInputTokens: inputTokenWeights > 0 ? Math.round(inputTokenSum / inputTokenWeights) : null,
      avgOutputTokens:
        outputTokenWeights > 0 ? Math.round(outputTokenSum / outputTokenWeights) : null,
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
        <div>
          Avg input: {formatTokenCount(row.avgInputTokens)} · Avg output: {formatTokenCount(row.avgOutputTokens)}
        </div>
      </div>
    </div>
  );
}

export function SupercategoriesSection({
  categoryStats,
  supercategories,
}: SupercategoriesSectionProps): JSX.Element | null {
  const [activeSlug, setActiveSlug] = useState<string | null>(null);

  const rows = useMemo(
    () => computeRows(supercategories, categoryStats),
    [supercategories, categoryStats],
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

  if (supercategories.length === 0) return null;

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
                          <text
                            x={viewBox.cx}
                            y={viewBox.cy}
                            textAnchor="middle"
                            dominantBaseline="middle"
                          >
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

            <div className="custom-scrollbar min-w-0 flex-1 overflow-auto">
              <Table className="text-[11px]">
                <TableHeader>
                  <TableRow className="border-b border-[var(--border-default)] hover:bg-transparent">
                    <TableHead className={`${HEAD_CLASSES} w-full`}>Group</TableHead>
                    <TableHead className={`${HEAD_CLASSES} text-right`}>Cat</TableHead>
                    <TableHead className={`${HEAD_CLASSES} text-right`}>Rec</TableHead>
                    <TableHead className={`${HEAD_CLASSES} text-right`}>Stars</TableHead>
                    <TableHead className={`${HEAD_CLASSES} text-right`}>Cost</TableHead>
                    <TableHead className={`${HEAD_CLASSES} text-right`}>Input</TableHead>
                    <TableHead className={`${HEAD_CLASSES} text-right`}>Output</TableHead>
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
                        <TableCell className="whitespace-nowrap px-3 py-[6px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
                          {formatTokenCount(row.avgInputTokens)}
                        </TableCell>
                        <TableCell className="whitespace-nowrap px-3 py-[6px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
                          {formatTokenCount(row.avgOutputTokens)}
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
