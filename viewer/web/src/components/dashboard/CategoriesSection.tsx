import { useCallback, useState, useMemo, type JSX } from "react";
import { ArrowDown, ArrowUp, ArrowUpDown, ChevronDown, ChevronRight, Download, Search } from "lucide-react";

import { formatCost, formatTokenCount } from "@/lib/dashboard-stats";
import { formatCategoryLabel } from "@/lib/utils";
import type { SupercategoryOption } from "@/lib/types";
import { Badge } from "@/components/ui/badge";
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

type CategoriesSectionProps = {
  categoryStats: Record<string, CategoryStats>;
  supercategories?: SupercategoryOption[];
};

type SortKey =
  | "category"
  | "sdk_package"
  | "count"
  | "average_rating"
  | "average_cost_usd"
  | "average_input_tokens"
  | "average_output_tokens";
type SortDirection = "asc" | "desc";

function formatAverageRating(value: number | null): string {
  return value != null ? `${value.toFixed(1)}★` : "—";
}

function formatSdkPackage(value: string | null): string {
  if (value === "sdk_hybrid") return "sdk_hybrid";
  if (value === "sdk") return "sdk";
  return "—";
}

type CategoryRow = [string, CategoryStats];

function sortRows(rows: CategoryRow[], sortKey: SortKey, sortDirection: SortDirection): CategoryRow[] {
  return [...rows].sort((left, right) => {
    const [leftCategory, leftStats] = left;
    const [rightCategory, rightStats] = right;
    const leftCategoryLabel = formatCategoryLabel(leftCategory);
    const rightCategoryLabel = formatCategoryLabel(rightCategory);

    const categoryFallback = leftCategoryLabel.localeCompare(rightCategoryLabel);

    if (sortKey === "category") {
      return sortDirection === "asc" ? categoryFallback : -categoryFallback;
    }

    if (sortKey === "sdk_package") {
      const leftValue = formatSdkPackage(leftStats.sdk_package);
      const rightValue = formatSdkPackage(rightStats.sdk_package);
      const sdkDelta = leftValue.localeCompare(rightValue);
      if (sdkDelta === 0) return categoryFallback;
      return sortDirection === "asc" ? sdkDelta : -sdkDelta;
    }

    const leftValue = leftStats[sortKey];
    const rightValue = rightStats[sortKey];

    if (leftValue == null && rightValue == null) return categoryFallback;
    if (leftValue == null) return 1;
    if (rightValue == null) return -1;

    const numericDelta = leftValue - rightValue;
    if (numericDelta === 0) return categoryFallback;
    return sortDirection === "asc" ? numericDelta : -numericDelta;
  });
}

type SupercategoryGroup = {
  slug: string;
  title: string;
  rows: CategoryRow[];
  totalCount: number;
  avgRating: number | null;
  avgCost: number | null;
  avgInputTokens: number | null;
  avgOutputTokens: number | null;
};

function buildGroups(
  rows: CategoryRow[],
  supercategories: SupercategoryOption[],
): SupercategoryGroup[] {
  const catToSuper = new Map<string, string>();
  const superBySlug = new Map<string, SupercategoryOption>();
  for (const sc of supercategories) {
    superBySlug.set(sc.slug, sc);
    for (const cat of sc.category_slugs) {
      catToSuper.set(cat, sc.slug);
    }
  }

  const grouped = new Map<string, CategoryRow[]>();
  for (const sc of supercategories) {
    grouped.set(sc.slug, []);
  }
  grouped.set("__uncategorized__", []);

  for (const row of rows) {
    const superSlug = catToSuper.get(row[0]);
    const key = superSlug ?? "__uncategorized__";
    grouped.get(key)!.push(row);
  }

  const groups: SupercategoryGroup[] = [];
  for (const sc of supercategories) {
    const scRows = grouped.get(sc.slug) ?? [];
    if (scRows.length === 0) continue;
    groups.push(buildGroupStats(sc.slug, sc.title, scRows));
  }
  const uncategorized = grouped.get("__uncategorized__") ?? [];
  if (uncategorized.length > 0) {
    groups.push(buildGroupStats("__uncategorized__", "Uncategorized", uncategorized));
  }
  return groups;
}

function buildGroupStats(slug: string, title: string, rows: CategoryRow[]): SupercategoryGroup {
  let totalCount = 0;
  let ratingSum = 0;
  let ratingCount = 0;
  let costSum = 0;
  let costCount = 0;
  let inputTokenSum = 0;
  let inputTokenCount = 0;
  let outputTokenSum = 0;
  let outputTokenCount = 0;
  for (const [, stats] of rows) {
    totalCount += stats.count;
    if (stats.average_rating != null) {
      ratingSum += stats.average_rating * stats.count;
      ratingCount += stats.count;
    }
    if (stats.average_cost_usd != null) {
      costSum += stats.average_cost_usd * stats.count;
      costCount += stats.count;
    }
    if (stats.average_input_tokens != null && stats.input_token_sample_count > 0) {
      inputTokenSum += stats.average_input_tokens * stats.input_token_sample_count;
      inputTokenCount += stats.input_token_sample_count;
    }
    if (stats.average_output_tokens != null && stats.output_token_sample_count > 0) {
      outputTokenSum += stats.average_output_tokens * stats.output_token_sample_count;
      outputTokenCount += stats.output_token_sample_count;
    }
  }
  return {
    slug,
    title,
    rows,
    totalCount,
    avgRating: ratingCount > 0 ? Math.round((ratingSum / ratingCount) * 10) / 10 : null,
    avgCost: costCount > 0 ? Math.round((costSum / costCount) * 10000) / 10000 : null,
    avgInputTokens: inputTokenCount > 0 ? Math.round(inputTokenSum / inputTokenCount) : null,
    avgOutputTokens: outputTokenCount > 0 ? Math.round(outputTokenSum / outputTokenCount) : null,
  };
}

function CategoryTableRows({
  rows,
  startIndex,
}: {
  rows: CategoryRow[];
  startIndex: number;
}): JSX.Element {
  return (
    <>
      {rows.map(([category, stats], index) => (
        <TableRow
          key={category}
          className="border-b border-[var(--border-subtle)] hover:bg-[var(--surface-1)]"
        >
          <TableCell className="w-8 px-0 py-[5px] text-center font-mono tabular-nums text-[var(--text-quaternary)]">
            {startIndex + index + 1}
          </TableCell>
          <TableCell className="w-full px-4 py-[5px] text-[var(--text-secondary)]">
            {formatCategoryLabel(category)}
          </TableCell>
          <TableCell className="whitespace-nowrap px-4 py-[5px] text-[var(--text-tertiary)]">
            {stats.sdk_package ? (
              <Badge variant="secondary" className="rounded-sm px-1.5 py-0 font-mono text-[10px]">
                {formatSdkPackage(stats.sdk_package)}
              </Badge>
            ) : (
              <span className="font-mono text-[var(--text-quaternary)]">—</span>
            )}
          </TableCell>
          <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
            {stats.count}
          </TableCell>
          <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
            {formatAverageRating(stats.average_rating)}
          </TableCell>
          <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
            {formatCost(stats.average_cost_usd)}
          </TableCell>
          <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
            {formatTokenCount(stats.average_input_tokens)}
          </TableCell>
          <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
            {formatTokenCount(stats.average_output_tokens)}
          </TableCell>
        </TableRow>
      ))}
    </>
  );
}

export function CategoriesSection({
  categoryStats,
  supercategories,
}: CategoriesSectionProps): JSX.Element {
  const [filter, setFilter] = useState("");
  const [sortKey, setSortKey] = useState<SortKey>("count");
  const [sortDirection, setSortDirection] = useState<SortDirection>("desc");
  const [collapsedGroups, setCollapsedGroups] = useState<Set<string>>(new Set());

  const allEntries = useMemo(
    () => Object.entries(categoryStats) as CategoryRow[],
    [categoryStats],
  );

  const allSorted = useMemo(
    () => sortRows(allEntries, sortKey, sortDirection),
    [allEntries, sortKey, sortDirection],
  );

  const exportCsv = useCallback(
    (rows: CategoryRow[]) => {
      const header = "Category,SDK,Count,Avg Stars,Avg Cost USD,Avg Input Tokens,Avg Output Tokens";
      const lines = rows.map(([slug, s]) => {
        const cat = formatCategoryLabel(slug).replaceAll('"', '""');
        const sdk = formatSdkPackage(s.sdk_package);
        const rating = s.average_rating != null ? s.average_rating.toFixed(1) : "";
        const cost = s.average_cost_usd != null ? s.average_cost_usd.toFixed(4) : "";
        const inputTokens =
          s.average_input_tokens != null ? String(Math.round(s.average_input_tokens)) : "";
        const outputTokens =
          s.average_output_tokens != null ? String(Math.round(s.average_output_tokens)) : "";
        return `"${cat}",${sdk},${s.count},${rating},${cost},${inputTokens},${outputTokens}`;
      });
      const blob = new Blob([header + "\n" + lines.join("\n") + "\n"], { type: "text/csv" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "categories.csv";
      a.click();
      URL.revokeObjectURL(url);
    },
    [],
  );

  const filtered = useMemo(() => {
    const needle = filter.trim().toLowerCase();
    return needle
      ? allSorted.filter(([category]) => formatCategoryLabel(category).toLowerCase().includes(needle))
      : allSorted;
  }, [allSorted, filter]);

  const hasSupercategories = supercategories && supercategories.length > 0;

  const groups = useMemo(() => {
    if (!hasSupercategories) return null;
    return buildGroups(
      sortRows(
        filtered.map(([cat, stats]) => [cat, stats]),
        sortKey,
        sortDirection,
      ),
      supercategories,
    );
  }, [filtered, hasSupercategories, supercategories, sortKey, sortDirection]);

  function toggleSort(nextSortKey: SortKey): void {
    if (nextSortKey === sortKey) {
      setSortDirection((current) => (current === "asc" ? "desc" : "asc"));
      return;
    }
    setSortKey(nextSortKey);
    setSortDirection(nextSortKey === "category" || nextSortKey === "sdk_package" ? "asc" : "desc");
  }

  function toggleGroup(slug: string): void {
    setCollapsedGroups((prev) => {
      const next = new Set(prev);
      if (next.has(slug)) {
        next.delete(slug);
      } else {
        next.add(slug);
      }
      return next;
    });
  }

  function SortIndicator({ column }: { column: SortKey }): JSX.Element {
    if (sortKey !== column) {
      return <ArrowUpDown className="size-3 text-[var(--text-quaternary)]" />;
    }
    if (sortDirection === "asc") {
      return <ArrowUp className="size-3 text-[var(--text-tertiary)]" />;
    }
    return <ArrowDown className="size-3 text-[var(--text-tertiary)]" />;
  }

  let runningIndex = 0;

  return (
    <section>
      <div className="mb-2 flex items-baseline justify-between">
        <h2 className="text-[10px] font-medium uppercase tracking-[0.06em] text-[var(--text-tertiary)]">
          Categories
          <span className="ml-1.5 font-mono font-normal tabular-nums text-[var(--text-quaternary)]">
            {filtered.length}
          </span>
        </h2>
        <button
          type="button"
          onClick={() => exportCsv(filtered)}
          title="Export CSV"
          className="rounded p-0.5 text-[var(--text-quaternary)] transition-colors hover:bg-[var(--surface-1)] hover:text-[var(--text-tertiary)]"
        >
          <Download className="size-3" />
        </button>
      </div>
      <div className="rounded-md border border-[var(--border-default)] bg-[var(--surface-0)]">
        {allEntries.length > 12 ? (
          <div className="flex items-center gap-1.5 border-b border-[var(--border-subtle)] px-3 py-1.5">
            <Search className="size-3 shrink-0 text-[var(--text-quaternary)]" />
            <input
              type="text"
              value={filter}
              onChange={(e) => setFilter(e.target.value)}
              placeholder="Filter categories..."
              className="h-6 flex-1 bg-transparent text-[11px] text-[var(--text-primary)] placeholder:text-[var(--text-quaternary)] focus:outline-none"
            />
          </div>
        ) : null}

        <div className="custom-scrollbar max-h-[320px] overflow-auto">
          {filtered.length === 0 ? (
            <div className="px-4 py-3">
              <span className="text-[11px] text-[var(--text-quaternary)]">
                No matching categories
              </span>
            </div>
          ) : (
            <Table className="text-[11px]">
              <TableHeader>
                <TableRow className="hover:bg-transparent">
                  <TableHead className="h-8 w-8 px-0 py-[5px] text-center text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    #
                  </TableHead>
                  <TableHead className="h-8 px-4 py-[5px] text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("category")}
                      className="flex items-center gap-1 text-left"
                    >
                      <span>Category</span>
                      <SortIndicator column="category" />
                    </button>
                  </TableHead>
                  <TableHead className="h-8 whitespace-nowrap px-4 py-[5px] text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("sdk_package")}
                      className="flex items-center gap-1 text-left"
                    >
                      <span>SDK</span>
                      <SortIndicator column="sdk_package" />
                    </button>
                  </TableHead>
                  <TableHead className="h-8 whitespace-nowrap px-4 py-[5px] text-right text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("count")}
                      className="ml-auto flex items-center gap-1"
                    >
                      <span>Count</span>
                      <SortIndicator column="count" />
                    </button>
                  </TableHead>
                  <TableHead className="h-8 whitespace-nowrap px-4 py-[5px] text-right text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("average_rating")}
                      className="ml-auto flex items-center gap-1"
                    >
                      <span>Avg Stars</span>
                      <SortIndicator column="average_rating" />
                    </button>
                  </TableHead>
                  <TableHead className="h-8 whitespace-nowrap px-4 py-[5px] text-right text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("average_cost_usd")}
                      className="ml-auto flex items-center gap-1"
                    >
                      <span>Avg Cost</span>
                      <SortIndicator column="average_cost_usd" />
                    </button>
                  </TableHead>
                  <TableHead className="h-8 whitespace-nowrap px-4 py-[5px] text-right text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("average_input_tokens")}
                      className="ml-auto flex items-center gap-1"
                    >
                      <span>Avg Input</span>
                      <SortIndicator column="average_input_tokens" />
                    </button>
                  </TableHead>
                  <TableHead className="h-8 whitespace-nowrap px-4 py-[5px] text-right text-[10px] uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
                    <button
                      type="button"
                      onClick={() => toggleSort("average_output_tokens")}
                      className="ml-auto flex items-center gap-1"
                    >
                      <span>Avg Output</span>
                      <SortIndicator column="average_output_tokens" />
                    </button>
                  </TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {groups
                  ? groups.map((group) => {
                      const collapsed = collapsedGroups.has(group.slug);
                      const groupStartIndex = runningIndex;
                      if (!collapsed) {
                        runningIndex += group.rows.length;
                      }
                      return (
                        <SupercategoryGroupRows
                          key={group.slug}
                          group={group}
                          collapsed={collapsed}
                          startIndex={groupStartIndex}
                          onToggle={() => toggleGroup(group.slug)}
                        />
                      );
                    })
                  : (
                    <CategoryTableRows rows={filtered} startIndex={0} />
                  )}
              </TableBody>
            </Table>
          )}
        </div>
      </div>
    </section>
  );
}

function SupercategoryGroupRows({
  group,
  collapsed,
  startIndex,
  onToggle,
}: {
  group: SupercategoryGroup;
  collapsed: boolean;
  startIndex: number;
  onToggle: () => void;
}): JSX.Element {
  return (
    <>
      <TableRow
        className="border-b border-[var(--border-default)] bg-[var(--surface-1)] hover:bg-[var(--surface-1)]"
      >
        <TableCell colSpan={3} className="px-2 py-[5px]">
          <button
            type="button"
            onClick={onToggle}
            className="flex items-center gap-1.5 text-left text-[11px] font-medium text-[var(--text-secondary)]"
          >
            {collapsed ? (
              <ChevronRight className="size-3 text-[var(--text-tertiary)]" />
            ) : (
              <ChevronDown className="size-3 text-[var(--text-tertiary)]" />
            )}
            <span>{group.title}</span>
            <span className="font-normal text-[var(--text-quaternary)]">
              ({group.rows.length})
            </span>
          </button>
        </TableCell>
        <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
          {group.totalCount}
        </TableCell>
        <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
          {formatAverageRating(group.avgRating)}
        </TableCell>
        <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
          {formatCost(group.avgCost)}
        </TableCell>
        <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
          {formatTokenCount(group.avgInputTokens)}
        </TableCell>
        <TableCell className="whitespace-nowrap px-4 py-[5px] text-right font-mono tabular-nums text-[var(--text-tertiary)]">
          {formatTokenCount(group.avgOutputTokens)}
        </TableCell>
      </TableRow>
      {!collapsed ? (
        <CategoryTableRows rows={group.rows} startIndex={startIndex} />
      ) : null}
    </>
  );
}
