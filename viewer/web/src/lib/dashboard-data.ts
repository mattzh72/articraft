import type { CostFilter, DatasetEntry, RecordSummary, TimeFilter } from "@/lib/types";

const TIME_FILTER_DURATIONS_MS: Record<string, number> = {
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

export type DashboardFilters = {
  timeFilter: TimeFilter;
  starsFilter: [number, number];
  costFilter: CostFilter;
  sdkFilter: string | null;
};

export type DashboardOverviewStats = {
  totalRecords: number;
  totalRuns: number;
  totalCostUsd: number | null;
  averageCostUsd: number | null;
  dataSizeBytes: number | null;
  categoryCount: number;
  modelCount: number;
  sdkCount: number;
  isFiltered: boolean;
};

export type DashboardCategoryStats = {
  count: number;
  sdk_package: string | null;
  average_rating: number | null;
  average_cost_usd: number | null;
  average_input_tokens: number | null;
  average_output_tokens: number | null;
  input_token_sample_count: number;
  output_token_sample_count: number;
};

export type DashboardCostTrendPoint = {
  dateKey: string;
  dayStartMs: number;
  recordCount: number;
  totalCostUsd: number;
  dailyAverageCostUsd: number | null;
  rollingAverageCostUsd: number | null;
};

export type DashboardCostTrendSummary = {
  points: DashboardCostTrendPoint[];
  latestAverageCostUsd: number | null;
  previousAverageCostUsd: number | null;
  deltaUsd: number | null;
  deltaPct: number | null;
};

function parseIsoTimestamp(value: string | null): number | null {
  if (!value) return null;
  const timestamp = new Date(value).getTime();
  return Number.isFinite(timestamp) ? timestamp : null;
}

function localDayKeyFromTimestamp(timestamp: number): string {
  const date = new Date(timestamp);
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, "0");
  const day = String(date.getDate()).padStart(2, "0");
  return `${year}-${month}-${day}`;
}

function localDayStartMsFromTimestamp(timestamp: number): number {
  const date = new Date(timestamp);
  return new Date(date.getFullYear(), date.getMonth(), date.getDate()).getTime();
}

function uniqueNonEmpty(values: Array<string | null | undefined>): string[] {
  return Array.from(
    new Set(
      values.map((value) => value?.trim() ?? "").filter((value) => value.length > 0),
    ),
  ).sort((left, right) => left.localeCompare(right));
}

export function hasActiveDashboardFilters(filters: DashboardFilters): boolean {
  return (
    filters.timeFilter.oldest != null ||
    filters.timeFilter.newest != null ||
    filters.sdkFilter != null ||
    filters.costFilter.min != null ||
    filters.costFilter.max != null ||
    filters.starsFilter[0] > 0 ||
    filters.starsFilter[1] < 5
  );
}

export function collectDatasetRecords(entries: DatasetEntry[]): RecordSummary[] {
  return entries
    .map((entry) => entry.record)
    .filter((record): record is RecordSummary => record != null);
}

export function getDashboardAvailableSdks(records: RecordSummary[]): string[] {
  return uniqueNonEmpty(records.map((record) => record.sdk_package));
}

export function getDashboardCostBounds(
  records: RecordSummary[],
): { min: number; max: number } | null {
  const values = records
    .map((record) => record.total_cost_usd)
    .filter((value): value is number => value != null && Number.isFinite(value));
  if (values.length === 0) return null;
  return {
    min: Math.min(...values),
    max: Math.max(...values),
  };
}

export function filterDashboardRecords(
  records: RecordSummary[],
  filters: DashboardFilters,
  now: number = Date.now(),
): RecordSummary[] {
  const starsFilterActive = filters.starsFilter[0] > 0 || filters.starsFilter[1] < 5;
  const oldestDuration =
    filters.timeFilter.oldest != null
      ? TIME_FILTER_DURATIONS_MS[filters.timeFilter.oldest]
      : null;
  const newestDuration =
    filters.timeFilter.newest != null
      ? TIME_FILTER_DURATIONS_MS[filters.timeFilter.newest]
      : null;

  return records.filter((record) => {
    if (filters.sdkFilter && record.sdk_package !== filters.sdkFilter) {
      return false;
    }

    if (oldestDuration != null || newestDuration != null) {
      const createdAtMs = parseIsoTimestamp(record.created_at);
      if (createdAtMs == null) return false;
      const age = now - createdAtMs;
      if (oldestDuration != null && age > oldestDuration) return false;
      if (newestDuration != null && age < newestDuration) return false;
    }

    if (filters.costFilter.min != null || filters.costFilter.max != null) {
      const totalCostUsd = record.total_cost_usd;
      if (totalCostUsd == null) return false;
      if (filters.costFilter.min != null && totalCostUsd < filters.costFilter.min) {
        return false;
      }
      if (filters.costFilter.max != null && totalCostUsd > filters.costFilter.max) {
        return false;
      }
    }

    if (starsFilterActive) {
      const rating = record.effective_rating;
      if (rating == null) return false;
      if (rating < filters.starsFilter[0] || rating > filters.starsFilter[1]) {
        return false;
      }
    }

    return true;
  });
}

export function buildDashboardOverviewStats(
  records: RecordSummary[],
  {
    dataSizeBytes,
    isFiltered,
  }: {
  dataSizeBytes: number | null;
  isFiltered: boolean;
}): DashboardOverviewStats {
  const totalCostValues = records
    .map((record) => record.total_cost_usd)
    .filter((value): value is number => value != null && Number.isFinite(value));
  const totalCostUsd =
    totalCostValues.length > 0 ? totalCostValues.reduce((sum, value) => sum + value, 0) : null;

  return {
    totalRecords: records.length,
    totalRuns: uniqueNonEmpty(records.map((record) => record.run_id)).length,
    totalCostUsd,
    averageCostUsd:
      totalCostUsd != null && records.length > 0 ? totalCostUsd / records.length : null,
    dataSizeBytes,
    categoryCount: uniqueNonEmpty(records.map((record) => record.category_slug)).length,
    modelCount: uniqueNonEmpty(records.map((record) => record.model_id)).length,
    sdkCount: uniqueNonEmpty(records.map((record) => record.sdk_package)).length,
    isFiltered,
  };
}

export function buildDashboardCategoryStats(
  filteredRecords: RecordSummary[],
): Record<string, DashboardCategoryStats> {
  const categorySdkPackages = new Map<string, string | null>();
  const sdkByCategory = new Map<string, Set<string>>();

  for (const record of filteredRecords) {
    const categorySlug = record.category_slug?.trim();
    const sdkPackage = record.sdk_package?.trim();
    if (!categorySlug || !sdkPackage) continue;
    let sdkSet = sdkByCategory.get(categorySlug);
    if (!sdkSet) {
      sdkSet = new Set<string>();
      sdkByCategory.set(categorySlug, sdkSet);
    }
    sdkSet.add(sdkPackage);
  }

  for (const [categorySlug, sdkSet] of sdkByCategory.entries()) {
    categorySdkPackages.set(categorySlug, sdkSet.size === 1 ? Array.from(sdkSet)[0] : null);
  }

  const aggregates = new Map<
    string,
    {
      count: number;
      ratingTotal: number;
      ratingCount: number;
      costTotal: number;
      costCount: number;
      inputTokenTotal: number;
      inputTokenCount: number;
      outputTokenTotal: number;
      outputTokenCount: number;
    }
  >();

  for (const record of filteredRecords) {
    const categorySlug = record.category_slug?.trim();
    if (!categorySlug) continue;
    let aggregate = aggregates.get(categorySlug);
    if (!aggregate) {
      aggregate = {
        count: 0,
        ratingTotal: 0,
        ratingCount: 0,
        costTotal: 0,
        costCount: 0,
        inputTokenTotal: 0,
        inputTokenCount: 0,
        outputTokenTotal: 0,
        outputTokenCount: 0,
      };
      aggregates.set(categorySlug, aggregate);
    }

    aggregate.count += 1;

    if (record.effective_rating != null) {
      aggregate.ratingTotal += record.effective_rating;
      aggregate.ratingCount += 1;
    }
    if (record.total_cost_usd != null) {
      aggregate.costTotal += record.total_cost_usd;
      aggregate.costCount += 1;
    }
    if (record.input_tokens != null) {
      aggregate.inputTokenTotal += record.input_tokens;
      aggregate.inputTokenCount += 1;
    }
    if (record.output_tokens != null) {
      aggregate.outputTokenTotal += record.output_tokens;
      aggregate.outputTokenCount += 1;
    }
  }

  const categoryStats: Record<string, DashboardCategoryStats> = {};
  for (const [categorySlug, aggregate] of aggregates.entries()) {
    categoryStats[categorySlug] = {
      count: aggregate.count,
      sdk_package: categorySdkPackages.get(categorySlug) ?? null,
      average_rating:
        aggregate.ratingCount > 0
          ? Math.round((aggregate.ratingTotal / aggregate.ratingCount) * 100) / 100
          : null,
      average_cost_usd:
        aggregate.costCount > 0
          ? Math.round((aggregate.costTotal / aggregate.costCount) * 10000) / 10000
          : null,
      average_input_tokens:
        aggregate.inputTokenCount > 0
          ? Math.round(aggregate.inputTokenTotal / aggregate.inputTokenCount)
          : null,
      average_output_tokens:
        aggregate.outputTokenCount > 0
          ? Math.round(aggregate.outputTokenTotal / aggregate.outputTokenCount)
          : null,
      input_token_sample_count: aggregate.inputTokenCount,
      output_token_sample_count: aggregate.outputTokenCount,
    };
  }

  return categoryStats;
}

export function buildDashboardCostTrend(
  records: RecordSummary[],
  rollingWindowDays: number,
): DashboardCostTrendSummary {
  const datedRecords = records
    .map((record) => ({
      createdAtMs: parseIsoTimestamp(record.created_at),
      totalCostUsd: record.total_cost_usd,
    }))
    .filter(
      (
        record,
      ): record is {
        createdAtMs: number;
        totalCostUsd: number;
      } => record.createdAtMs != null && record.totalCostUsd != null,
    );

  if (datedRecords.length === 0) {
    return {
      points: [],
      latestAverageCostUsd: null,
      previousAverageCostUsd: null,
      deltaUsd: null,
      deltaPct: null,
    };
  }

  const aggregates = new Map<
    string,
    { dayStartMs: number; recordCount: number; totalCostUsd: number }
  >();

  for (const record of datedRecords) {
    const dateKey = localDayKeyFromTimestamp(record.createdAtMs);
    const dayStartMs = localDayStartMsFromTimestamp(record.createdAtMs);
    const aggregate = aggregates.get(dateKey) ?? {
      dayStartMs,
      recordCount: 0,
      totalCostUsd: 0,
    };
    aggregate.recordCount += 1;
    aggregate.totalCostUsd += record.totalCostUsd;
    aggregates.set(dateKey, aggregate);
  }

  const sortedKeys = Array.from(aggregates.keys()).sort((left, right) => left.localeCompare(right));
  const firstDayStartMs = aggregates.get(sortedKeys[0])?.dayStartMs ?? 0;
  const lastDayStartMs = aggregates.get(sortedKeys[sortedKeys.length - 1])?.dayStartMs ?? 0;
  const dayMs = 24 * 60 * 60 * 1000;

  const points: DashboardCostTrendPoint[] = [];
  for (let dayStartMs = firstDayStartMs; dayStartMs <= lastDayStartMs; dayStartMs += dayMs) {
    const dateKey = localDayKeyFromTimestamp(dayStartMs);
    const aggregate = aggregates.get(dateKey);
    const recordCount = aggregate?.recordCount ?? 0;
    const totalCostUsd = aggregate?.totalCostUsd ?? 0;
    points.push({
      dateKey,
      dayStartMs,
      recordCount,
      totalCostUsd,
      dailyAverageCostUsd: recordCount > 0 ? totalCostUsd / recordCount : null,
      rollingAverageCostUsd: null,
    });
  }

  const costPrefix: number[] = [0];
  const countPrefix: number[] = [0];
  for (const point of points) {
    costPrefix.push(costPrefix[costPrefix.length - 1] + point.totalCostUsd);
    countPrefix.push(countPrefix[countPrefix.length - 1] + point.recordCount);
  }

  for (let index = 0; index < points.length; index += 1) {
    const startIndex = Math.max(0, index - rollingWindowDays + 1);
    const totalCostUsd = costPrefix[index + 1] - costPrefix[startIndex];
    const recordCount = countPrefix[index + 1] - countPrefix[startIndex];
    points[index] = {
      ...points[index],
      rollingAverageCostUsd: recordCount > 0 ? totalCostUsd / recordCount : null,
    };
  }

  const endIndex = points.length - 1;
  const currentStartIndex = Math.max(0, endIndex - rollingWindowDays + 1);
  const currentCostUsd = costPrefix[endIndex + 1] - costPrefix[currentStartIndex];
  const currentRecordCount = countPrefix[endIndex + 1] - countPrefix[currentStartIndex];
  const latestAverageCostUsd =
    currentRecordCount > 0 ? currentCostUsd / currentRecordCount : null;

  const previousEndIndex = currentStartIndex - 1;
  let previousAverageCostUsd: number | null = null;
  if (previousEndIndex >= 0) {
    const previousStartIndex = previousEndIndex - rollingWindowDays + 1;
    if (previousStartIndex >= 0) {
      const previousCostUsd =
        costPrefix[previousEndIndex + 1] - costPrefix[previousStartIndex];
      const previousRecordCount =
        countPrefix[previousEndIndex + 1] - countPrefix[previousStartIndex];
      previousAverageCostUsd =
        previousRecordCount > 0 ? previousCostUsd / previousRecordCount : null;
    }
  }

  const deltaUsd =
    latestAverageCostUsd != null && previousAverageCostUsd != null
      ? latestAverageCostUsd - previousAverageCostUsd
      : null;
  const deltaPct =
    deltaUsd != null && previousAverageCostUsd != null && previousAverageCostUsd > 0
      ? (deltaUsd / previousAverageCostUsd) * 100
      : null;

  return {
    points,
    latestAverageCostUsd,
    previousAverageCostUsd,
    deltaUsd,
    deltaPct,
  };
}
