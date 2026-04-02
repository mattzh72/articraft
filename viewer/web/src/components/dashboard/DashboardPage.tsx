import { useState, useEffect, useCallback, useMemo, type JSX } from "react";

import { fetchDatasetEntries, fetchRepoStats } from "@/lib/api";
import {
  buildDashboardCategoryStats,
  buildDashboardOverviewStats,
  collectDatasetRecords,
  filterDashboardRecords,
  getDashboardAvailableSdks,
  getDashboardCostBounds,
  hasActiveDashboardFilters,
} from "@/lib/dashboard-data";
import type { DatasetEntry, RepoStats, TimeFilter } from "@/lib/types";
import { useViewer } from "@/lib/viewer-context";
import { CostTrendSection } from "@/components/dashboard/CostTrendSection";
import { DashboardFilterBar } from "@/components/dashboard/DashboardFilterBar";
import { OverviewSection } from "@/components/dashboard/OverviewSection";
import { SupercategoriesSection } from "@/components/dashboard/SupercategoriesSection";
import { CategoriesSection } from "@/components/dashboard/CategoriesSection";
import { STAR_SLIDER_MAX, STAR_SLIDER_MIN } from "@/components/dashboard/dashboard-filters";

export function DashboardPage(): JSX.Element {
  const { bootstrap } = useViewer();
  const [repoStats, setRepoStats] = useState<RepoStats | null>(null);
  const [statsError, setStatsError] = useState<string | null>(null);
  const [datasetEntries, setDatasetEntries] = useState<DatasetEntry[] | null>(null);
  const [timeFilter, setTimeFilter] = useState<TimeFilter>({ oldest: null, newest: null });
  const [starsFilter, setStarsFilter] = useState<[number, number]>([
    STAR_SLIDER_MIN,
    STAR_SLIDER_MAX,
  ]);
  const [costFilter, setCostFilter] = useState<{ min: number | null; max: number | null }>({
    min: null,
    max: null,
  });
  const [sdkFilter, setSdkFilter] = useState<string | null>(null);
  const [rollingWindowDays, setRollingWindowDays] = useState(14);

  const loadStats = useCallback(() => {
    fetchRepoStats()
      .then((data) => {
        setRepoStats(data);
        setStatsError(null);
      })
      .catch((err) => {
        setStatsError(err instanceof Error ? err.message : "Failed to load stats");
      });
  }, []);

  useEffect(() => {
    loadStats();
  }, [loadStats]);

  useEffect(() => {
    fetchDatasetEntries()
      .then((entries) => {
        setDatasetEntries(entries);
      })
      .catch(() => {
        setDatasetEntries([]);
      });
  }, [bootstrap?.generated_at]);

  const allDatasetRecords = useMemo(
    () => collectDatasetRecords(datasetEntries ?? []),
    [datasetEntries],
  );

  const dashboardFilters = useMemo(
    () => ({
      timeFilter,
      starsFilter,
      costFilter,
      sdkFilter,
    }),
    [timeFilter, starsFilter, costFilter, sdkFilter],
  );

  const filteredDatasetRecords = useMemo(
    () => filterDashboardRecords(allDatasetRecords, dashboardFilters),
    [allDatasetRecords, dashboardFilters],
  );

  const availableSdks = useMemo(
    () => getDashboardAvailableSdks(allDatasetRecords),
    [allDatasetRecords],
  );

  const costBounds = useMemo(
    () => getDashboardCostBounds(allDatasetRecords),
    [allDatasetRecords],
  );

  const categoryStats = useMemo(
    () => buildDashboardCategoryStats(filteredDatasetRecords),
    [filteredDatasetRecords],
  );

  const overviewStats = useMemo(
    () =>
      buildDashboardOverviewStats(filteredDatasetRecords, {
        dataSizeBytes: repoStats?.data_size_bytes ?? null,
        isFiltered: hasActiveDashboardFilters(dashboardFilters),
      }),
    [filteredDatasetRecords, repoStats?.data_size_bytes, dashboardFilters],
  );

  if (!bootstrap && datasetEntries == null && !repoStats) {
    return (
      <div className="flex h-full items-center justify-center">
        <p className="text-[12px] text-[var(--text-quaternary)]">Loading...</p>
      </div>
    );
  }

  return (
    <main className="custom-scrollbar h-full overflow-y-auto bg-[var(--surface-2)]">
      <div className="mx-auto max-w-[1120px] px-6 py-6">
        <div className="flex flex-col gap-6">
          {datasetEntries ? (
            <DashboardFilterBar
              timeFilter={timeFilter}
              onTimeFilterChange={setTimeFilter}
              starsFilter={starsFilter}
              onStarsFilterChange={setStarsFilter}
              costFilter={costFilter}
              onCostFilterChange={setCostFilter}
              sdkFilter={sdkFilter}
              onSdkFilterChange={setSdkFilter}
              availableSdks={availableSdks}
              costBounds={costBounds}
              recordCount={filteredDatasetRecords.length}
              categoryCount={overviewStats.categoryCount}
            />
          ) : null}

          {statsError ? (
            <p className="text-[11px] text-[var(--text-quaternary)]">
              Some repo stats are unavailable: {statsError}
            </p>
          ) : null}

          {datasetEntries ? (
            <OverviewSection stats={overviewStats} />
          ) : null}

          {datasetEntries ? (
            <CostTrendSection
              records={filteredDatasetRecords}
              rollingWindowDays={rollingWindowDays}
              onRollingWindowDaysChange={setRollingWindowDays}
            />
          ) : null}

          {bootstrap?.supercategories && bootstrap.supercategories.length > 0 ? (
            <SupercategoriesSection
              categoryStats={categoryStats}
              supercategories={bootstrap.supercategories}
            />
          ) : null}

          {datasetEntries ? (
            <CategoriesSection
              categoryStats={categoryStats}
              supercategories={bootstrap?.supercategories}
            />
          ) : null}
        </div>
      </div>
    </main>
  );
}
