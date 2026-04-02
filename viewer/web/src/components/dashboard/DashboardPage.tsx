import { useState, useEffect, useCallback, type JSX } from "react";

import { fetchDatasetEntries, fetchRepoStats } from "@/lib/api";
import type { DatasetEntry, RepoStats } from "@/lib/types";
import { useViewer } from "@/lib/viewer-context";
import { OverviewSection } from "@/components/dashboard/OverviewSection";
import { SupercategoriesSection } from "@/components/dashboard/SupercategoriesSection";
import { CategoriesSection } from "@/components/dashboard/CategoriesSection";

export function DashboardPage(): JSX.Element {
  const { bootstrap } = useViewer();
  const [stats, setStats] = useState<RepoStats | null>(null);
  const [statsError, setStatsError] = useState<string | null>(null);
  const [datasetEntries, setDatasetEntries] = useState<DatasetEntry[] | null>(null);

  const loadStats = useCallback(() => {
    fetchRepoStats()
      .then((data) => {
        setStats(data);
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

  if (!bootstrap && !stats) {
    return (
      <div className="flex h-full items-center justify-center">
        <p className="text-[12px] text-[var(--text-quaternary)]">Loading...</p>
      </div>
    );
  }

  return (
    <main className="custom-scrollbar h-full overflow-y-auto bg-[var(--surface-2)]">
      <div className="mx-auto max-w-[960px] px-6 py-6">
        <div className="flex flex-col gap-6">
          {stats ? (
            <OverviewSection stats={stats} />
          ) : statsError ? (
            <p className="text-[11px] text-[var(--text-quaternary)]">
              Failed to load stats: {statsError}
            </p>
          ) : null}

          {stats && bootstrap?.supercategories && bootstrap.supercategories.length > 0 ? (
            <SupercategoriesSection
              categoryStats={stats.category_stats}
              supercategories={bootstrap.supercategories}
              datasetEntries={datasetEntries ?? undefined}
            />
          ) : null}

          {stats ? (
            <CategoriesSection
              categoryStats={stats.category_stats}
              supercategories={bootstrap?.supercategories}
              datasetEntries={datasetEntries ?? undefined}
            />
          ) : null}
        </div>
      </div>
    </main>
  );
}
