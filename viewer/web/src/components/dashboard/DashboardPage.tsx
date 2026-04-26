import {
  lazy,
  Suspense,
  useDeferredValue,
  useEffect,
  useMemo,
  useState,
  type JSX,
} from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { DASHBOARD_REFRESH_EVENT } from "@/lib/dashboard-events";
import type { DashboardData, TimeFilter } from "@/lib/types";
import { dashboardQueryOptions } from "@/lib/viewer-queries";
import { DashboardFilterBar } from "@/components/dashboard/DashboardFilterBar";
import { OverviewSection } from "@/components/dashboard/OverviewSection";
import { CategoriesSection } from "@/components/dashboard/CategoriesSection";
import { STAR_SLIDER_MAX, STAR_SLIDER_MIN } from "@/components/dashboard/dashboard-filters";

const CostTrendSection = lazy(() =>
  import("@/components/dashboard/CostTrendSection").then((module) => ({
    default: module.CostTrendSection,
  })),
);

const SupercategoriesSection = lazy(() =>
  import("@/components/dashboard/SupercategoriesSection").then((module) => ({
    default: module.SupercategoriesSection,
  })),
);

function DashboardSectionSkeleton({ heightClass }: { heightClass: string }): JSX.Element {
  return (
    <div className="rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] p-4">
      <div className={`animate-pulse rounded-md bg-[var(--surface-1)] ${heightClass}`} />
    </div>
  );
}

export function DashboardPage(): JSX.Element {
  const queryClient = useQueryClient();
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
  const [authorFilters, setAuthorFilters] = useState<string[]>([]);
  const [categoryFilters, setCategoryFilters] = useState<string[]>([]);
  const [rollingWindowDays, setRollingWindowDays] = useState(14);

  const requestParams = useMemo(
    () => ({
      timeFilter,
      starsFilter,
      costFilter,
      sdkFilter,
      authorFilters,
      categoryFilters,
      rollingWindowDays,
    }),
    [authorFilters, categoryFilters, costFilter, rollingWindowDays, sdkFilter, starsFilter, timeFilter],
  );
  const deferredRequest = useDeferredValue(requestParams);
  const dashboardQuery = useQuery(
    dashboardQueryOptions({
      timeFilter: deferredRequest.timeFilter,
      starsFilter: deferredRequest.starsFilter,
      costFilter: deferredRequest.costFilter,
      sdkFilter: deferredRequest.sdkFilter,
      authorFilters: deferredRequest.authorFilters,
      categoryFilters: deferredRequest.categoryFilters,
      rollingWindowDays: deferredRequest.rollingWindowDays,
    }),
  );
  const dashboard: DashboardData | null = dashboardQuery.data ?? null;
  const loadError = dashboardQuery.error instanceof Error ? dashboardQuery.error.message : null;
  const loading = dashboardQuery.isFetching;

  useEffect(() => {
    const handleRefresh = () => {
      void queryClient.invalidateQueries({ queryKey: ["viewer", "dashboard"] });
    };

    window.addEventListener(DASHBOARD_REFRESH_EVENT, handleRefresh);
    return () => {
      window.removeEventListener(DASHBOARD_REFRESH_EVENT, handleRefresh);
    };
  }, [queryClient]);

  if (!dashboard && dashboardQuery.isPending) {
    return (
      <div className="flex h-full items-center justify-center">
        <p className="text-[12px] text-[var(--text-quaternary)]">Loading...</p>
      </div>
    );
  }

  if (!dashboard) {
    return (
      <div className="flex h-full items-center justify-center">
        <p className="max-w-md px-6 text-center text-[12px] text-[var(--text-quaternary)]">
          {loadError ?? "Failed to load dashboard"}
        </p>
      </div>
    );
  }

  return (
    <main className="custom-scrollbar h-full overflow-y-auto bg-[var(--surface-2)]">
      <div className="mx-auto max-w-[1120px] px-6 py-6">
        <div className="flex flex-col gap-6">
          <DashboardFilterBar
            timeFilter={timeFilter}
            onTimeFilterChange={setTimeFilter}
            starsFilter={starsFilter}
            onStarsFilterChange={setStarsFilter}
            costFilter={costFilter}
            onCostFilterChange={setCostFilter}
            sdkFilter={sdkFilter}
            onSdkFilterChange={setSdkFilter}
            authorFilters={authorFilters}
            onAuthorFiltersChange={setAuthorFilters}
            categoryFilters={categoryFilters}
            onCategoryFiltersChange={setCategoryFilters}
            availableSdks={dashboard.available_sdks}
            availableAuthors={dashboard.available_authors}
            availableCategories={dashboard.available_categories}
            costBounds={dashboard.cost_bounds}
            recordCount={dashboard.overview.total_records}
            categoryCount={dashboard.overview.category_count}
            loading={loading}
          />

          {loadError ? (
            <p className="text-[11px] text-[var(--text-quaternary)]">
              Some dashboard data may be stale: {loadError}
            </p>
          ) : null}

          <OverviewSection stats={dashboard.overview} />

          <Suspense fallback={<DashboardSectionSkeleton heightClass="h-[260px]" />}>
            <CostTrendSection
              trend={dashboard.cost_trend}
              rollingWindowDays={rollingWindowDays}
              onRollingWindowDaysChange={setRollingWindowDays}
            />
          </Suspense>

          {dashboard.supercategories.length > 0 ? (
            <Suspense fallback={<DashboardSectionSkeleton heightClass="h-[420px]" />}>
              <SupercategoriesSection
                categoryStats={dashboard.category_stats}
                supercategories={dashboard.supercategories}
              />
            </Suspense>
          ) : null}

          <CategoriesSection
            categoryStats={dashboard.category_stats}
            supercategories={dashboard.supercategories}
          />
        </div>
      </div>
    </main>
  );
}
