import { queryOptions } from "@tanstack/react-query";

import {
  browseRecords,
  fetchBootstrap,
  fetchBrowseRecordIds,
  fetchRecordSummary,
  fetchRepoStats,
  fetchStagingEntries,
  searchRecords,
} from "@/lib/api";

type BrowseRecordsParams = Parameters<typeof browseRecords>[0];
type BrowseRecordIdsParams = Parameters<typeof fetchBrowseRecordIds>[0];
type SearchRecordsParams = Parameters<typeof searchRecords>[0];

export const viewerQueryKeys = {
  root: () => ["viewer"] as const,
  bootstrap: () => [...viewerQueryKeys.root(), "bootstrap"] as const,
  repoStats: () => [...viewerQueryKeys.root(), "repo-stats"] as const,
  stagingEntries: () => [...viewerQueryKeys.root(), "staging-entries"] as const,
  recordSummary: (recordId: string) => [...viewerQueryKeys.root(), "record-summary", recordId] as const,
  browseRecords: (params: BrowseRecordsParams) =>
    [...viewerQueryKeys.root(), "browse-records", params] as const,
  browseRecordIds: (params: BrowseRecordIdsParams) =>
    [...viewerQueryKeys.root(), "browse-record-ids", params] as const,
  searchRecords: (params: SearchRecordsParams) =>
    [...viewerQueryKeys.root(), "search-records", params] as const,
};

export function bootstrapQueryOptions() {
  return queryOptions({
    queryKey: viewerQueryKeys.bootstrap(),
    queryFn: fetchBootstrap,
    staleTime: 5_000,
  });
}

export function repoStatsQueryOptions() {
  return queryOptions({
    queryKey: viewerQueryKeys.repoStats(),
    queryFn: fetchRepoStats,
    staleTime: 10_000,
  });
}

export function stagingEntriesQueryOptions() {
  return queryOptions({
    queryKey: viewerQueryKeys.stagingEntries(),
    queryFn: fetchStagingEntries,
    staleTime: 0,
  });
}

export function recordSummaryQueryOptions(recordId: string) {
  return queryOptions({
    queryKey: viewerQueryKeys.recordSummary(recordId),
    queryFn: () => fetchRecordSummary(recordId),
    staleTime: 5_000,
  });
}

export function browseRecordsQueryOptions(params: BrowseRecordsParams) {
  return queryOptions({
    queryKey: viewerQueryKeys.browseRecords(params),
    queryFn: () => browseRecords(params),
    staleTime: 10_000,
  });
}

export function browseRecordIdsQueryOptions(params: BrowseRecordIdsParams) {
  return queryOptions({
    queryKey: viewerQueryKeys.browseRecordIds(params),
    queryFn: () => fetchBrowseRecordIds(params),
    staleTime: 10_000,
  });
}

export function searchRecordsQueryOptions(params: SearchRecordsParams) {
  return queryOptions({
    queryKey: viewerQueryKeys.searchRecords(params),
    queryFn: () => searchRecords(params),
    staleTime: 10_000,
  });
}
