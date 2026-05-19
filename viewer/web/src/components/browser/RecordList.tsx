import {
  useDeferredValue,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type JSX,
} from "react";
import { useQueryClient } from "@tanstack/react-query";
import { AutoSizer } from "react-virtualized-auto-sizer";
import { List, type ListImperativeAPI, type RowComponentProps } from "react-window";

import { RecordListItem } from "@/components/browser/RecordListItem";
import type { DatasetFilterMetadata } from "@/components/browser/ExplorerFilters";
import { recordMatchesAgentHarnessFilters } from "@/lib/agent-harness";
import type {
  CostFilter,
  RatingFilter,
  RecordBrowseResponse,
  RecordSummary,
  TimeFilter,
} from "@/lib/types";
import {
  browseRecordIdsQueryOptions,
  browseRecordsQueryOptions,
  searchRecordsQueryOptions,
} from "@/lib/viewer-queries";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

const TIME_DURATIONS: Record<string, number> = {
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

const DATASET_PAGE_SIZE = 120;
const DEFAULT_LIST_HEIGHT_PX = 480;
const VIRTUAL_OVERSCAN = 8;
const RECORD_ROW_HEIGHT_PX = 68;

function withinTimeFilter(createdAt: string | null, filter: TimeFilter): boolean {
  if (!filter.oldest && !filter.newest) return true;
  if (!createdAt) return false;

  const createdAtMs = new Date(createdAt).getTime();
  if (Number.isNaN(createdAtMs)) return false;

  const age = Date.now() - createdAtMs;
  if (filter.oldest) {
    const maxAge = TIME_DURATIONS[filter.oldest];
    if (maxAge != null && age > maxAge) return false;
  }
  if (filter.newest) {
    const minAge = TIME_DURATIONS[filter.newest];
    if (minAge != null && age < minAge) return false;
  }
  return true;
}

function withinCostFilter(totalCostUsd: number | null, filter: CostFilter): boolean {
  if (filter.min == null && filter.max == null) return true;
  if (totalCostUsd == null) return false;
  if (filter.min != null && totalCostUsd < filter.min) return false;
  if (filter.max != null && totalCostUsd > filter.max) return false;
  return true;
}

function withinRatingFilter(rating: number | null, filter: RatingFilter): boolean {
  if (filter.length === 0) return true;
  if (rating == null) return filter.includes("unrated");
  if (rating < 2) return filter.includes("1");
  if (rating < 3) return filter.includes("2");
  if (rating < 4) return filter.includes("3");
  if (rating < 5) return filter.includes("4");
  return filter.includes("5");
}

function recordSortTimestamp(record: RecordSummary): number {
  const timestamp = record.updated_at ?? record.created_at;
  return timestamp ? new Date(timestamp).getTime() : 0;
}

type VirtualRowData = {
  recordIdForIndex: (index: number) => string | null;
  recordForId: (recordId: string) => RecordSummary | null;
  repoRoot: string | null;
  selectedRecordId: string | null;
  multiSelectActive: boolean;
  multiSelection: Set<string>;
  onSelect: (recordId: string) => void;
  onMultiSelectToggle: (recordId: string, shiftKey: boolean) => void;
};

function VirtualRecordRow({
  ariaAttributes,
  index,
  style,
  recordIdForIndex,
  recordForId,
  repoRoot,
  selectedRecordId,
  multiSelectActive,
  multiSelection,
  onSelect,
  onMultiSelectToggle,
}: RowComponentProps<VirtualRowData>): JSX.Element | null {
  const recordId = recordIdForIndex(index);
  if (!recordId) {
    return (
      <div
        {...ariaAttributes}
        style={{
          ...style,
          boxSizing: "border-box",
          paddingInline: "2px",
        }}
      >
        <div className="flex h-full items-center border-b border-[var(--border-default)] px-3 text-[11px] text-[var(--text-quaternary)]">
          Loading...
        </div>
      </div>
    );
  }

  return (
    <div
      {...ariaAttributes}
      style={{
        ...style,
        boxSizing: "border-box",
        paddingInline: "2px",
      }}
    >
      <RecordListItem
        recordId={recordId}
        record={recordForId(recordId)}
        repoRoot={repoRoot}
        isSelected={selectedRecordId === recordId}
        multiSelectActive={multiSelectActive}
        isMultiSelected={multiSelection.has(recordId)}
        onSelect={onSelect}
        onMultiSelectToggle={onMultiSelectToggle}
      />
    </div>
  );
}

interface RecordListProps {
  onVisibleIdsChange?: (ids: string[]) => void;
  onCountsChange?: (counts: { visible: number; total: number }) => void;
  onDatasetMetadataChange?: (metadata: DatasetFilterMetadata | null) => void;
}

export function RecordList({
  onVisibleIdsChange,
  onCountsChange,
  onDatasetMetadataChange,
}: RecordListProps): JSX.Element {
  const queryClient = useQueryClient();
  const {
    bootstrap,
    recordCache,
    searchQuery,
    sourceFilter,
    timeFilter,
    modelFilter,
    sdkFilter,
    agentHarnessFilters,
    authorFilters,
    categoryFilters,
    costFilter,
    ratingFilter,
    secondaryRatingFilter,
    selectedRunId,
    selectedRecordId,
    multiSelection,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const listRef = useRef<ListImperativeAPI | null>(null);
  const previousSelectedRecordIdRef = useRef<string | null>(selectedRecordId);
  const previousSelectedVisibleIndexRef = useRef<number | null>(null);
  const selectionAdvanceDirectionRef = useRef<1 | -1>(1);
  const followSelectionRef = useRef(true);
  const activeDatasetRequestIdRef = useRef(0);
  const loadedDatasetPagesRef = useRef<Set<number>>(new Set());
  const loadingDatasetPagesRef = useRef<Set<number>>(new Set());
  const deferredSearchQuery = useDeferredValue(searchQuery.trim());
  const [searchedRecords, setSearchedRecords] = useState<RecordSummary[] | null>(null);
  const [searchPending, setSearchPending] = useState(false);
  const [datasetRecordIdsByIndex, setDatasetRecordIdsByIndex] = useState<Record<number, string>>({});
  const [datasetRecordsById, setDatasetRecordsById] = useState<Record<string, RecordSummary>>({});
  const [datasetMatchTotal, setDatasetMatchTotal] = useState(0);
  const [datasetSourceTotal, setDatasetSourceTotal] = useState(0);
  const [datasetLoading, setDatasetLoading] = useState(false);

  const workbenchSourceRecords = useMemo(() => {
    if (!bootstrap || sourceFilter !== "workbench") {
      return [];
    }

    const seen = new Map<string, RecordSummary>();
    for (const entry of bootstrap.workbench_entries) {
      if (entry.record && !seen.has(entry.record_id)) {
        seen.set(entry.record_id, entry.record);
      }
    }
    return Array.from(seen.values());
  }, [bootstrap, sourceFilter]);

  const workbenchRecordById = useMemo(
    () => new Map(workbenchSourceRecords.map((record) => [record.record_id, record])),
    [workbenchSourceRecords],
  );

  useEffect(() => {
    if (sourceFilter !== "workbench" || !deferredSearchQuery) {
      setSearchedRecords(null);
      setSearchPending(false);
      return;
    }

    let cancelled = false;
    setSearchedRecords(null);
    setSearchPending(true);

    queryClient.fetchQuery(
      searchRecordsQueryOptions({
        query: deferredSearchQuery,
        source: sourceFilter,
        runId: selectedRunId,
        timeFilter,
        modelFilter,
        sdkFilter,
        agentHarnessFilters,
        authorFilters: [],
        categoryFilters: [],
        costFilter,
        ratingFilter,
        secondaryRatingFilter,
        limit: 200,
      }),
    )
      .then((results) => {
        if (!cancelled) {
          setSearchedRecords(results);
          setSearchPending(false);
          dispatch({ type: "UPSERT_RECORDS", payload: results });
        }
      })
      .catch(() => {
        if (!cancelled) {
          setSearchedRecords([]);
          setSearchPending(false);
        }
      });

    return () => {
      cancelled = true;
    };
  }, [
    costFilter,
    deferredSearchQuery,
    dispatch,
    modelFilter,
    agentHarnessFilters,
    ratingFilter,
    secondaryRatingFilter,
    sdkFilter,
    selectedRunId,
    sourceFilter,
    timeFilter,
    queryClient,
  ]);

  const workbenchRecords = useMemo(() => {
    if (!bootstrap || sourceFilter !== "workbench") return [];

    let list = deferredSearchQuery
      ? (searchedRecords ?? []).map((record) => workbenchRecordById.get(record.record_id) ?? record)
      : workbenchSourceRecords;

    if (!deferredSearchQuery && selectedRunId) {
      list = list.filter((record) => record.run_id === selectedRunId);
    }

    if (!deferredSearchQuery) {
      list = list.filter((record) => withinTimeFilter(record.created_at, timeFilter));
      list = list.filter((record) => withinCostFilter(record.total_cost_usd, costFilter));
      list = list.filter((record) => withinRatingFilter(record.rating, ratingFilter));
      list = list.filter((record) => withinRatingFilter(record.secondary_rating ?? null, secondaryRatingFilter));
      if (modelFilter) {
        list = list.filter((record) => record.model_id === modelFilter);
      }
      if (sdkFilter) {
        list = list.filter((record) => record.sdk_package === sdkFilter);
      }
      list = list.filter((record) => recordMatchesAgentHarnessFilters(record, agentHarnessFilters));
      list.sort((left, right) => recordSortTimestamp(right) - recordSortTimestamp(left));
    }

    return list;
  }, [
    bootstrap,
    costFilter,
    deferredSearchQuery,
    modelFilter,
    agentHarnessFilters,
    ratingFilter,
    secondaryRatingFilter,
    searchedRecords,
    sdkFilter,
    selectedRunId,
    sourceFilter,
    timeFilter,
    workbenchRecordById,
    workbenchSourceRecords,
  ]);

  const datasetBrowseParams = useMemo(
    () => ({
      source: "dataset" as const,
      query: deferredSearchQuery,
      runId: selectedRunId,
      timeFilter,
      modelFilter,
      sdkFilter,
      agentHarnessFilters,
      authorFilters,
      categoryFilters,
      costFilter,
      ratingFilter,
      secondaryRatingFilter,
    }),
    [
      authorFilters,
      categoryFilters,
      costFilter,
      deferredSearchQuery,
      modelFilter,
      agentHarnessFilters,
      ratingFilter,
      secondaryRatingFilter,
      selectedRunId,
      sdkFilter,
      timeFilter,
    ],
  );

  const datasetGeneratedAt = bootstrap?.generated_at ?? "";

  const mergeDatasetRecords = useCallback(
    (records: RecordSummary[]) => {
      if (records.length === 0) {
        return;
      }
      dispatch({ type: "UPSERT_RECORDS", payload: records });
      setDatasetRecordsById((current) => {
        const next = { ...current };
        for (const record of records) {
          next[record.record_id] = record;
        }
        return next;
      });
    },
    [dispatch],
  );

  const applyDatasetMetadata = useCallback(
    (response: RecordBrowseResponse) => {
      onDatasetMetadataChange?.({
        availableModels: response.facets.models,
        availableSdks: response.facets.sdk_packages,
        availableAgentHarnesses: response.facets.agent_harnesses,
        availableAuthors: response.facets.authors,
        availableCategories: response.facets.categories,
        availableCostBounds:
          response.facets.cost_min != null && response.facets.cost_max != null
            ? { min: response.facets.cost_min, max: response.facets.cost_max }
            : null,
      });
      setDatasetMatchTotal(response.total);
      setDatasetSourceTotal(response.source_total);
    },
    [onDatasetMetadataChange],
  );

  const mergeDatasetPage = useCallback(
    (response: RecordBrowseResponse) => {
      mergeDatasetRecords(response.records);
      setDatasetRecordIdsByIndex((current) => {
        const next = { ...current };
        response.records.forEach((record, index) => {
          next[response.offset + index] = record.record_id;
        });
        return next;
      });
    },
    [mergeDatasetRecords],
  );

  const loadDatasetPage = useCallback(
    (pageIndex: number) => {
      if (sourceFilter !== "dataset") {
        return;
      }
      if (loadedDatasetPagesRef.current.has(pageIndex) || loadingDatasetPagesRef.current.has(pageIndex)) {
        return;
      }

      const requestId = activeDatasetRequestIdRef.current;
      loadingDatasetPagesRef.current.add(pageIndex);

      queryClient.fetchQuery(
        browseRecordsQueryOptions({
          ...datasetBrowseParams,
          offset: pageIndex * DATASET_PAGE_SIZE,
          limit: DATASET_PAGE_SIZE,
        }),
      )
        .then((response) => {
          if (activeDatasetRequestIdRef.current !== requestId) {
            return;
          }
          loadingDatasetPagesRef.current.delete(pageIndex);
          loadedDatasetPagesRef.current.add(pageIndex);
          applyDatasetMetadata(response);
          mergeDatasetPage(response);
        })
        .catch(() => {
          if (activeDatasetRequestIdRef.current !== requestId) {
            return;
          }
          loadingDatasetPagesRef.current.delete(pageIndex);
        });
    },
    [applyDatasetMetadata, datasetBrowseParams, mergeDatasetPage, queryClient, sourceFilter],
  );

  useEffect(() => {
    if (sourceFilter !== "dataset") {
      onDatasetMetadataChange?.(null);
      setDatasetLoading(false);
      setDatasetRecordIdsByIndex({});
      setDatasetRecordsById({});
      setDatasetMatchTotal(0);
      setDatasetSourceTotal(0);
      loadedDatasetPagesRef.current.clear();
      loadingDatasetPagesRef.current.clear();
      return;
    }

    let cancelled = false;
    const requestId = activeDatasetRequestIdRef.current + 1;
    activeDatasetRequestIdRef.current = requestId;
    loadedDatasetPagesRef.current.clear();
    loadingDatasetPagesRef.current.clear();
    setDatasetLoading(true);
    setDatasetRecordIdsByIndex({});
    setDatasetRecordsById({});
    setDatasetMatchTotal(0);
    setDatasetSourceTotal(0);

    queryClient.fetchQuery(
      browseRecordsQueryOptions({
        ...datasetBrowseParams,
        offset: 0,
        limit: DATASET_PAGE_SIZE,
      }),
    )
      .then((response) => {
        if (cancelled || activeDatasetRequestIdRef.current !== requestId) {
          return;
        }
        loadedDatasetPagesRef.current.add(0);
        applyDatasetMetadata(response);
        mergeDatasetPage(response);
        setDatasetLoading(false);
      })
      .catch(() => {
        if (cancelled || activeDatasetRequestIdRef.current !== requestId) {
          return;
        }
        onDatasetMetadataChange?.(null);
        setDatasetRecordIdsByIndex({});
        setDatasetRecordsById({});
        setDatasetMatchTotal(0);
        setDatasetSourceTotal(0);
        setDatasetLoading(false);
      });

    return () => {
      cancelled = true;
    };
  }, [
    applyDatasetMetadata,
    datasetBrowseParams,
    datasetGeneratedAt,
    mergeDatasetPage,
    onDatasetMetadataChange,
    queryClient,
    sourceFilter,
  ]);

  const recordForId = useCallback(
    (recordId: string): RecordSummary | null => {
      if (sourceFilter === "dataset") {
        return recordCache[recordId] ?? datasetRecordsById[recordId] ?? null;
      }
      return workbenchRecordById.get(recordId) ?? recordCache[recordId] ?? null;
    },
    [datasetRecordsById, recordCache, sourceFilter, workbenchRecordById],
  );

  const workbenchVisibleIds = useMemo(
    () => workbenchRecords.map((record) => record.record_id),
    [workbenchRecords],
  );

  const datasetLoadedIds = useMemo(
    () =>
      Object.entries(datasetRecordIdsByIndex)
        .sort(([left], [right]) => Number(left) - Number(right))
        .map(([, recordId]) => recordId),
    [datasetRecordIdsByIndex],
  );

  const visibleIds = sourceFilter === "dataset" ? datasetLoadedIds : workbenchVisibleIds;
  const rowCount = sourceFilter === "dataset" ? datasetMatchTotal : visibleIds.length;

  const recordIndexById = useMemo(() => {
    const entries =
      sourceFilter === "dataset"
        ? Object.entries(datasetRecordIdsByIndex).map(
            ([index, recordId]) => [recordId, Number(index)] as const,
          )
        : workbenchVisibleIds.map((recordId, index) => [recordId, index] as const);
    return new Map(entries);
  }, [datasetRecordIdsByIndex, sourceFilter, workbenchVisibleIds]);

  const recordIdForIndex = useCallback(
    (index: number): string | null => {
      if (sourceFilter === "dataset") {
        return datasetRecordIdsByIndex[index] ?? null;
      }
      return workbenchVisibleIds[index] ?? null;
    },
    [datasetRecordIdsByIndex, sourceFilter, workbenchVisibleIds],
  );

  const counts = useMemo(
    () => ({
      visible: sourceFilter === "dataset" ? datasetMatchTotal : visibleIds.length,
      total: sourceFilter === "dataset" ? datasetSourceTotal : workbenchSourceRecords.length,
    }),
    [
      datasetMatchTotal,
      datasetSourceTotal,
      sourceFilter,
      visibleIds.length,
      workbenchSourceRecords.length,
    ],
  );

  useEffect(() => {
    onVisibleIdsChange?.(visibleIds);
  }, [onVisibleIdsChange, visibleIds]);

  useEffect(() => {
    onCountsChange?.(counts);
  }, [counts, onCountsChange]);

  useEffect(() => {
    const previousSelectedVisibleIndex = previousSelectedVisibleIndexRef.current;
    const selectedVisibleIndex = selectedRecordId ? (recordIndexById.get(selectedRecordId) ?? -1) : -1;

    if (selectedRecordId && selectedVisibleIndex === -1) {
      if (previousSelectedVisibleIndex != null) {
        if (rowCount === 0) {
          dispatch({ type: "SELECT_RECORD", payload: null });
          return;
        }

        const replacementIndex =
          selectionAdvanceDirectionRef.current === -1
            ? Math.max(previousSelectedVisibleIndex - 1, 0)
            : Math.min(previousSelectedVisibleIndex, rowCount - 1);
        const replacementRecordId = recordIdForIndex(replacementIndex);
        if (replacementRecordId) {
          if (sourceFilter === "dataset") {
            loadDatasetPage(Math.floor(replacementIndex / DATASET_PAGE_SIZE));
          }
          dispatch({ type: "SELECT_RECORD", payload: replacementRecordId });
          return;
        }
      }
    }

    previousSelectedVisibleIndexRef.current = selectedVisibleIndex >= 0 ? selectedVisibleIndex : null;
  }, [
    dispatch,
    loadDatasetPage,
    recordIdForIndex,
    recordIndexById,
    rowCount,
    selectedRecordId,
    sourceFilter,
  ]);

  useEffect(() => {
    const element = listRef.current?.element;
    if (!element) {
      return;
    }

    const disableAutoFollow = () => {
      followSelectionRef.current = false;
    };

    element.addEventListener("wheel", disableAutoFollow, { passive: true });
    element.addEventListener("touchmove", disableAutoFollow, { passive: true });
    element.addEventListener("pointerdown", disableAutoFollow);

    return () => {
      element.removeEventListener("wheel", disableAutoFollow);
      element.removeEventListener("touchmove", disableAutoFollow);
      element.removeEventListener("pointerdown", disableAutoFollow);
    };
  }, [visibleIds.length]);

  useEffect(() => {
    if (selectedRecordId !== previousSelectedRecordIdRef.current) {
      followSelectionRef.current = true;
      previousSelectedRecordIdRef.current = selectedRecordId;
    }

    if (!followSelectionRef.current || !selectedRecordId) {
      return;
    }

    const selectedIndex = recordIndexById.get(selectedRecordId) ?? -1;
    if (selectedIndex === -1) {
      return;
    }

    if (sourceFilter === "dataset") {
      loadDatasetPage(Math.floor(selectedIndex / DATASET_PAGE_SIZE));
    }

    listRef.current?.scrollToRow({
      align: "smart",
      behavior: "instant",
      index: selectedIndex,
    });
  }, [loadDatasetPage, recordIndexById, selectedRecordId, sourceFilter, visibleIds]);

  const multiSelectActive = multiSelection.size > 0;

  const onMultiSelectToggle = useCallback(
    (recordId: string, shiftKey: boolean) => {
      if (shiftKey && multiSelectActive) {
        dispatch({ type: "RANGE_MULTI_SELECT", payload: { targetId: recordId, visibleIds } });
      } else {
        dispatch({ type: "TOGGLE_MULTI_SELECT", payload: recordId });
      }
    },
    [dispatch, multiSelectActive, visibleIds],
  );

  const onSelect = useCallback(
    (recordId: string) => {
      selectionAdvanceDirectionRef.current = 1;
      dispatch({ type: "SELECT_RECORD", payload: recordId });
    },
    [dispatch],
  );

  const rowProps = useMemo<VirtualRowData>(
    () => ({
      recordIdForIndex,
      recordForId,
      repoRoot: bootstrap?.repo_root ?? null,
      selectedRecordId,
      multiSelectActive,
      multiSelection,
      onSelect,
      onMultiSelectToggle,
    }),
    [
      bootstrap?.repo_root,
      multiSelectActive,
      multiSelection,
      onMultiSelectToggle,
      onSelect,
      recordIdForIndex,
      recordForId,
      selectedRecordId,
    ],
  );

  const handleRowsRendered = useCallback(
    (_visibleRows: { startIndex: number; stopIndex: number }, allRows: { startIndex: number; stopIndex: number }) => {
      if (sourceFilter !== "dataset" || rowCount === 0 || allRows.stopIndex < allRows.startIndex) {
        return;
      }

      const startPage = Math.floor(allRows.startIndex / DATASET_PAGE_SIZE);
      const endPage = Math.floor(allRows.stopIndex / DATASET_PAGE_SIZE);
      for (let pageIndex = startPage; pageIndex <= endPage; pageIndex += 1) {
        loadDatasetPage(pageIndex);
      }
    },
    [loadDatasetPage, rowCount, sourceFilter],
  );

  useEffect(() => {
    const isTypingTarget = (target: EventTarget | null): boolean => {
      if (!(target instanceof HTMLElement)) {
        return false;
      }

      if (target.closest('[data-record-select-trigger="true"]')) {
        return false;
      }

      return Boolean(
        target.closest(
          'input, textarea, select, button, [contenteditable="true"], [role="combobox"], [role="listbox"], [role="menu"], [role="dialog"]',
        ),
      );
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if ((event.metaKey || event.ctrlKey) && event.key === "a" && !event.shiftKey && !event.altKey) {
        if (!isTypingTarget(event.target) && rowCount > 0) {
          event.preventDefault();
          if (sourceFilter === "dataset") {
            void queryClient
              .fetchQuery(browseRecordIdsQueryOptions(datasetBrowseParams))
              .then((response) => {
                dispatch({ type: "SET_MULTI_SELECT_ALL", payload: response.record_ids });
              });
          } else {
            dispatch({ type: "SET_MULTI_SELECT_ALL", payload: visibleIds });
          }
          return;
        }
      }

      if (event.key === "Escape" && multiSelectActive) {
        event.preventDefault();
        dispatch({ type: "CLEAR_MULTI_SELECT" });
        return;
      }

      if (event.defaultPrevented || event.altKey || event.ctrlKey || event.metaKey || event.shiftKey) {
        return;
      }
      if (event.key !== "ArrowDown" && event.key !== "ArrowUp") {
        return;
      }
      if (isTypingTarget(event.target)) {
        return;
      }
      if (rowCount === 0) {
        return;
      }

      event.preventDefault();

      const currentIndex = selectedRecordId ? (recordIndexById.get(selectedRecordId) ?? -1) : -1;
      const fallbackIndex =
        previousSelectedVisibleIndexRef.current == null
          ? null
          : event.key === "ArrowDown"
            ? Math.min(previousSelectedVisibleIndexRef.current, rowCount - 1)
            : Math.max(previousSelectedVisibleIndexRef.current - 1, 0);
      const nextIndex =
        event.key === "ArrowDown"
          ? currentIndex >= 0
            ? (currentIndex + 1) % rowCount
            : fallbackIndex ?? 0
          : currentIndex >= 0
            ? (currentIndex - 1 + rowCount) % rowCount
            : fallbackIndex ?? rowCount - 1;
      selectionAdvanceDirectionRef.current = event.key === "ArrowUp" ? -1 : 1;

      const nextRecordId = recordIdForIndex(nextIndex);
      if (sourceFilter === "dataset") {
        loadDatasetPage(Math.floor(nextIndex / DATASET_PAGE_SIZE));
      }
      if (!nextRecordId) {
        return;
      }
      dispatch({ type: "SELECT_RECORD", payload: nextRecordId });
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [
    datasetBrowseParams,
    dispatch,
    loadDatasetPage,
    multiSelectActive,
    queryClient,
    recordIdForIndex,
    recordIndexById,
    rowCount,
    selectedRecordId,
    sourceFilter,
    visibleIds,
  ]);

  if (!bootstrap) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">Loading…</p>
      </div>
    );
  }

  if (sourceFilter === "workbench" && deferredSearchQuery && searchPending && searchedRecords === null) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">Searching…</p>
      </div>
    );
  }

  if (sourceFilter === "dataset" && datasetLoading && rowCount === 0) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">
          {deferredSearchQuery ? "Searching…" : "Loading records…"}
        </p>
      </div>
    );
  }

  if (rowCount === 0) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">
          {searchQuery ? "No matching records" : "No records found"}
        </p>
      </div>
    );
  }

  return (
    <div className="min-h-0 flex-1">
      <AutoSizer
        renderProp={({ height, width }) => {
          if (!height || !width) {
            return null;
          }

          return (
            <List
              className="outline-none"
              defaultHeight={DEFAULT_LIST_HEIGHT_PX}
              listRef={listRef}
              onRowsRendered={handleRowsRendered}
              overscanCount={VIRTUAL_OVERSCAN}
              rowComponent={VirtualRecordRow}
              rowCount={rowCount}
              rowHeight={RECORD_ROW_HEIGHT_PX}
              rowProps={rowProps}
              style={{ height, width }}
            />
          );
        }}
        style={{ height: "100%", width: "100%" }}
      />
    </div>
  );
}
