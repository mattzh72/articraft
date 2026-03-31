import { useCallback, useDeferredValue, useEffect, useMemo, useRef, useState, type JSX } from "react";

import { searchRecords } from "@/lib/api";
import type { CostFilter, RatingFilter, RecordSummary, TimeFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { ScrollArea } from "@/components/ui/scroll-area";
import { RecordListItem } from "@/components/browser/RecordListItem";

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

interface RecordListProps {
  onVisibleIdsChange?: (ids: string[]) => void;
  onCountsChange?: (counts: { visible: number; total: number }) => void;
}

export function RecordList({ onVisibleIdsChange, onCountsChange }: RecordListProps): JSX.Element {
  const {
    bootstrap,
    searchQuery,
    sourceFilter,
    timeFilter,
    modelFilter,
    sdkFilter,
    authorFilters,
    categoryFilters,
    costFilter,
    ratingFilter,
    selectedRunId,
    selectedRecordId,
    multiSelection,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const scrollAreaRef = useRef<HTMLDivElement | null>(null);
  const previousSelectedRecordIdRef = useRef<string | null>(selectedRecordId);
  const followSelectionRef = useRef(true);
  const deferredSearchQuery = useDeferredValue(searchQuery.trim());
  const [searchedRecords, setSearchedRecords] = useState<RecordSummary[] | null>(null);
  const [searchPending, setSearchPending] = useState(false);

  const sourceRecords = useMemo(() => {
    if (!bootstrap) return [];

    const seen = new Map<string, RecordSummary>();
    const addRecord = (recordId: string, record: RecordSummary | null) => {
      if (!record || seen.has(recordId)) {
        return;
      }
      seen.set(recordId, record);
    };

    if (sourceFilter === "workbench") {
      for (const entry of bootstrap.workbench_entries) {
        addRecord(entry.record_id, entry.record);
      }
    }

    if (sourceFilter === "dataset") {
      for (const entry of bootstrap.dataset_entries) {
        addRecord(entry.record_id, entry.record);
      }
    }

    return Array.from(seen.values());
  }, [bootstrap, sourceFilter]);
  const sourceRecordById = useMemo(
    () => new Map(sourceRecords.map((record) => [record.record_id, record])),
    [sourceRecords],
  );

  useEffect(() => {
    if (!deferredSearchQuery) {
      setSearchedRecords(null);
      setSearchPending(false);
      return;
    }

    let cancelled = false;
    setSearchedRecords(null);
    setSearchPending(true);

    searchRecords({
      query: deferredSearchQuery,
      source: sourceFilter,
      runId: selectedRunId,
      timeFilter,
      modelFilter,
      authorFilters: sourceFilter === "dataset" ? authorFilters : [],
      categoryFilters: sourceFilter === "dataset" ? categoryFilters : [],
      costFilter,
      ratingFilter,
      limit: 200,
    })
      .then((results) => {
        if (!cancelled) {
          setSearchedRecords(results);
          setSearchPending(false);
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
  }, [authorFilters, categoryFilters, costFilter, deferredSearchQuery, modelFilter, ratingFilter, selectedRunId, sourceFilter, timeFilter]);

  const records = useMemo(() => {
    if (!bootstrap) return [];

    let list = deferredSearchQuery
      ? (searchedRecords ?? []).map((record) => sourceRecordById.get(record.record_id) ?? record)
      : sourceRecords;

    if (!deferredSearchQuery && selectedRunId) {
      list = list.filter((r) => r.run_id === selectedRunId);
    }

    if (sourceFilter === "dataset" && categoryFilters.length > 0) {
      const selectedCategories = new Set(categoryFilters);
      list = list.filter((record) => record.category_slug && selectedCategories.has(record.category_slug));
    }

    if (sourceFilter === "dataset" && authorFilters.length > 0) {
      const selectedAuthors = new Set(authorFilters);
      list = list.filter((record) => record.author && selectedAuthors.has(record.author));
    }

    if (!deferredSearchQuery) {
      list = list.filter((record) => withinTimeFilter(record.created_at, timeFilter));
      list = list.filter((record) => withinCostFilter(record.total_cost_usd, costFilter));
      list = list.filter((record) => withinRatingFilter(record.effective_rating, ratingFilter));
      if (modelFilter) {
        list = list.filter((record) => record.model_id === modelFilter);
      }
      if (sdkFilter) {
        list = list.filter((record) => record.sdk_package === sdkFilter);
      }
    }

    if (!deferredSearchQuery) {
      list.sort((a, b) => {
        const dateA = recordSortTimestamp(a);
        const dateB = recordSortTimestamp(b);
        return dateB - dateA;
      });
    }

    return list;
  }, [
    bootstrap,
    authorFilters,
    categoryFilters,
    costFilter,
    deferredSearchQuery,
    modelFilter,
    sdkFilter,
    ratingFilter,
    searchedRecords,
    selectedRunId,
    sourceRecordById,
    sourceRecords,
    sourceFilter,
    timeFilter,
  ]);

  useEffect(() => {
    onVisibleIdsChange?.(records.map((r) => r.record_id));
  }, [records, onVisibleIdsChange]);

  useEffect(() => {
    if (!bootstrap) {
      onCountsChange?.({ visible: 0, total: 0 });
      return;
    }

    onCountsChange?.({ visible: records.length, total: sourceRecords.length });
  }, [bootstrap, onCountsChange, records.length, sourceRecords.length]);

  useEffect(() => {
    if (selectedRecordId !== previousSelectedRecordIdRef.current) {
      followSelectionRef.current = true;
      previousSelectedRecordIdRef.current = selectedRecordId;
    }

    if (!followSelectionRef.current || !selectedRecordId) {
      return;
    }

    const selectedItem = selectedRecordId
      ? scrollAreaRef.current?.querySelector<HTMLElement>(
          `[data-record-list-item="${CSS.escape(selectedRecordId)}"]`,
        ) ?? null
      : null;
    selectedItem?.scrollIntoView({ block: "nearest" });
  }, [records, selectedRecordId]);

  useEffect(() => {
    const root = scrollAreaRef.current;
    if (!root) {
      return;
    }

    const viewport = root.querySelector<HTMLElement>('[data-slot="scroll-area-viewport"]');
    if (!viewport) {
      return;
    }

    const disableAutoFollow = () => {
      followSelectionRef.current = false;
    };

    const handleWheel = () => {
      disableAutoFollow();
    };

    const handleTouchMove = () => {
      disableAutoFollow();
    };

    const handlePointerDown = (event: PointerEvent) => {
      if (!(event.target instanceof HTMLElement)) {
        return;
      }
      if (event.target.closest('[data-slot="scroll-area-scrollbar"]')) {
        disableAutoFollow();
      }
    };

    viewport.addEventListener("wheel", handleWheel, { passive: true });
    viewport.addEventListener("touchmove", handleTouchMove, { passive: true });
    root.addEventListener("pointerdown", handlePointerDown);

    return () => {
      viewport.removeEventListener("wheel", handleWheel);
      viewport.removeEventListener("touchmove", handleTouchMove);
      root.removeEventListener("pointerdown", handlePointerDown);
    };
  }, []);

  const multiSelectActive = multiSelection.size > 0;

  const visibleIds = useMemo(() => records.map((r) => r.record_id), [records]);

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
      // Cmd/Ctrl+A: select all visible records
      if ((event.metaKey || event.ctrlKey) && event.key === "a" && !event.shiftKey && !event.altKey) {
        if (!isTypingTarget(event.target) && records.length > 0) {
          event.preventDefault();
          dispatch({ type: "SET_MULTI_SELECT_ALL", payload: records.map((r) => r.record_id) });
          return;
        }
      }

      // Escape: clear multi-select
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
      if (records.length === 0) {
        return;
      }

      event.preventDefault();

      const currentIndex = selectedRecordId
        ? records.findIndex((record) => record.record_id === selectedRecordId)
        : -1;

      const nextIndex =
        event.key === "ArrowDown"
          ? currentIndex >= 0
            ? (currentIndex + 1) % records.length
            : 0
          : currentIndex >= 0
            ? (currentIndex - 1 + records.length) % records.length
            : records.length - 1;

      dispatch({ type: "SELECT_RECORD", payload: records[nextIndex]?.record_id ?? null });
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [dispatch, multiSelectActive, records, selectedRecordId]);

  if (!bootstrap) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">Loading…</p>
      </div>
    );
  }

  if (deferredSearchQuery && searchPending && searchedRecords === null) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">Searching…</p>
      </div>
    );
  }

  if (records.length === 0) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">
          {searchQuery ? "No matching records" : "No records found"}
        </p>
      </div>
    );
  }

  return (
    <ScrollArea ref={scrollAreaRef} className="min-h-0 flex-1">
      <div className="py-0.5">
        {records.map((record) => (
          <RecordListItem
            key={record.record_id}
            record={record}
            multiSelectActive={multiSelectActive}
            isMultiSelected={multiSelection.has(record.record_id)}
            onMultiSelectToggle={onMultiSelectToggle}
          />
        ))}
      </div>
    </ScrollArea>
  );
}
