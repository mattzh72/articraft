import { useDeferredValue, useEffect, useMemo, useRef, useState, type JSX } from "react";

import { searchRecords } from "@/lib/api";
import type { CostFilter, RatingFilter, RecordSummary, TimeFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { ScrollArea } from "@/components/ui/scroll-area";
import { RecordListItem } from "@/components/browser/RecordListItem";

function withinTimeFilter(createdAt: string | null, filter: TimeFilter): boolean {
  if (filter === "any") return true;
  if (!createdAt) return false;

  const createdAtMs = new Date(createdAt).getTime();
  if (Number.isNaN(createdAtMs)) return false;

  const now = Date.now();
  const thresholds: Record<Exclude<TimeFilter, "any">, number> = {
    "24h": 24 * 60 * 60 * 1000,
    "7d": 7 * 24 * 60 * 60 * 1000,
    "30d": 30 * 24 * 60 * 60 * 1000,
    "90d": 90 * 24 * 60 * 60 * 1000,
  };

  return now - createdAtMs <= thresholds[filter];
}

function withinCostFilter(totalCostUsd: number | null, filter: CostFilter): boolean {
  if (filter === "any") return true;
  if (filter === "missing") return totalCostUsd == null;
  if (totalCostUsd == null) return false;
  if (filter === "lt_0_01") return totalCostUsd <= 0.01;
  if (filter === "lt_0_05") return totalCostUsd <= 0.05;
  if (filter === "lt_0_10") return totalCostUsd <= 0.1;
  return totalCostUsd >= 0.1;
}

function withinRatingFilter(rating: number | null, filter: RatingFilter): boolean {
  if (filter === "any") return true;
  if (filter === "unrated") return rating == null;
  return rating === Number(filter);
}

export function RecordList(): JSX.Element {
  const {
    bootstrap,
    searchQuery,
    sourceFilter,
    timeFilter,
    modelFilter,
    costFilter,
    ratingFilter,
    selectedRunId,
    selectedRecordId,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const scrollAreaRef = useRef<HTMLDivElement | null>(null);
  const previousSelectedRecordIdRef = useRef<string | null>(selectedRecordId);
  const followSelectionRef = useRef(true);
  const deferredSearchQuery = useDeferredValue(searchQuery.trim());
  const [searchedRecords, setSearchedRecords] = useState<RecordSummary[] | null>(null);
  const [searchPending, setSearchPending] = useState(false);

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
  }, [costFilter, deferredSearchQuery, modelFilter, ratingFilter, selectedRunId, sourceFilter, timeFilter]);

  const records = useMemo(() => {
    if (!bootstrap) return [];

    const seen = new Map<string, RecordSummary>();

    const addRecord = (recordId: string, record: RecordSummary | null) => {
      if (!record) return;
      if (!seen.has(recordId)) {
        seen.set(recordId, record);
      }
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

    let list = deferredSearchQuery ? searchedRecords ?? [] : Array.from(seen.values());

    if (!deferredSearchQuery && selectedRunId) {
      list = list.filter((r) => r.run_id === selectedRunId);
    }

    if (!deferredSearchQuery) {
      list = list.filter((record) => withinTimeFilter(record.created_at, timeFilter));
      list = list.filter((record) => withinCostFilter(record.total_cost_usd, costFilter));
      list = list.filter((record) => withinRatingFilter(record.rating, ratingFilter));
      if (modelFilter) {
        list = list.filter((record) => record.model_id === modelFilter);
      }
    }

    if (!deferredSearchQuery) {
      list.sort((a, b) => {
        const dateA = a.created_at ? new Date(a.created_at).getTime() : 0;
        const dateB = b.created_at ? new Date(b.created_at).getTime() : 0;
        return dateB - dateA;
      });
    }

    return list;
  }, [
    bootstrap,
    costFilter,
    deferredSearchQuery,
    modelFilter,
    ratingFilter,
    searchedRecords,
    selectedRunId,
    sourceFilter,
    timeFilter,
  ]);

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
  }, [dispatch, records, selectedRecordId]);

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
          <RecordListItem key={record.record_id} record={record} />
        ))}
      </div>
    </ScrollArea>
  );
}
