import { useEffect, useMemo, type JSX } from "react";

import { RecordListItem } from "@/components/browser/RecordListItem";
import { ScrollArea } from "@/components/ui/scroll-area";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import type { RecordSummary } from "@/lib/types";

type RecordListProps = {
  onVisibleIdsChange?: (ids: string[]) => void;
  onCountsChange?: (counts: { visible: number; total: number }) => void;
};

function matchesQuery(record: RecordSummary, query: string): boolean {
  if (!query.trim()) {
    return true;
  }
  const haystack = [
    record.record_id,
    record.title,
    record.prompt_preview,
    record.category_slug,
    record.category_title,
    record.label,
    ...record.tags,
  ]
    .filter((value): value is string => Boolean(value))
    .join(" ")
    .toLowerCase();
  return query
    .toLowerCase()
    .split(/\s+/)
    .filter(Boolean)
    .every((token) => haystack.includes(token));
}

function withinRating(record: RecordSummary, filters: string[]): boolean {
  if (filters.length === 0) {
    return true;
  }
  const rating = record.effective_rating;
  if (rating == null) {
    return filters.includes("unrated");
  }
  const bucket = String(Math.max(1, Math.min(5, Math.ceil(rating))));
  return filters.includes(bucket);
}

function recordSortTimestamp(record: RecordSummary): number {
  const timestamp = record.updated_at ?? record.created_at;
  return timestamp ? new Date(timestamp).getTime() : 0;
}

export function RecordList({
  onVisibleIdsChange,
  onCountsChange,
}: RecordListProps): JSX.Element {
  const {
    bootstrap,
    searchQuery,
    selectedRecordId,
    multiSelection,
    modelFilter,
    sdkFilter,
    agentHarnessFilters,
    categoryFilters,
    ratingFilter,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const records = useMemo(() => bootstrap?.library_records ?? [], [bootstrap?.library_records]);
  const visibleRecords = useMemo(() => {
    return records
      .filter((record) => matchesQuery(record, searchQuery))
      .filter((record) => !modelFilter || record.model_id === modelFilter)
      .filter((record) => !sdkFilter || record.sdk_package === sdkFilter)
      .filter(
        (record) =>
          agentHarnessFilters.length === 0 || agentHarnessFilters.includes(record.agent_harness),
      )
      .filter(
        (record) =>
          categoryFilters.length === 0
          || (record.category_slug != null && categoryFilters.includes(record.category_slug)),
      )
      .filter((record) => withinRating(record, ratingFilter))
      .sort((left, right) => recordSortTimestamp(right) - recordSortTimestamp(left));
  }, [
    agentHarnessFilters,
    categoryFilters,
    modelFilter,
    ratingFilter,
    records,
    sdkFilter,
    searchQuery,
  ]);
  const visibleIds = useMemo(
    () => visibleRecords.map((record) => record.record_id),
    [visibleRecords],
  );

  useEffect(() => {
    onVisibleIdsChange?.(visibleIds);
    onCountsChange?.({ visible: visibleIds.length, total: records.length });
  }, [onCountsChange, onVisibleIdsChange, records.length, visibleIds]);

  const handleSelect = (recordId: string) => {
    dispatch({ type: "SELECT_RECORD", payload: recordId });
  };

  const handleMultiSelectToggle = (recordId: string, shiftKey: boolean) => {
    if (shiftKey) {
      dispatch({ type: "RANGE_MULTI_SELECT", payload: { targetId: recordId, visibleIds } });
      return;
    }
    dispatch({ type: "TOGGLE_MULTI_SELECT", payload: recordId });
  };

  if (visibleRecords.length === 0) {
    return (
      <div className="flex flex-1 items-center justify-center px-4">
        <p className="text-[11px] text-[var(--text-quaternary)]">No records</p>
      </div>
    );
  }

  return (
    <ScrollArea className="min-h-0 flex-1">
      <div className="space-y-1 px-1.5 py-2">
        {visibleRecords.map((record) => (
          <RecordListItem
            key={record.record_id}
            recordId={record.record_id}
            record={record}
            dataRoot={bootstrap?.data_root ?? null}
            isSelected={selectedRecordId === record.record_id}
            multiSelectActive={multiSelection.size > 0}
            isMultiSelected={multiSelection.has(record.record_id)}
            onSelect={handleSelect}
            onMultiSelectToggle={handleMultiSelectToggle}
          />
        ))}
      </div>
    </ScrollArea>
  );
}
