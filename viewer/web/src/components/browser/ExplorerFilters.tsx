import { useMemo, type JSX } from "react";

import type { CostFilter, RatingFilter, RecordSummary, TimeFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Button } from "@/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

const timeOptions: Array<{ value: TimeFilter; label: string }> = [
  { value: "any", label: "Any time" },
  { value: "24h", label: "Last 24h" },
  { value: "7d", label: "Last 7 days" },
  { value: "30d", label: "Last 30 days" },
  { value: "90d", label: "Last 90 days" },
];

const costOptions: Array<{ value: CostFilter; label: string }> = [
  { value: "any", label: "Any cost" },
  { value: "lt_0_01", label: "Up to $0.01" },
  { value: "lt_0_05", label: "Up to $0.05" },
  { value: "lt_0_10", label: "Up to $0.10" },
  { value: "gte_0_10", label: "$0.10+" },
  { value: "missing", label: "Missing cost" },
];

const ratingOptions: Array<{ value: RatingFilter; label: string }> = [
  { value: "any", label: "Any rating" },
  { value: "5", label: "5 stars" },
  { value: "4", label: "4 stars" },
  { value: "3", label: "3 stars" },
  { value: "2", label: "2 stars" },
  { value: "1", label: "1 star" },
  { value: "unrated", label: "Unrated" },
];

function uniqueRecords(records: Array<RecordSummary | null>): RecordSummary[] {
  const seen = new Map<string, RecordSummary>();

  for (const record of records) {
    if (!record) continue;
    if (!seen.has(record.record_id)) {
      seen.set(record.record_id, record);
    }
  }

  return Array.from(seen.values());
}

export function ExplorerFilters(): JSX.Element | null {
  const { bootstrap, sourceFilter, timeFilter, modelFilter, costFilter, ratingFilter } = useViewer();
  const dispatch = useViewerDispatch();

  const availableModels = useMemo(() => {
    if (!bootstrap) return [];

    const records = uniqueRecords(
      sourceFilter === "dataset"
        ? bootstrap.dataset_entries.map((entry) => entry.record)
        : bootstrap.workbench_entries.map((entry) => entry.record),
    );

    return Array.from(
      new Set(records.map((record) => record.model_id).filter((value): value is string => Boolean(value))),
    ).sort((left, right) => left.localeCompare(right));
  }, [bootstrap, sourceFilter]);

  if (!bootstrap) return null;

  const filtersActive = timeFilter !== "any" || costFilter !== "any" || ratingFilter !== "any" || modelFilter !== null;

  return (
    <div className="space-y-2">
      <div className="flex items-center justify-between">
        <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[var(--text-tertiary)]">Filters</span>
        {filtersActive ? (
          <Button
            type="button"
            variant="ghost"
            size="sm"
            onClick={() => {
              dispatch({ type: "SET_TIME_FILTER", payload: "any" });
              dispatch({ type: "SET_COST_FILTER", payload: "any" });
              dispatch({ type: "SET_RATING_FILTER", payload: "any" });
              dispatch({ type: "SET_MODEL_FILTER", payload: null });
            }}
            className="h-5 px-1.5 text-[10px] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
          >
            Clear
          </Button>
        ) : null}
      </div>

      <div className="flex flex-wrap gap-1.5">
        <Select value={timeFilter} onValueChange={(value) => dispatch({ type: "SET_TIME_FILTER", payload: value as TimeFilter })}>
          <SelectTrigger size="sm" className="h-7 w-auto min-w-0 rounded-full border-[var(--border-default)] bg-[var(--surface-1)] px-2.5 text-[10px]">
            <SelectValue placeholder="Any time" />
          </SelectTrigger>
          <SelectContent>
            {timeOptions.map((option) => (
              <SelectItem key={option.value} value={option.value}>
                {option.label}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        <Select value={costFilter} onValueChange={(value) => dispatch({ type: "SET_COST_FILTER", payload: value as CostFilter })}>
          <SelectTrigger size="sm" className="h-7 w-auto min-w-0 rounded-full border-[var(--border-default)] bg-[var(--surface-1)] px-2.5 text-[10px]">
            <SelectValue placeholder="Any cost" />
          </SelectTrigger>
          <SelectContent>
            {costOptions.map((option) => (
              <SelectItem key={option.value} value={option.value}>
                {option.label}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        <Select value={ratingFilter} onValueChange={(value) => dispatch({ type: "SET_RATING_FILTER", payload: value as RatingFilter })}>
          <SelectTrigger size="sm" className="h-7 w-auto min-w-0 rounded-full border-[var(--border-default)] bg-[var(--surface-1)] px-2.5 text-[10px]">
            <SelectValue placeholder="Any rating" />
          </SelectTrigger>
          <SelectContent>
            {ratingOptions.map((option) => (
              <SelectItem key={option.value} value={option.value}>
                {option.label}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        <Select
          value={modelFilter ?? "all"}
          onValueChange={(value) =>
            dispatch({
              type: "SET_MODEL_FILTER",
              payload: value === "all" ? null : value,
            })
          }
        >
          <SelectTrigger size="sm" className="h-7 w-auto min-w-0 max-w-full rounded-full border-[var(--border-default)] bg-[var(--surface-1)] px-2.5 text-[10px]">
            <SelectValue placeholder="All models" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="all">All models</SelectItem>
            {availableModels.map((model) => (
              <SelectItem key={model} value={model}>
                <span className="truncate font-mono text-[11px]">{model}</span>
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
      </div>
    </div>
  );
}
