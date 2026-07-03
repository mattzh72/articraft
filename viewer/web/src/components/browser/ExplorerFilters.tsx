import { type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import type { RatingFilterValue } from "@/lib/types";
import { Button } from "@/components/ui/button";

const ALL_VALUE = "__all__";
const RATING_VALUES: RatingFilterValue[] = ["5", "4", "3", "2", "1", "unrated"];
const FILTER_SELECT_CLASS =
  "h-8 w-full rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] px-2.5 text-[11px] text-[var(--text-primary)] outline-none transition-colors focus-visible:border-[var(--accent)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)] disabled:cursor-not-allowed disabled:opacity-40";

function uniqueSorted(values: Array<string | null | undefined>): string[] {
  return Array.from(
    new Set(values.filter((value): value is string => Boolean(value && value.trim()))),
  ).sort((left, right) => left.localeCompare(right));
}

export function ExplorerFilters(): JSX.Element {
  const {
    bootstrap,
    modelFilter,
    sdkFilter,
    agentHarnessFilters,
    categoryFilters,
    ratingFilter,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const records = bootstrap?.library_records ?? [];
  const models = uniqueSorted(records.map((record) => record.model_id));
  const sdks = uniqueSorted(records.map((record) => record.sdk_package));
  const agentHarnesses = uniqueSorted(records.map((record) => record.agent_harness));
  const categories = uniqueSorted(records.map((record) => record.category_slug));

  return (
    <div className="space-y-2">
      <div className="grid grid-cols-2 gap-2">
        <select
          value={modelFilter ?? ALL_VALUE}
          className={FILTER_SELECT_CLASS}
          aria-label="Model filter"
          onChange={(event) =>
            dispatch({
              type: "SET_MODEL_FILTER",
              payload: event.target.value === ALL_VALUE ? null : event.target.value,
            })
          }
        >
          <option value={ALL_VALUE}>All models</option>
          {models.map((model) => (
            <option key={model} value={model}>
              {model}
            </option>
          ))}
        </select>

        <select
          value={sdkFilter ?? ALL_VALUE}
          className={FILTER_SELECT_CLASS}
          aria-label="SDK filter"
          onChange={(event) =>
            dispatch({
              type: "SET_SDK_FILTER",
              payload: event.target.value === ALL_VALUE ? null : event.target.value,
            })
          }
        >
          <option value={ALL_VALUE}>All SDKs</option>
          {sdks.map((sdk) => (
            <option key={sdk} value={sdk}>
              {sdk}
            </option>
          ))}
        </select>

        <select
          value={agentHarnessFilters[0] ?? ALL_VALUE}
          className={FILTER_SELECT_CLASS}
          aria-label="Agent filter"
          onChange={(event) =>
            dispatch({
              type: "SET_AGENT_HARNESS_FILTERS",
              payload: event.target.value === ALL_VALUE ? [] : [event.target.value],
            })
          }
        >
          <option value={ALL_VALUE}>All agents</option>
          {agentHarnesses.map((agent) => (
            <option key={agent} value={agent}>
              {agent}
            </option>
          ))}
        </select>

        <select
          value={categoryFilters[0] ?? ALL_VALUE}
          className={FILTER_SELECT_CLASS}
          aria-label="Category filter"
          onChange={(event) =>
            dispatch({
              type: "SET_CATEGORY_FILTERS",
              payload: event.target.value === ALL_VALUE ? [] : [event.target.value],
            })
          }
        >
          <option value={ALL_VALUE}>All categories</option>
          {categories.map((category) => (
            <option key={category} value={category}>
              {category}
            </option>
          ))}
        </select>
      </div>

      <div className="flex flex-wrap gap-1">
        {RATING_VALUES.map((value) => {
          const active = ratingFilter.includes(value);
          return (
            <Button
              key={value}
              type="button"
              variant={active ? "default" : "outline"}
              size="sm"
              className="h-6 px-2 text-[10px]"
              onClick={() =>
                dispatch({
                  type: "SET_RATING_FILTER",
                  payload: active
                    ? ratingFilter.filter((item) => item !== value)
                    : [...ratingFilter, value],
                })
              }
            >
              {value === "unrated" ? "Unrated" : `${value} star`}
            </Button>
          );
        })}
      </div>
    </div>
  );
}
