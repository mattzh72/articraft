import { type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import type { RatingFilterValue } from "@/lib/types";
import { Button } from "@/components/ui/button";
import { NativeSelect } from "@/components/ui/native-select";

const ALL_VALUE = "__all__";
const RATING_VALUES: RatingFilterValue[] = ["5", "4", "3", "2", "1", "unrated"];

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
        <NativeSelect
          aria-label="Model filter"
          className="h-8 text-[11px]"
          value={modelFilter ?? ALL_VALUE}
          onChange={(event) => {
            const value = event.currentTarget.value;
            dispatch({ type: "SET_MODEL_FILTER", payload: value === ALL_VALUE ? null : value });
          }}
        >
          <option value={ALL_VALUE}>All models</option>
          {models.map((model) => (
            <option key={model} value={model}>
              {model}
            </option>
          ))}
        </NativeSelect>

        <NativeSelect
          aria-label="SDK filter"
          className="h-8 text-[11px]"
          value={sdkFilter ?? ALL_VALUE}
          onChange={(event) => {
            const value = event.currentTarget.value;
            dispatch({ type: "SET_SDK_FILTER", payload: value === ALL_VALUE ? null : value });
          }}
        >
          <option value={ALL_VALUE}>All SDKs</option>
          {sdks.map((sdk) => (
            <option key={sdk} value={sdk}>
              {sdk}
            </option>
          ))}
        </NativeSelect>

        <NativeSelect
          aria-label="Agent filter"
          className="h-8 text-[11px]"
          value={agentHarnessFilters[0] ?? ALL_VALUE}
          onChange={(event) => {
            const value = event.currentTarget.value;
            dispatch({
              type: "SET_AGENT_HARNESS_FILTERS",
              payload: value === ALL_VALUE ? [] : [value],
            });
          }}
        >
          <option value={ALL_VALUE}>All agents</option>
          {agentHarnesses.map((agent) => (
            <option key={agent} value={agent}>
              {agent}
            </option>
          ))}
        </NativeSelect>

        <NativeSelect
          aria-label="Category filter"
          className="h-8 text-[11px]"
          value={categoryFilters[0] ?? ALL_VALUE}
          onChange={(event) => {
            const value = event.currentTarget.value;
            dispatch({
              type: "SET_CATEGORY_FILTERS",
              payload: value === ALL_VALUE ? [] : [value],
            });
          }}
        >
          <option value={ALL_VALUE}>All categories</option>
          {categories.map((category) => (
            <option key={category} value={category}>
              {category}
            </option>
          ))}
        </NativeSelect>
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
