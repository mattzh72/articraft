import { type JSX } from "react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import type { RatingFilterValue } from "@/lib/types";
import { Button } from "@/components/ui/button";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";

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
        <Select
          value={modelFilter ?? ALL_VALUE}
          onValueChange={(value) =>
            dispatch({ type: "SET_MODEL_FILTER", payload: value === ALL_VALUE ? null : value })
          }
        >
          <SelectTrigger className="h-8 text-[11px]">
            <SelectValue placeholder="Model" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value={ALL_VALUE}>All models</SelectItem>
            {models.map((model) => (
              <SelectItem key={model} value={model}>
                {model}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        <Select
          value={sdkFilter ?? ALL_VALUE}
          onValueChange={(value) =>
            dispatch({ type: "SET_SDK_FILTER", payload: value === ALL_VALUE ? null : value })
          }
        >
          <SelectTrigger className="h-8 text-[11px]">
            <SelectValue placeholder="SDK" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value={ALL_VALUE}>All SDKs</SelectItem>
            {sdks.map((sdk) => (
              <SelectItem key={sdk} value={sdk}>
                {sdk}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        <Select
          value={agentHarnessFilters[0] ?? ALL_VALUE}
          onValueChange={(value) =>
            dispatch({
              type: "SET_AGENT_HARNESS_FILTERS",
              payload: value === ALL_VALUE ? [] : [value],
            })
          }
        >
          <SelectTrigger className="h-8 text-[11px]">
            <SelectValue placeholder="Agent" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value={ALL_VALUE}>All agents</SelectItem>
            {agentHarnesses.map((agent) => (
              <SelectItem key={agent} value={agent}>
                {agent}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>

        <Select
          value={categoryFilters[0] ?? ALL_VALUE}
          onValueChange={(value) =>
            dispatch({
              type: "SET_CATEGORY_FILTERS",
              payload: value === ALL_VALUE ? [] : [value],
            })
          }
        >
          <SelectTrigger className="h-8 text-[11px]">
            <SelectValue placeholder="Category" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value={ALL_VALUE}>All categories</SelectItem>
            {categories.map((category) => (
              <SelectItem key={category} value={category}>
                {category}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
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
