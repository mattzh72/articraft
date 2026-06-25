import {
  createContext,
  useContext,
  useEffect,
  useReducer,
  type Dispatch,
  type JSX,
  type ReactNode,
} from "react";
import { useQuery } from "@tanstack/react-query";

import { HttpError } from "@/lib/api";
import { normalizeAgentHarnessFilters } from "@/lib/agent-harness";
import {
  bootstrapQueryOptions,
  recordSummaryQueryOptions,
  stagingEntriesQueryOptions,
} from "@/lib/viewer-queries";
import type {
  BrowserTab,
  CostFilter,
  InspectorTab,
  RatingFilter,
  RatingFilterValue,
  RecordSummary,
  RunSummary,
  TimeFilterPoint,
  ViewerAction,
  ViewerBootstrap,
  ViewerSelection,
  ViewerState,
} from "@/lib/types";

const URL_QUERY_PARAMS = {
  record: "record",
  staging: "staging",
  browser: "browser",
  tab: "tab",
  search: "q",
  timeFrom: "time_from",
  timeTo: "time_to",
  model: "model",
  sdk: "sdk",
  agentHarness: "agent_harness",
  author: "author",
  category: "category",
  costMin: "cost_min",
  costMax: "cost_max",
  rating: "rating",
  secondaryRating: "secondary_rating",
  run: "run",
} as const;

const INSPECTOR_TABS = ["inspect", "render", "code", "metadata"] as const satisfies readonly InspectorTab[];
const BROWSER_TABS = ["library", "staging"] as const satisfies readonly BrowserTab[];
const TIME_FILTER_POINTS = new Set<string>(["1y", "180d", "90d", "60d", "30d", "14d", "7d", "3d", "24h", "12h", "6h", "1h"]);
const RATING_FILTERS = ["1", "2", "3", "4", "5", "unrated"] as const satisfies readonly RatingFilterValue[];
const STAGING_POLL_INTERVAL_MS = 3000;

function computeEffectiveRating(
  rating: number | null | undefined,
  secondaryRating: number | null | undefined,
): number | null {
  const values = [rating, secondaryRating].filter((value): value is number => value != null);
  if (values.length === 0) {
    return null;
  }
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

type ViewerUrlState = Pick<
  ViewerState,
  | "selection"
  | "selectedRecordId"
  | "selectedInspectorTab"
  | "searchQuery"
  | "browserTab"
  | "sourceFilter"
  | "timeFilter"
  | "modelFilter"
  | "sdkFilter"
  | "agentHarnessFilters"
  | "authorFilters"
  | "categoryFilters"
  | "costFilter"
  | "ratingFilter"
  | "secondaryRatingFilter"
  | "selectedRunId"
>;

const defaultViewerUrlState: ViewerUrlState = {
  selection: null,
  selectedRecordId: null,
  selectedInspectorTab: "inspect",
  searchQuery: "",
  browserTab: "library",
  sourceFilter: "library",
  timeFilter: { oldest: null, newest: null },
  modelFilter: null,
  sdkFilter: null,
  agentHarnessFilters: [],
  authorFilters: [],
  categoryFilters: [],
  costFilter: { min: null, max: null },
  ratingFilter: [],
  secondaryRatingFilter: [],
  selectedRunId: null,
};

function selectionToSelectedRecordId(selection: ViewerSelection | null): string | null {
  return selection?.kind === "record" ? selection.recordId : null;
}

function parseEnumParam<const T extends readonly string[]>(
  value: string | null,
  allowedValues: T,
  fallback: T[number],
): T[number] {
  const normalizedValue = value?.trim() ?? "";
  return allowedValues.includes(normalizedValue as T[number]) ? (normalizedValue as T[number]) : fallback;
}

function parseCostBoundParam(value: string | null): number | null {
  if (!value) {
    return null;
  }
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : null;
}

function normalizeCostFilter(costFilter: CostFilter): CostFilter {
  if (costFilter.min != null && costFilter.max != null && costFilter.min > costFilter.max) {
    return { min: costFilter.max, max: costFilter.min };
  }
  return costFilter;
}

function parseStagingParam(value: string | null): ViewerSelection | null {
  const normalizedValue = value?.trim() ?? "";
  if (!normalizedValue) return null;
  const [runId, ...recordParts] = normalizedValue.split(":");
  const recordId = recordParts.join(":").trim();
  if (!runId || !recordId) return null;
  return { kind: "staging", runId: runId.trim(), recordId };
}

function normalizeOptionalQueryParam(value: string | null): string | null {
  const normalizedValue = value?.trim() ?? "";
  return normalizedValue ? normalizedValue : null;
}

function parseTimeFilterPoint(value: string | null): TimeFilterPoint | null {
  const normalized = value?.trim() ?? "";
  return TIME_FILTER_POINTS.has(normalized) ? (normalized as TimeFilterPoint) : null;
}

function normalizeRatingFilter(values: string[]): RatingFilter {
  return Array.from(
    new Set(
      values
        .map((value) => value.trim())
        .filter((value): value is RatingFilterValue =>
          RATING_FILTERS.includes(value as RatingFilterValue),
        ),
    ),
  );
}

function readViewerUrlState(): ViewerUrlState {
  if (typeof window === "undefined") {
    return defaultViewerUrlState;
  }

  try {
    const params = new URLSearchParams(window.location.search);
    const rawRecordId = normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.record));
    const rawStaging = normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.staging));
    let selection: ViewerSelection | null = null;
    if (rawStaging) {
      selection = parseStagingParam(rawStaging);
    }
    if (!selection && rawRecordId) {
      selection = { kind: "record", recordId: rawRecordId };
    }

    const browserTab = parseEnumParam(
      params.get(URL_QUERY_PARAMS.browser),
      BROWSER_TABS,
      selection?.kind === "staging" ? "staging" : "library",
    );

    return {
      selection,
      selectedRecordId: selectionToSelectedRecordId(selection),
      selectedInspectorTab: parseEnumParam(
        params.get(URL_QUERY_PARAMS.tab),
        INSPECTOR_TABS,
        defaultViewerUrlState.selectedInspectorTab,
      ),
      searchQuery: params.get(URL_QUERY_PARAMS.search) ?? "",
      browserTab,
      sourceFilter: "library",
      timeFilter: {
        oldest: parseTimeFilterPoint(params.get(URL_QUERY_PARAMS.timeFrom)),
        newest: parseTimeFilterPoint(params.get(URL_QUERY_PARAMS.timeTo)),
      },
      modelFilter: normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.model)),
      sdkFilter: normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.sdk)),
      agentHarnessFilters: normalizeAgentHarnessFilters(
        params.getAll(URL_QUERY_PARAMS.agentHarness),
      ),
      authorFilters: Array.from(
        new Set(params.getAll(URL_QUERY_PARAMS.author).map((value) => value.trim()).filter(Boolean)),
      ),
      categoryFilters: Array.from(
        new Set(params.getAll(URL_QUERY_PARAMS.category).map((value) => value.trim()).filter(Boolean)),
      ),
      costFilter: normalizeCostFilter({
        min: parseCostBoundParam(params.get(URL_QUERY_PARAMS.costMin)),
        max: parseCostBoundParam(params.get(URL_QUERY_PARAMS.costMax)),
      }),
      ratingFilter: normalizeRatingFilter(params.getAll(URL_QUERY_PARAMS.rating)),
      secondaryRatingFilter: normalizeRatingFilter(
        params.getAll(URL_QUERY_PARAMS.secondaryRating),
      ),
      selectedRunId: normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.run)),
    };
  } catch {
    return defaultViewerUrlState;
  }
}

function syncViewerStateToUrl(state: ViewerUrlState): void {
  if (typeof window === "undefined") {
    return;
  }

  const url = new URL(window.location.href);
  if (state.selection?.kind === "record") {
    url.searchParams.set(URL_QUERY_PARAMS.record, state.selection.recordId);
    url.searchParams.delete(URL_QUERY_PARAMS.staging);
  } else if (state.selection?.kind === "staging") {
    url.searchParams.delete(URL_QUERY_PARAMS.record);
    url.searchParams.set(URL_QUERY_PARAMS.staging, `${state.selection.runId}:${state.selection.recordId}`);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.record);
    url.searchParams.delete(URL_QUERY_PARAMS.staging);
  }

  if (state.selectedInspectorTab !== "inspect") {
    url.searchParams.set(URL_QUERY_PARAMS.tab, state.selectedInspectorTab);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.tab);
  }

  if (state.searchQuery.trim()) {
    url.searchParams.set(URL_QUERY_PARAMS.search, state.searchQuery);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.search);
  }

  if (state.browserTab === "staging") {
    url.searchParams.set(URL_QUERY_PARAMS.browser, "staging");
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.browser);
  }

  const setSingle = (key: string, value: string | null) => {
    if (value) {
      url.searchParams.set(key, value);
    } else {
      url.searchParams.delete(key);
    }
  };

  setSingle(URL_QUERY_PARAMS.timeFrom, state.timeFilter.oldest);
  setSingle(URL_QUERY_PARAMS.timeTo, state.timeFilter.newest);
  setSingle(URL_QUERY_PARAMS.model, state.modelFilter);
  setSingle(URL_QUERY_PARAMS.sdk, state.sdkFilter);
  setSingle(URL_QUERY_PARAMS.run, state.selectedRunId);
  setSingle(URL_QUERY_PARAMS.costMin, state.costFilter.min == null ? null : String(state.costFilter.min));
  setSingle(URL_QUERY_PARAMS.costMax, state.costFilter.max == null ? null : String(state.costFilter.max));

  for (const key of [
    URL_QUERY_PARAMS.agentHarness,
    URL_QUERY_PARAMS.author,
    URL_QUERY_PARAMS.category,
    URL_QUERY_PARAMS.rating,
    URL_QUERY_PARAMS.secondaryRating,
  ]) {
    url.searchParams.delete(key);
  }
  for (const value of [...state.agentHarnessFilters].sort()) {
    url.searchParams.append(URL_QUERY_PARAMS.agentHarness, value);
  }
  for (const value of [...state.authorFilters].sort()) {
    url.searchParams.append(URL_QUERY_PARAMS.author, value);
  }
  for (const value of [...state.categoryFilters].sort()) {
    url.searchParams.append(URL_QUERY_PARAMS.category, value);
  }
  for (const value of [...state.ratingFilter].sort()) {
    url.searchParams.append(URL_QUERY_PARAMS.rating, value);
  }
  for (const value of [...state.secondaryRatingFilter].sort()) {
    url.searchParams.append(URL_QUERY_PARAMS.secondaryRating, value);
  }

  window.history.replaceState(window.history.state, "", url);
}

const initialState: ViewerState = {
  ...readViewerUrlState(),
  bootstrap: null,
  recordCache: {},
  selectedRecordSummary: null,
  inspectorOpen: true,
  loading: true,
  error: null,
  multiSelection: new Set(),
};

function seedRecordCacheFromBootstrap(
  bootstrap: ViewerBootstrap,
  existing: Record<string, RecordSummary>,
): Record<string, RecordSummary> {
  const next: Record<string, RecordSummary> = { ...existing };
  for (const record of bootstrap.library_records) {
    next[record.record_id] = record;
  }
  return next;
}

function updateRecordSummaryFields(
  summary: RecordSummary | null,
  recordId: string,
  updates: Partial<
    Pick<
      RecordSummary,
      "rating" | "secondary_rating" | "effective_rating" | "rated_by" | "secondary_rated_by" | "updated_at"
    >
  >,
): RecordSummary | null {
  if (!summary || summary.record_id !== recordId) {
    return summary;
  }
  const nextSummary: RecordSummary = { ...summary, ...updates };
  return {
    ...nextSummary,
    effective_rating: computeEffectiveRating(nextSummary.rating, nextSummary.secondary_rating),
  };
}

function updateRecordCacheFields(
  recordCache: Record<string, RecordSummary>,
  recordId: string,
  updates: Partial<
    Pick<
      RecordSummary,
      "rating" | "secondary_rating" | "effective_rating" | "rated_by" | "secondary_rated_by" | "updated_at"
    >
  >,
): Record<string, RecordSummary> {
  const nextRecord = updateRecordSummaryFields(recordCache[recordId] ?? null, recordId, updates);
  if (!nextRecord) {
    return recordCache;
  }
  return { ...recordCache, [recordId]: nextRecord };
}

function updateBootstrapRecordFields(
  bootstrap: ViewerBootstrap | null,
  recordId: string,
  updates: Partial<
    Pick<
      RecordSummary,
      "rating" | "secondary_rating" | "effective_rating" | "rated_by" | "secondary_rated_by" | "updated_at"
    >
  >,
): ViewerBootstrap | null {
  if (!bootstrap) {
    return bootstrap;
  }
  return {
    ...bootstrap,
    library_records: bootstrap.library_records.map((record) =>
      updateRecordSummaryFields(record, recordId, updates) ?? record,
    ),
  };
}

function removeRecordFromBootstrap(
  bootstrap: ViewerBootstrap | null,
  recordId: string,
): ViewerBootstrap | null {
  if (!bootstrap) {
    return bootstrap;
  }
  return {
    ...bootstrap,
    library_records: bootstrap.library_records.filter((record) => record.record_id !== recordId),
  };
}

function removeRecordFromCache(
  recordCache: Record<string, RecordSummary>,
  recordId: string,
): Record<string, RecordSummary> {
  if (!(recordId in recordCache)) {
    return recordCache;
  }
  const next = { ...recordCache };
  delete next[recordId];
  return next;
}

function applySelection(state: ViewerState, selection: ViewerSelection | null): ViewerState {
  return {
    ...state,
    selection,
    selectedRecordId: selectionToSelectedRecordId(selection),
    selectedRecordSummary:
      selection?.kind === "record"
        ? state.selectedRecordSummary?.record_id === selection.recordId
          ? state.selectedRecordSummary
          : state.recordCache[selection.recordId] ?? null
        : null,
    inspectorOpen: selection !== null ? true : state.inspectorOpen,
  };
}

function isRunActive(run: RunSummary): boolean {
  const status = (run.status ?? "").toLowerCase();
  return Boolean(status) && status !== "success" && status !== "failed";
}

function viewerReducer(state: ViewerState, action: ViewerAction): ViewerState {
  switch (action.type) {
    case "SET_BOOTSTRAP": {
      const nextRecordCache = seedRecordCacheFromBootstrap(action.payload, state.recordCache);
      return {
        ...state,
        bootstrap: action.payload,
        recordCache: nextRecordCache,
        selectedRecordSummary:
          state.selectedRecordId != null
            ? nextRecordCache[state.selectedRecordId] ?? state.selectedRecordSummary
            : null,
        loading: false,
        error: null,
      };
    }
    case "UPSERT_RECORDS": {
      const nextRecordCache = { ...state.recordCache };
      for (const record of action.payload) {
        nextRecordCache[record.record_id] = record;
      }
      return {
        ...state,
        recordCache: nextRecordCache,
        selectedRecordSummary:
          state.selectedRecordId != null
            ? nextRecordCache[state.selectedRecordId] ?? state.selectedRecordSummary
            : null,
      };
    }
    case "SET_SELECTED_RECORD_SUMMARY":
      return {
        ...state,
        selectedRecordSummary: action.payload,
        recordCache:
          action.payload == null
            ? state.recordCache
            : { ...state.recordCache, [action.payload.record_id]: action.payload },
      };
    case "SYNC_FROM_URL":
      return {
        ...state,
        ...action.payload,
        selectedRecordSummary:
          action.payload.selectedRecordId != null
            ? state.recordCache[action.payload.selectedRecordId] ?? null
            : null,
      };
    case "DELETE_RECORD_LOCAL": {
      const nextMultiSelection = new Set(state.multiSelection);
      nextMultiSelection.delete(action.payload);
      const nextSelection =
        state.selection?.kind === "record" && state.selection.recordId === action.payload
          ? null
          : state.selection;
      return {
        ...state,
        bootstrap: removeRecordFromBootstrap(state.bootstrap, action.payload),
        recordCache: removeRecordFromCache(state.recordCache, action.payload),
        selection: nextSelection,
        selectedRecordId: selectionToSelectedRecordId(nextSelection),
        selectedRecordSummary:
          state.selectedRecordSummary?.record_id === action.payload
            ? null
            : state.selectedRecordSummary,
        error: null,
        multiSelection: nextMultiSelection,
      };
    }
    case "SET_INSPECTOR_TAB":
      return { ...state, selectedInspectorTab: action.payload };
    case "SELECT_RECORD":
      return applySelection(
        state,
        action.payload ? { kind: "record", recordId: action.payload } : null,
      );
    case "SELECT_ITEM":
      return applySelection(state, action.payload);
    case "UPDATE_STAGING": {
      if (!state.bootstrap) return state;
      let nextSelection = state.selection;
      if (nextSelection?.kind === "staging") {
        const currentSelection = nextSelection;
        const stillExists = action.payload.some(
          (entry) =>
            entry.run_id === currentSelection.runId && entry.record_id === currentSelection.recordId,
        );
        if (!stillExists) {
          nextSelection = null;
        }
      }
      return {
        ...state,
        bootstrap: { ...state.bootstrap, staging_entries: action.payload },
        selection: nextSelection,
        selectedRecordId: selectionToSelectedRecordId(nextSelection),
      };
    }
    case "UPDATE_RECORD_RATING":
      return {
        ...state,
        recordCache: updateRecordCacheFields(state.recordCache, action.payload.recordId, {
          rating: action.payload.rating,
          rated_by: null,
          updated_at: action.payload.updatedAt,
        }),
        selectedRecordSummary: updateRecordSummaryFields(
          state.selectedRecordSummary,
          action.payload.recordId,
          { rating: action.payload.rating, rated_by: null, updated_at: action.payload.updatedAt },
        ),
        bootstrap: updateBootstrapRecordFields(state.bootstrap, action.payload.recordId, {
          rating: action.payload.rating,
          rated_by: null,
          updated_at: action.payload.updatedAt,
        }),
      };
    case "UPDATE_RECORD_SECONDARY_RATING":
      return {
        ...state,
        recordCache: updateRecordCacheFields(state.recordCache, action.payload.recordId, {
          secondary_rating: action.payload.secondaryRating,
          secondary_rated_by: null,
          updated_at: action.payload.updatedAt,
        }),
        selectedRecordSummary: updateRecordSummaryFields(
          state.selectedRecordSummary,
          action.payload.recordId,
          {
            secondary_rating: action.payload.secondaryRating,
            secondary_rated_by: null,
            updated_at: action.payload.updatedAt,
          },
        ),
        bootstrap: updateBootstrapRecordFields(state.bootstrap, action.payload.recordId, {
          secondary_rating: action.payload.secondaryRating,
          secondary_rated_by: null,
          updated_at: action.payload.updatedAt,
        }),
      };
    case "TOGGLE_INSPECTOR":
      return { ...state, inspectorOpen: !state.inspectorOpen };
    case "SET_LOADING":
      return { ...state, loading: action.payload };
    case "SET_ERROR":
      return { ...state, error: action.payload, loading: false };
    case "SET_SEARCH":
      return { ...state, searchQuery: action.payload, multiSelection: new Set() };
    case "SET_BROWSER_TAB":
      return { ...state, browserTab: action.payload, multiSelection: new Set() };
    case "SET_SOURCE_FILTER":
      return { ...state, sourceFilter: "library", browserTab: "library" };
    case "SET_TIME_FILTER":
      return { ...state, timeFilter: action.payload };
    case "SET_MODEL_FILTER":
      return { ...state, modelFilter: action.payload };
    case "SET_SDK_FILTER":
      return { ...state, sdkFilter: action.payload };
    case "SET_AGENT_HARNESS_FILTERS":
      return { ...state, agentHarnessFilters: normalizeAgentHarnessFilters(action.payload) };
    case "SET_AUTHOR_FILTERS":
      return { ...state, authorFilters: Array.from(new Set(action.payload.filter(Boolean))) };
    case "SET_CATEGORY_FILTERS":
      return { ...state, categoryFilters: Array.from(new Set(action.payload.filter(Boolean))) };
    case "SET_COST_FILTER":
      return { ...state, costFilter: normalizeCostFilter(action.payload) };
    case "SET_RATING_FILTER":
      return { ...state, ratingFilter: normalizeRatingFilter(action.payload) };
    case "SET_SECONDARY_RATING_FILTER":
      return { ...state, secondaryRatingFilter: normalizeRatingFilter(action.payload) };
    case "SET_RUN_FILTER":
      return { ...state, selectedRunId: action.payload };
    case "TOGGLE_MULTI_SELECT": {
      const next = new Set(state.multiSelection);
      if (next.has(action.payload)) {
        next.delete(action.payload);
      } else {
        next.add(action.payload);
      }
      return { ...state, multiSelection: next };
    }
    case "RANGE_MULTI_SELECT": {
      const { targetId, visibleIds } = action.payload;
      const targetIndex = visibleIds.indexOf(targetId);
      if (targetIndex === -1) return state;
      let anchorIndex = -1;
      for (let i = 0; i < visibleIds.length; i += 1) {
        if (state.multiSelection.has(visibleIds[i])) {
          anchorIndex = i;
        }
      }
      if (anchorIndex === -1) anchorIndex = targetIndex;
      const start = Math.min(anchorIndex, targetIndex);
      const end = Math.max(anchorIndex, targetIndex);
      const next = new Set(state.multiSelection);
      for (let i = start; i <= end; i += 1) {
        next.add(visibleIds[i]);
      }
      return { ...state, multiSelection: next };
    }
    case "SET_MULTI_SELECT_ALL":
      return { ...state, multiSelection: new Set(action.payload) };
    case "CLEAR_MULTI_SELECT":
      return { ...state, multiSelection: new Set() };
    default:
      return state;
  }
}

const ViewerStateContext = createContext<ViewerState>(initialState);
const ViewerDispatchContext = createContext<Dispatch<ViewerAction>>(() => {});

export function ViewerProvider({ children }: { children: ReactNode }): JSX.Element {
  const [state, dispatch] = useReducer(viewerReducer, initialState);
  const selectedCachedRecord =
    state.selection?.kind === "record" && state.selectedRecordId
      ? state.recordCache[state.selectedRecordId] ?? null
      : null;
  const bootstrapQuery = useQuery(bootstrapQueryOptions());
  const activeBootstrap = state.bootstrap ?? bootstrapQuery.data ?? null;
  const shouldPollStaging =
    activeBootstrap != null
    && (state.browserTab === "staging" || activeBootstrap.runs.some((run) => isRunActive(run)));
  const stagingEntriesQuery = useQuery({
    ...stagingEntriesQueryOptions(),
    enabled: shouldPollStaging,
    refetchInterval: STAGING_POLL_INTERVAL_MS,
    refetchIntervalInBackground: true,
  });
  const selectedRecordId =
    state.selection?.kind === "record" ? state.selectedRecordId : null;
  const selectedRecordSummaryQuery = useQuery({
    ...recordSummaryQueryOptions(selectedRecordId ?? ""),
    enabled: selectedRecordId != null,
    retry: (failureCount, error) =>
      !(error instanceof HttpError && error.status === 404) && failureCount < 2,
  });

  useEffect(() => {
    dispatch({ type: "SET_LOADING", payload: bootstrapQuery.isPending });
  }, [bootstrapQuery.isPending]);

  useEffect(() => {
    if (bootstrapQuery.data) {
      dispatch({ type: "SET_BOOTSTRAP", payload: bootstrapQuery.data });
    }
  }, [bootstrapQuery.data]);

  useEffect(() => {
    if (!bootstrapQuery.error) {
      return;
    }
    dispatch({
      type: "SET_ERROR",
      payload:
        bootstrapQuery.error instanceof Error
          ? bootstrapQuery.error.message
          : "Failed to load viewer data.",
    });
  }, [bootstrapQuery.error]);

  useEffect(() => {
    if (state.selection?.kind !== "record" || !state.selectedRecordId) {
      dispatch({ type: "SET_SELECTED_RECORD_SUMMARY", payload: null });
      return;
    }
    if (selectedCachedRecord) {
      dispatch({ type: "SET_SELECTED_RECORD_SUMMARY", payload: selectedCachedRecord });
    }
  }, [selectedCachedRecord, state.selectedRecordId, state.selection]);

  useEffect(() => {
    if (state.selection?.kind !== "record" || !state.selectedRecordId) {
      return;
    }
    if (selectedRecordSummaryQuery.data) {
      dispatch({ type: "SET_SELECTED_RECORD_SUMMARY", payload: selectedRecordSummaryQuery.data });
    }
  }, [
    selectedRecordSummaryQuery.data,
    state.selectedRecordId,
    state.selection?.kind,
  ]);

  useEffect(() => {
    if (
      state.selection?.kind !== "record"
      || !state.selectedRecordId
      || !selectedRecordSummaryQuery.error
    ) {
      return;
    }
    if (
      selectedRecordSummaryQuery.error instanceof HttpError
      && selectedRecordSummaryQuery.error.status === 404
    ) {
      dispatch({ type: "DELETE_RECORD_LOCAL", payload: state.selectedRecordId });
      return;
    }
    if (!selectedCachedRecord) {
      dispatch({ type: "SET_SELECTED_RECORD_SUMMARY", payload: null });
    }
  }, [
    selectedCachedRecord,
    selectedRecordSummaryQuery.error,
    state.selectedRecordId,
    state.selection?.kind,
  ]);

  useEffect(() => {
    if (stagingEntriesQuery.data) {
      dispatch({ type: "UPDATE_STAGING", payload: stagingEntriesQuery.data });
    }
  }, [stagingEntriesQuery.data]);

  useEffect(() => {
    syncViewerStateToUrl({
      selection: state.selection,
      selectedRecordId: state.selectedRecordId,
      selectedInspectorTab: state.selectedInspectorTab,
      searchQuery: state.searchQuery,
      browserTab: state.browserTab,
      sourceFilter: state.sourceFilter,
      timeFilter: state.timeFilter,
      modelFilter: state.modelFilter,
      sdkFilter: state.sdkFilter,
      agentHarnessFilters: state.agentHarnessFilters,
      authorFilters: state.authorFilters,
      categoryFilters: state.categoryFilters,
      costFilter: state.costFilter,
      ratingFilter: state.ratingFilter,
      secondaryRatingFilter: state.secondaryRatingFilter,
      selectedRunId: state.selectedRunId,
    });
  }, [
    state.agentHarnessFilters,
    state.authorFilters,
    state.browserTab,
    state.categoryFilters,
    state.costFilter,
    state.modelFilter,
    state.ratingFilter,
    state.sdkFilter,
    state.searchQuery,
    state.secondaryRatingFilter,
    state.selectedInspectorTab,
    state.selectedRecordId,
    state.selectedRunId,
    state.selection,
    state.sourceFilter,
    state.timeFilter,
  ]);

  useEffect(() => {
    if (typeof window === "undefined") {
      return;
    }
    const handlePopState = () => {
      dispatch({ type: "SYNC_FROM_URL", payload: readViewerUrlState() });
    };
    window.addEventListener("popstate", handlePopState);
    return () => {
      window.removeEventListener("popstate", handlePopState);
    };
  }, []);

  return (
    <ViewerStateContext.Provider value={state}>
      <ViewerDispatchContext.Provider value={dispatch}>
        {children}
      </ViewerDispatchContext.Provider>
    </ViewerStateContext.Provider>
  );
}

export function useViewer(): ViewerState {
  return useContext(ViewerStateContext);
}

export function useViewerDispatch(): Dispatch<ViewerAction> {
  return useContext(ViewerDispatchContext);
}
