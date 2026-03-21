import {
  createContext,
  useContext,
  useEffect,
  useReducer,
  useRef,
  type Dispatch,
  type JSX,
  type ReactNode,
} from "react";

import { fetchBootstrap, fetchStagingEntries } from "@/lib/api";
import { useRoute } from "@/lib/useRoute";
import type {
  BrowserTab,
  CostFilter,
  InspectorTab,
  RatingFilter,
  RatingFilterValue,
  RecordSummary,
  SourceFilter,
  TimeFilter,
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
  source: "source",
  time: "time",
  model: "model",
  category: "category",
  costMin: "cost_min",
  costMax: "cost_max",
  rating: "rating",
  run: "run",
} as const;

const INSPECTOR_TABS = ["inspect", "render", "code", "metadata"] as const satisfies readonly InspectorTab[];
const BROWSER_TABS = ["workbench", "dataset", "staging"] as const satisfies readonly BrowserTab[];
const SOURCE_FILTERS = ["workbench", "dataset"] as const satisfies readonly SourceFilter[];
const TIME_FILTERS = ["any", "24h", "7d", "30d", "90d"] as const satisfies readonly TimeFilter[];
const RATING_FILTERS = ["1", "2", "3", "4", "5", "unrated"] as const satisfies readonly RatingFilterValue[];

const STAGING_POLL_INTERVAL_MS = 3000;

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
  | "categoryFilters"
  | "costFilter"
  | "ratingFilter"
  | "selectedRunId"
>;

const defaultViewerUrlState: ViewerUrlState = {
  selection: null,
  selectedRecordId: null,
  selectedInspectorTab: "inspect",
  searchQuery: "",
  browserTab: "workbench",
  sourceFilter: "workbench",
  timeFilter: "any",
  modelFilter: null,
  categoryFilters: [],
  costFilter: { min: null, max: null },
  ratingFilter: [],
  selectedRunId: null,
};

function selectionToSelectedRecordId(selection: ViewerSelection | null): string | null {
  if (!selection) return null;
  return selection.kind === "record" ? selection.recordId : null;
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
  if (!Number.isFinite(parsed) || parsed < 0) {
    return null;
  }

  return parsed;
}

function normalizeCostFilter(costFilter: CostFilter): CostFilter {
  const min = costFilter.min;
  const max = costFilter.max;

  if (min == null && max == null) {
    return { min: null, max: null };
  }
  if (min != null && max != null && min > max) {
    return { min: max, max: min };
  }
  return { min, max };
}

function parseStagingParam(value: string | null): ViewerSelection | null {
  const normalizedValue = value?.trim() ?? "";
  if (!normalizedValue) return null;
  const parts = normalizedValue.split(":");
  if (parts.length < 2) return null;
  const runId = parts[0]?.trim() ?? "";
  const recordId = parts.slice(1).join(":").trim();
  if (!runId || !recordId) return null;
  return { kind: "staging", runId, recordId };
}

function normalizeOptionalQueryParam(value: string | null): string | null {
  const normalizedValue = value?.trim() ?? "";
  return normalizedValue ? normalizedValue : null;
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

    const selectedRecordId = selectionToSelectedRecordId(selection);
    const selectedInspectorTab = parseEnumParam(
      params.get(URL_QUERY_PARAMS.tab),
      INSPECTOR_TABS,
      defaultViewerUrlState.selectedInspectorTab,
    );
    const searchQuery = params.get(URL_QUERY_PARAMS.search) ?? defaultViewerUrlState.searchQuery;
    const parsedSourceFilter = parseEnumParam(
      params.get(URL_QUERY_PARAMS.source),
      SOURCE_FILTERS,
      defaultViewerUrlState.sourceFilter,
    );
    const parsedBrowserTab = parseEnumParam(
      params.get(URL_QUERY_PARAMS.browser),
      BROWSER_TABS,
      selection?.kind === "staging" ? "staging" : parsedSourceFilter,
    );
    const sourceFilter =
      parsedBrowserTab === "staging" ? parsedSourceFilter : parsedBrowserTab;
    const timeFilter = parseEnumParam(
      params.get(URL_QUERY_PARAMS.time),
      TIME_FILTERS,
      defaultViewerUrlState.timeFilter,
    );
    const modelFilter = normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.model));
    const categoryFilters = Array.from(
      new Set(
        params
          .getAll(URL_QUERY_PARAMS.category)
          .map((value) => value.trim())
          .filter((value) => value.length > 0),
      ),
    );
    const costFilter = normalizeCostFilter({
      min: parseCostBoundParam(params.get(URL_QUERY_PARAMS.costMin)),
      max: parseCostBoundParam(params.get(URL_QUERY_PARAMS.costMax)),
    });
    const ratingFilter = normalizeRatingFilter(params.getAll(URL_QUERY_PARAMS.rating));
    const selectedRunId = normalizeOptionalQueryParam(params.get(URL_QUERY_PARAMS.run));

    return {
      selection,
      selectedRecordId,
      selectedInspectorTab,
      searchQuery,
      browserTab: parsedBrowserTab,
      sourceFilter,
      timeFilter,
      modelFilter,
      categoryFilters,
      costFilter,
      ratingFilter,
      selectedRunId,
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

  // Selection: record or staging param
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

  if (state.selectedInspectorTab !== defaultViewerUrlState.selectedInspectorTab) {
    url.searchParams.set(URL_QUERY_PARAMS.tab, state.selectedInspectorTab);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.tab);
  }

  if (state.searchQuery.trim()) {
    url.searchParams.set(URL_QUERY_PARAMS.search, state.searchQuery);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.search);
  }

  url.searchParams.set(URL_QUERY_PARAMS.browser, state.browserTab);

  if (state.browserTab === "staging" && state.sourceFilter !== defaultViewerUrlState.sourceFilter) {
    url.searchParams.set(URL_QUERY_PARAMS.source, state.sourceFilter);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.source);
  }

  if (state.browserTab !== "staging" && state.timeFilter !== defaultViewerUrlState.timeFilter) {
    url.searchParams.set(URL_QUERY_PARAMS.time, state.timeFilter);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.time);
  }

  if (state.browserTab !== "staging" && state.modelFilter) {
    url.searchParams.set(URL_QUERY_PARAMS.model, state.modelFilter);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.model);
  }

  url.searchParams.delete(URL_QUERY_PARAMS.category);
  if (state.browserTab !== "staging" && state.sourceFilter === "dataset") {
    for (const categoryFilter of [...state.categoryFilters].sort((left, right) => left.localeCompare(right))) {
      url.searchParams.append(URL_QUERY_PARAMS.category, categoryFilter);
    }
  }

  if (state.browserTab !== "staging" && state.costFilter.min != null) {
    url.searchParams.set(URL_QUERY_PARAMS.costMin, String(state.costFilter.min));
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.costMin);
  }

  if (state.browserTab !== "staging" && state.costFilter.max != null) {
    url.searchParams.set(URL_QUERY_PARAMS.costMax, String(state.costFilter.max));
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.costMax);
  }

  url.searchParams.delete(URL_QUERY_PARAMS.rating);
  if (state.browserTab !== "staging") {
    for (const ratingFilter of [...state.ratingFilter].sort((left, right) => left.localeCompare(right))) {
      url.searchParams.append(URL_QUERY_PARAMS.rating, ratingFilter);
    }
  }

  if (state.browserTab !== "staging" && state.selectedRunId) {
    url.searchParams.set(URL_QUERY_PARAMS.run, state.selectedRunId);
  } else {
    url.searchParams.delete(URL_QUERY_PARAMS.run);
  }

  window.history.replaceState(window.history.state, "", url);
}

const initialState: ViewerState = {
  ...readViewerUrlState(),
  bootstrap: null,
  inspectorOpen: true,
  loading: true,
  error: null,
  multiSelection: new Set(),
};

function recordSortTimestamp(record: RecordSummary): number {
  const timestamp = record.updated_at ?? record.created_at;
  return timestamp ? new Date(timestamp).getTime() : 0;
}

function getBootstrapRecords(bootstrap: ViewerBootstrap): RecordSummary[] {
  const records = new Map<string, RecordSummary>();

  for (const entry of bootstrap.workbench_entries) {
    if (entry.record && !records.has(entry.record_id)) {
      records.set(entry.record_id, entry.record);
    }
  }

  for (const entry of bootstrap.dataset_entries) {
    if (entry.record && !records.has(entry.record_id)) {
      records.set(entry.record_id, entry.record);
    }
  }

  return Array.from(records.values()).sort((a, b) => {
    const dateA = recordSortTimestamp(a);
    const dateB = recordSortTimestamp(b);
    return dateB - dateA;
  });
}

function getNextSelection(
  bootstrap: ViewerBootstrap,
  currentSelection: ViewerSelection | null,
): ViewerSelection | null {
  // If current selection is a staging entry that still exists, keep it
  if (currentSelection?.kind === "staging") {
    const stillExists = bootstrap.staging_entries.some(
      (entry) => entry.run_id === currentSelection.runId && entry.record_id === currentSelection.recordId,
    );
    if (stillExists) return currentSelection;
  }

  // If current selection is a record that still exists, keep it
  const records = getBootstrapRecords(bootstrap);
  if (currentSelection?.kind === "record") {
    if (records.some((record) => record.record_id === currentSelection.recordId)) {
      return currentSelection;
    }
  }

  // Fall back to first record
  const firstRecord = records[0];
  return firstRecord ? { kind: "record", recordId: firstRecord.record_id } : null;
}

function updateRecordSummaryRating(
  summary: RecordSummary | null,
  recordId: string,
  rating: number,
  updatedAt: string | null,
): RecordSummary | null {
  if (!summary || summary.record_id !== recordId) {
    return summary;
  }
  return {
    ...summary,
    rating,
    updated_at: updatedAt,
  };
}

function updateBootstrapRecordRating(
  bootstrap: ViewerBootstrap | null,
  recordId: string,
  rating: number,
  updatedAt: string | null,
): ViewerBootstrap | null {
  if (!bootstrap) {
    return bootstrap;
  }

  return {
    ...bootstrap,
    workbench_entries: bootstrap.workbench_entries.map((entry) => ({
      ...entry,
      record: updateRecordSummaryRating(entry.record, recordId, rating, updatedAt),
    })),
    dataset_entries: bootstrap.dataset_entries.map((entry) => ({
      ...entry,
      record: updateRecordSummaryRating(entry.record, recordId, rating, updatedAt),
    })),
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
    workbench_entries: bootstrap.workbench_entries.filter((entry) => entry.record_id !== recordId),
    dataset_entries: bootstrap.dataset_entries.filter((entry) => entry.record_id !== recordId),
  };
}

function applySelection(state: ViewerState, selection: ViewerSelection | null): ViewerState {
  return {
    ...state,
    selection,
    selectedRecordId: selectionToSelectedRecordId(selection),
    inspectorOpen: selection !== null ? true : state.inspectorOpen,
  };
}

function viewerReducer(state: ViewerState, action: ViewerAction): ViewerState {
  switch (action.type) {
    case "SET_BOOTSTRAP": {
      const nextSelection = getNextSelection(action.payload, state.selection);
      const validIds = new Set(getBootstrapRecords(action.payload).map((r) => r.record_id));
      const prunedMultiSelection = new Set([...state.multiSelection].filter((id) => validIds.has(id)));
      return {
        ...state,
        bootstrap: action.payload,
        selection: nextSelection,
        selectedRecordId: selectionToSelectedRecordId(nextSelection),
        loading: false,
        error: null,
        multiSelection: prunedMultiSelection,
      };
    }
    case "SYNC_FROM_URL":
      return {
        ...state,
        ...action.payload,
      };
    case "DELETE_RECORD_LOCAL": {
      const nextBootstrap = removeRecordFromBootstrap(state.bootstrap, action.payload);
      const nextSelection = nextBootstrap ? getNextSelection(nextBootstrap, state.selection) : state.selection;
      const nextMultiSelection = new Set(state.multiSelection);
      nextMultiSelection.delete(action.payload);
      return {
        ...state,
        bootstrap: nextBootstrap,
        selection: nextSelection,
        selectedRecordId: selectionToSelectedRecordId(nextSelection),
        error: null,
        multiSelection: nextMultiSelection,
      };
    }
    case "SET_INSPECTOR_TAB":
      return {
        ...state,
        selectedInspectorTab: action.payload,
      };
    case "SELECT_RECORD": {
      const selection: ViewerSelection | null = action.payload
        ? { kind: "record", recordId: action.payload }
        : null;
      return applySelection(state, selection);
    }
    case "SELECT_ITEM":
      return applySelection(state, action.payload);
    case "UPDATE_STAGING": {
      if (!state.bootstrap) return state;
      const nextBootstrap: ViewerBootstrap = {
        ...state.bootstrap,
        staging_entries: action.payload,
      };
      // If current selection is a staging entry that no longer exists, clear it
      let nextSelection = state.selection;
      if (nextSelection?.kind === "staging") {
        const { runId, recordId } = nextSelection;
        const stillExists = action.payload.some(
          (entry) => entry.run_id === runId && entry.record_id === recordId,
        );
        if (!stillExists) {
          nextSelection = getNextSelection(nextBootstrap, null);
        }
      }
      return {
        ...state,
        bootstrap: nextBootstrap,
        selection: nextSelection,
        selectedRecordId: selectionToSelectedRecordId(nextSelection),
      };
    }
    case "UPDATE_RECORD_RATING":
      return {
        ...state,
        bootstrap: updateBootstrapRecordRating(
          state.bootstrap,
          action.payload.recordId,
          action.payload.rating,
          action.payload.updatedAt,
        ),
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
      if (action.payload === "staging") {
        return { ...state, browserTab: action.payload };
      }
      return {
        ...state,
        browserTab: action.payload,
        sourceFilter: action.payload,
        modelFilter: null,
        selectedRunId: null,
        multiSelection: new Set(),
      };
    case "SET_SOURCE_FILTER":
      return {
        ...state,
        browserTab: action.payload,
        sourceFilter: action.payload,
        modelFilter: null,
        selectedRunId: null,
        multiSelection: new Set(),
      };
    case "SET_TIME_FILTER":
      return { ...state, timeFilter: action.payload };
    case "SET_MODEL_FILTER":
      return { ...state, modelFilter: action.payload };
    case "SET_CATEGORY_FILTERS":
      return { ...state, categoryFilters: Array.from(new Set(action.payload.filter((value) => value.trim().length > 0))) };
    case "SET_COST_FILTER":
      return { ...state, costFilter: normalizeCostFilter(action.payload) };
    case "SET_RATING_FILTER":
      return { ...state, ratingFilter: normalizeRatingFilter(action.payload) };
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

      // Find the last-toggled item that is currently selected, or fall back to first selected
      let anchorIndex = -1;
      for (let i = 0; i < visibleIds.length; i++) {
        if (state.multiSelection.has(visibleIds[i])) {
          anchorIndex = i;
        }
      }
      if (anchorIndex === -1) anchorIndex = targetIndex;

      const start = Math.min(anchorIndex, targetIndex);
      const end = Math.max(anchorIndex, targetIndex);
      const next = new Set(state.multiSelection);
      for (let i = start; i <= end; i++) {
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

function useStagingPolling(state: ViewerState, dispatch: Dispatch<ViewerAction>): void {
  const stateRef = useRef(state);
  stateRef.current = state;

  const stagingEntryCount = state.bootstrap?.staging_entries.length ?? 0;
  const selectionKind = state.selection?.kind;

  useEffect(() => {
    const shouldPoll = (): boolean => {
      const { bootstrap, selection } = stateRef.current;
      if (!bootstrap) return false;
      if (bootstrap.staging_entries.length > 0) return true;
      if (selection?.kind === "staging") return true;
      return false;
    };

    if (!shouldPoll()) return;

    const intervalId = setInterval(() => {
      if (!shouldPoll()) return;

      fetchStagingEntries()
        .then((entries) => {
          dispatch({ type: "UPDATE_STAGING", payload: entries });
        })
        .catch(() => {
          // Silently ignore polling errors
        });
    }, STAGING_POLL_INTERVAL_MS);

    return () => {
      clearInterval(intervalId);
    };
  }, [dispatch, stagingEntryCount, selectionKind]);
}

export function ViewerProvider({ children }: { children: ReactNode }): JSX.Element {
  const [state, dispatch] = useReducer(viewerReducer, initialState);
  const route = useRoute();

  useEffect(() => {
    let cancelled = false;
    dispatch({ type: "SET_LOADING", payload: true });

    fetchBootstrap()
      .then((data) => {
        if (!cancelled) {
          dispatch({ type: "SET_BOOTSTRAP", payload: data });
        }
      })
      .catch((err) => {
        if (!cancelled) {
          dispatch({
            type: "SET_ERROR",
            payload: err instanceof Error ? err.message : "Failed to load viewer data.",
          });
        }
      });

    return () => {
      cancelled = true;
    };
  }, []);

  useStagingPolling(state, dispatch);

  useEffect(() => {
    if (route.page !== "viewer") {
      return;
    }

    syncViewerStateToUrl({
      selection: state.selection,
      selectedRecordId: state.selectedRecordId,
      selectedInspectorTab: state.selectedInspectorTab,
      searchQuery: state.searchQuery,
      browserTab: state.browserTab,
      sourceFilter: state.sourceFilter,
      timeFilter: state.timeFilter,
      modelFilter: state.modelFilter,
      categoryFilters: state.categoryFilters,
      costFilter: state.costFilter,
      ratingFilter: state.ratingFilter,
      selectedRunId: state.selectedRunId,
    });
  }, [
    state.categoryFilters,
    state.costFilter,
    state.modelFilter,
    state.ratingFilter,
    state.browserTab,
    state.searchQuery,
    state.selectedInspectorTab,
    state.selectedRecordId,
    state.selectedRunId,
    state.selection,
    state.sourceFilter,
    state.timeFilter,
    route.page,
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
