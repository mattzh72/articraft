export type RecordSummary = {
  record_id: string;
  title: string;
  prompt_preview: string;
  rating: number | null;
  created_at: string | null;
  updated_at: string | null;
  sdk_package: string | null;
  provider: string | null;
  model_id: string | null;
  turn_count: number | null;
  total_cost_usd: number | null;
  category_slug: string | null;
  run_id: string | null;
  collections: string[];
  materialization_status: string | null;
  has_compile_report: boolean;
  has_provenance: boolean;
  has_cost: boolean;
};

export type WorkbenchEntry = {
  record_id: string;
  added_at: string;
  label: string | null;
  tags: string[];
  archived: boolean;
  record: RecordSummary | null;
};

export type DatasetEntry = {
  record_id: string;
  dataset_id: string;
  category_slug: string;
  promoted_at: string;
  record: RecordSummary | null;
};

export type CategoryOption = {
  slug: string;
  title: string;
};

export type StagingEntry = {
  run_id: string;
  record_id: string;
  title: string;
  prompt_preview: string;
  status: string | null;
  message: string | null;
  created_at: string | null;
  updated_at: string | null;
  collection: string | null;
  category_slug: string | null;
  provider: string | null;
  model_id: string | null;
  sdk_package: string | null;
  turn_count: number | null;
  tool_call_count: number | null;
  compile_attempt_count: number | null;
  staging_dir: string;
  has_prompt: boolean;
  has_model_script: boolean;
  model_script_updated_at: string | null;
  has_checkpoint_urdf: boolean;
  checkpoint_updated_at: string | null;
  has_cost: boolean;
  has_traces: boolean;
  persisted_record: RecordSummary | null;
};

export type RunSummary = {
  run_id: string;
  run_mode: string | null;
  collection: string | null;
  status: string | null;
  created_at: string | null;
  updated_at: string | null;
  provider: string | null;
  model_id: string | null;
  sdk_package: string | null;
  prompt_count: number | null;
  result_count: number;
  success_count: number;
  failed_count: number;
};

export type RunResult = {
  record_id: string | null;
  status: string | null;
  message: string | null;
  turn_count: number | null;
  tool_call_count: number | null;
  compile_attempt_count: number | null;
  record_dir: string | null;
  staging_dir: string | null;
  raw: Record<string, unknown>;
};

export type RecordDetail = {
  summary: RecordSummary;
  record: Record<string, unknown> | null;
  compile_report: Record<string, unknown> | null;
  provenance: Record<string, unknown> | null;
  cost: Record<string, unknown> | null;
};

export type RecordRatingResponse = {
  record_id: string;
  rating: number;
  updated_at: string | null;
};

export type DeleteRecordResult = {
  status: string;
  record_id: string;
};

export type DeleteStagingResult = {
  status: string;
  run_id: string;
  record_id: string;
};

export type OpenRecordFolderResult = {
  status: string;
  record_id: string;
  path: string;
};

export type OpenStagingFolderResult = {
  status: string;
  run_id: string;
  record_id: string;
  path: string;
};

export type RunDetail = {
  run: RunSummary;
  run_metadata: Record<string, unknown>;
  results: RunResult[];
  records: RecordDetail[];
};

export type RepoStats = {
  total_records: number;
  workbench_count: number;
  dataset_count: number;
  total_runs: number;
  total_cost_usd: number | null;
  data_size_bytes: number | null;
  category_counts: Record<string, number>;
  category_stats: Record<
    string,
    {
      count: number;
      average_rating: number | null;
      average_cost_usd: number | null;
    }
  >;
  model_counts: Record<string, number>;
  provider_counts: Record<string, number>;
  rating_distribution: Record<string, number>;
};

export type ViewerBootstrap = {
  repo_root: string;
  generated_at: string;
  workbench_entries: WorkbenchEntry[];
  dataset_entries: DatasetEntry[];
  staging_entries: StagingEntry[];
  runs: RunSummary[];
};

export type ViewerSelection =
  | { kind: "record"; recordId: string }
  | { kind: "staging"; runId: string; recordId: string };

export type SourceFilter = "workbench" | "dataset";
export type BrowserTab = SourceFilter | "staging";
export type TimeFilter = "any" | "24h" | "7d" | "30d" | "90d";
export type CostFilter = {
  min: number | null;
  max: number | null;
};
export type RatingFilterValue = "1" | "2" | "3" | "4" | "5" | "unrated";
export type RatingFilter = RatingFilterValue[];
export type InspectorTab = "inspect" | "render" | "code" | "metadata";

export type ViewerState = {
  bootstrap: ViewerBootstrap | null;
  selection: ViewerSelection | null;
  selectedRecordId: string | null;
  selectedInspectorTab: InspectorTab;
  inspectorOpen: boolean;
  loading: boolean;
  error: string | null;
  searchQuery: string;
  browserTab: BrowserTab;
  sourceFilter: SourceFilter;
  timeFilter: TimeFilter;
  modelFilter: string | null;
  categoryFilters: string[];
  costFilter: CostFilter;
  ratingFilter: RatingFilter;
  selectedRunId: string | null;
  multiSelection: Set<string>;
};

export type ViewerAction =
  | { type: "SET_BOOTSTRAP"; payload: ViewerBootstrap }
  | {
      type: "SYNC_FROM_URL";
      payload: {
        selection: ViewerSelection | null;
        selectedRecordId: string | null;
        selectedInspectorTab: InspectorTab;
        searchQuery: string;
        browserTab: BrowserTab;
        sourceFilter: SourceFilter;
        timeFilter: TimeFilter;
        modelFilter: string | null;
        categoryFilters: string[];
        costFilter: CostFilter;
        ratingFilter: RatingFilter;
        selectedRunId: string | null;
      };
    }
  | { type: "DELETE_RECORD_LOCAL"; payload: string }
  | { type: "SELECT_RECORD"; payload: string | null }
  | { type: "SELECT_ITEM"; payload: ViewerSelection | null }
  | { type: "UPDATE_STAGING"; payload: StagingEntry[] }
  | { type: "SET_INSPECTOR_TAB"; payload: InspectorTab }
  | { type: "UPDATE_RECORD_RATING"; payload: { recordId: string; rating: number; updatedAt: string | null } }
  | { type: "TOGGLE_INSPECTOR" }
  | { type: "SET_LOADING"; payload: boolean }
  | { type: "SET_ERROR"; payload: string | null }
  | { type: "SET_SEARCH"; payload: string }
  | { type: "SET_BROWSER_TAB"; payload: BrowserTab }
  | { type: "SET_SOURCE_FILTER"; payload: SourceFilter }
  | { type: "SET_TIME_FILTER"; payload: TimeFilter }
  | { type: "SET_MODEL_FILTER"; payload: string | null }
  | { type: "SET_CATEGORY_FILTERS"; payload: string[] }
  | { type: "SET_COST_FILTER"; payload: CostFilter }
  | { type: "SET_RATING_FILTER"; payload: RatingFilter }
  | { type: "SET_RUN_FILTER"; payload: string | null }
  | { type: "TOGGLE_MULTI_SELECT"; payload: string }
  | { type: "RANGE_MULTI_SELECT"; payload: { targetId: string; visibleIds: string[] } }
  | { type: "SET_MULTI_SELECT_ALL"; payload: string[] }
  | { type: "CLEAR_MULTI_SELECT" };
