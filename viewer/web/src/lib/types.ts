export type RecordSummary = {
  record_id: string;
  title: string;
  prompt_preview: string;
  rating: number | null;
  secondary_rating: number | null;
  effective_rating: number | null;
  author: string | null;
  rated_by: string | null;
  secondary_rated_by: string | null;
  created_at: string | null;
  updated_at: string | null;
  viewer_asset_updated_at: string | null;
  sdk_package: string | null;
  provider: string | null;
  model_id: string | null;
  creator_mode: string | null;
  external_agent: string | null;
  agent_harness: AgentHarness;
  has_traces: boolean;
  thinking_level: string | null;
  turn_count: number | null;
  input_tokens: number | null;
  output_tokens: number | null;
  total_cost_usd: number | null;
  category_slug: string | null;
  run_id: string | null;
  run_status: string | null;
  run_message: string | null;
  active_revision_id: string | null;
  origin_record_id: string | null;
  parent_record_id: string | null;
  revision_count: number;
  has_history: boolean;
  collections: string[];
  materialization_status: string | null;
  has_compile_report: boolean;
  has_provenance: boolean;
  has_cost: boolean;
};

export type RecordBrowseFacets = {
  models: string[];
  sdk_packages: string[];
  agent_harnesses: AgentHarness[];
  authors: string[];
  categories: string[];
  cost_min: number | null;
  cost_max: number | null;
};

export type RecordBrowseResponse = {
  source: SourceFilter;
  total: number;
  source_total: number;
  offset: number;
  limit: number;
  record_ids: string[];
  records: RecordSummary[];
  facets: RecordBrowseFacets;
};

export type RecordBrowseIdsResponse = {
  source: SourceFilter;
  total: number;
  record_ids: string[];
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

export type SupercategoryOption = {
  slug: string;
  title: string;
  description: string;
  category_slugs: string[];
};

export type CategoryOption = {
  slug: string;
  title: string;
  supercategory_slug: string | null;
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
  thinking_level: string | null;
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

export type RecordHistoryRevision = {
  record_id: string;
  revision_id: string;
  active: boolean;
  created_at: string | null;
  prompt_preview: string;
  provider: string | null;
  model_id: string | null;
  run_id: string | null;
  parent_record_id: string | null;
  parent_revision_id: string | null;
  status: string | null;
  total_cost_usd: number | null;
  has_cost: boolean;
  has_traces: boolean;
  has_model: boolean;
  has_provenance: boolean;
};

export type RecordHistory = {
  record_id: string;
  active_revision_id: string | null;
  ancestors: RecordHistoryRevision[];
  revisions: RecordHistoryRevision[];
  descendants: RecordSummary[];
};

export type RecordRatingResponse = {
  record_id: string;
  rating: number;
  updated_at: string | null;
};

export type RecordSecondaryRatingResponse = {
  record_id: string;
  secondary_rating: number | null;
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
      sdk_package: string | null;
      average_rating: number | null;
      average_cost_usd: number | null;
      average_input_tokens: number | null;
      average_output_tokens: number | null;
    }
  >;
  model_counts: Record<string, number>;
  provider_counts: Record<string, number>;
  rating_distribution: Record<string, number>;
};

export type DashboardCostBounds = {
  min: number;
  max: number;
};

export type DashboardOverview = {
  total_records: number;
  total_runs: number;
  total_cost_usd: number | null;
  average_cost_usd: number | null;
  data_size_bytes: number | null;
  category_count: number;
  model_count: number;
  sdk_count: number;
  is_filtered: boolean;
};

export type DashboardCategoryStats = {
  count: number;
  sdk_package: string | null;
  average_rating: number | null;
  average_cost_usd: number | null;
  average_input_tokens: number | null;
  average_output_tokens: number | null;
  input_token_sample_count: number;
  output_token_sample_count: number;
};

export type DashboardCostTrendPoint = {
  date_key: string;
  day_start_ms: number;
  record_count: number;
  total_cost_usd: number;
  daily_average_cost_usd: number | null;
  rolling_average_cost_usd: number | null;
};

export type DashboardCostTrend = {
  points: DashboardCostTrendPoint[];
  latest_average_cost_usd: number | null;
  previous_average_cost_usd: number | null;
  delta_usd: number | null;
  delta_pct: number | null;
};

export type DashboardData = {
  generated_at: string;
  supercategories: SupercategoryOption[];
  available_sdks: string[];
  available_agent_harnesses: AgentHarness[];
  available_authors: string[];
  available_categories: string[];
  cost_bounds: DashboardCostBounds | null;
  overview: DashboardOverview;
  category_stats: Record<string, DashboardCategoryStats>;
  cost_trend: DashboardCostTrend;
};

export type ViewerBootstrap = {
  repo_root: string;
  generated_at: string;
  workbench_entries: WorkbenchEntry[];
  dataset_entries: DatasetEntry[];
  staging_entries: StagingEntry[];
  runs: RunSummary[];
  supercategories: SupercategoryOption[];
};

export type ViewerSelection =
  | { kind: "record"; recordId: string }
  | { kind: "staging"; runId: string; recordId: string };

export type SourceFilter = "workbench" | "dataset";
export type BrowserTab = SourceFilter | "staging";
export type AgentHarness = "articraft" | "codex" | "claude-code";
export type TimeFilterPoint = "1y" | "180d" | "90d" | "60d" | "30d" | "14d" | "7d" | "3d" | "24h" | "12h" | "6h" | "1h";
export type TimeFilter = {
  oldest: TimeFilterPoint | null;
  newest: TimeFilterPoint | null;
};
export type CostFilter = {
  min: number | null;
  max: number | null;
};
export type RatingFilterValue = "1" | "2" | "3" | "4" | "5" | "unrated";
export type RatingFilter = RatingFilterValue[];
export type InspectorTab = "inspect" | "render" | "code" | "metadata";

export type ViewerState = {
  bootstrap: ViewerBootstrap | null;
  recordCache: Record<string, RecordSummary>;
  selection: ViewerSelection | null;
  selectedRecordId: string | null;
  selectedRecordSummary: RecordSummary | null;
  selectedInspectorTab: InspectorTab;
  inspectorOpen: boolean;
  loading: boolean;
  error: string | null;
  searchQuery: string;
  browserTab: BrowserTab;
  sourceFilter: SourceFilter;
  timeFilter: TimeFilter;
  modelFilter: string | null;
  sdkFilter: string | null;
  agentHarnessFilters: AgentHarness[];
  authorFilters: string[];
  categoryFilters: string[];
  costFilter: CostFilter;
  ratingFilter: RatingFilter;
  secondaryRatingFilter: RatingFilter;
  selectedRunId: string | null;
  multiSelection: Set<string>;
};

export type ViewerAction =
  | { type: "SET_BOOTSTRAP"; payload: ViewerBootstrap }
  | { type: "UPSERT_RECORDS"; payload: RecordSummary[] }
  | { type: "SET_SELECTED_RECORD_SUMMARY"; payload: RecordSummary | null }
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
        sdkFilter: string | null;
        agentHarnessFilters: AgentHarness[];
        authorFilters: string[];
        categoryFilters: string[];
        costFilter: CostFilter;
        ratingFilter: RatingFilter;
        secondaryRatingFilter: RatingFilter;
        selectedRunId: string | null;
      };
    }
  | { type: "DELETE_RECORD_LOCAL"; payload: string }
  | { type: "SELECT_RECORD"; payload: string | null }
  | { type: "SELECT_ITEM"; payload: ViewerSelection | null }
  | { type: "UPDATE_STAGING"; payload: StagingEntry[] }
  | { type: "SET_INSPECTOR_TAB"; payload: InspectorTab }
  | { type: "UPDATE_RECORD_RATING"; payload: { recordId: string; rating: number; updatedAt: string | null } }
  | {
      type: "UPDATE_RECORD_SECONDARY_RATING";
      payload: { recordId: string; secondaryRating: number | null; updatedAt: string | null };
    }
  | { type: "TOGGLE_INSPECTOR" }
  | { type: "SET_LOADING"; payload: boolean }
  | { type: "SET_ERROR"; payload: string | null }
  | { type: "SET_SEARCH"; payload: string }
  | { type: "SET_BROWSER_TAB"; payload: BrowserTab }
  | { type: "SET_SOURCE_FILTER"; payload: SourceFilter }
  | { type: "SET_TIME_FILTER"; payload: TimeFilter }
  | { type: "SET_MODEL_FILTER"; payload: string | null }
  | { type: "SET_SDK_FILTER"; payload: string | null }
  | { type: "SET_AGENT_HARNESS_FILTERS"; payload: string[] }
  | { type: "SET_AUTHOR_FILTERS"; payload: string[] }
  | { type: "SET_CATEGORY_FILTERS"; payload: string[] }
  | { type: "SET_COST_FILTER"; payload: CostFilter }
  | { type: "SET_RATING_FILTER"; payload: RatingFilter }
  | { type: "SET_SECONDARY_RATING_FILTER"; payload: RatingFilter }
  | { type: "SET_RUN_FILTER"; payload: string | null }
  | { type: "TOGGLE_MULTI_SELECT"; payload: string }
  | { type: "RANGE_MULTI_SELECT"; payload: { targetId: string; visibleIds: string[] } }
  | { type: "SET_MULTI_SELECT_ALL"; payload: string[] }
  | { type: "CLEAR_MULTI_SELECT" };
