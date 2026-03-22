import type { StagingEntry } from "@/lib/types";

const PREVIEW_RUN_ID = "__preview_run__";
const PREVIEW_RECORD_ID = "__preview_record__";

const PREVIEW_STAGING_ENTRY: StagingEntry = {
  run_id: PREVIEW_RUN_ID,
  record_id: PREVIEW_RECORD_ID,
  title: "Folding workshop stool with spring-loaded backrest",
  prompt_preview:
    "Industrial folding stool with a spring-loaded backrest, exposed hinge hardware, and rubber feet.",
  status: "running",
  message: "UI preview entry",
  created_at: "2026-03-19T09:12:00Z",
  updated_at: "2026-03-19T09:18:00Z",
  collection: "preview",
  category_slug: "furniture",
  provider: "openai",
  model_id: "gpt-5.4",
  thinking_level: "high",
  sdk_package: "sdk_hybrid",
  turn_count: 7,
  tool_call_count: 18,
  compile_attempt_count: 2,
  staging_dir: "data/cache/runs/__preview_run__/staging/__preview_record__",
  has_prompt: true,
  has_model_script: true,
  model_script_updated_at: "2026-03-19T09:17:00Z",
  has_checkpoint_urdf: true,
  checkpoint_updated_at: "2026-03-19T09:18:00Z",
  has_cost: true,
  has_traces: true,
  persisted_record: null,
};

export function getPreviewStagingEntries(entries: StagingEntry[]): StagingEntry[] {
  if (entries.length > 0) {
    return entries;
  }
  return [PREVIEW_STAGING_ENTRY];
}

export function isPreviewStagingEntry(entry: StagingEntry): boolean {
  return entry.run_id === PREVIEW_RUN_ID && entry.record_id === PREVIEW_RECORD_ID;
}
