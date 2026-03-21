import type {
  CategoryOption,
  CostFilter,
  DatasetEntry,
  DeleteStagingResult,
  DeleteRecordResult,
  OpenRecordFolderResult,
  OpenStagingFolderResult,
  RecordRatingResponse,
  RecordSummary,
  RepoStats,
  RatingFilter,
  RunDetail,
  StagingEntry,
  ViewerBootstrap,
} from "@/lib/types";

export interface RecordTextFileResult {
  record_id: string;
  file_path: string;
  content: string;
  truncated: boolean;
  byte_count: number;
  preview_byte_limit: number | null;
}

async function readErrorMessage(response: Response): Promise<string> {
  const fallback = `${response.status} ${response.statusText}`;
  const contentType = response.headers.get("content-type") ?? "";
  if (!contentType.includes("application/json")) {
    return fallback;
  }

  try {
    const payload = (await response.json()) as { detail?: unknown; message?: unknown };
    if (typeof payload.detail === "string" && payload.detail.trim()) {
      return payload.detail;
    }
    if (typeof payload.message === "string" && payload.message.trim()) {
      return payload.message;
    }
  } catch {
    return fallback;
  }

  return fallback;
}

async function fetchJson<T>(path: string): Promise<T> {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as T;
}

export async function fetchBootstrap(): Promise<ViewerBootstrap> {
  return fetchJson<ViewerBootstrap>("/api/bootstrap");
}

export async function fetchRepoStats(): Promise<RepoStats> {
  return fetchJson<RepoStats>("/api/stats");
}

export async function fetchCategories(): Promise<CategoryOption[]> {
  return fetchJson<CategoryOption[]>("/api/categories");
}

export async function searchRecords(params: {
  query: string;
  source: "workbench" | "dataset";
  runId: string | null;
  timeFilter: "any" | "24h" | "7d" | "30d" | "90d";
  modelFilter: string | null;
  categoryFilters: string[];
  costFilter: CostFilter;
  ratingFilter: RatingFilter;
  limit?: number;
}): Promise<RecordSummary[]> {
  const searchParams = new URLSearchParams();
  searchParams.set("q", params.query);
  searchParams.set("source", params.source);
  if (params.runId) {
    searchParams.set("run_id", params.runId);
  }
  if (params.timeFilter !== "any") {
    searchParams.set("time", params.timeFilter);
  }
  if (params.modelFilter) {
    searchParams.set("model", params.modelFilter);
  }
  for (const categoryFilter of params.categoryFilters) {
    searchParams.append("category", categoryFilter);
  }
  if (params.costFilter.min != null) {
    searchParams.set("cost_min", String(params.costFilter.min));
  }
  if (params.costFilter.max != null) {
    searchParams.set("cost_max", String(params.costFilter.max));
  }
  for (const ratingFilter of params.ratingFilter) {
    searchParams.append("rating", ratingFilter);
  }
  if (params.limit) {
    searchParams.set("limit", String(params.limit));
  }
  return fetchJson<RecordSummary[]>(`/api/records/search?${searchParams.toString()}`);
}

export async function deleteRecord(recordId: string): Promise<DeleteRecordResult> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}`, {
    method: "DELETE",
  });
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as DeleteRecordResult;
}

export async function promoteRecordToDataset(
  recordId: string,
  params: {
    categorySlug: string;
    categoryTitle?: string | null;
    datasetId?: string | null;
  },
): Promise<DatasetEntry> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/promote`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      category_slug: params.categorySlug.trim(),
      category_title: params.categoryTitle?.trim() ? params.categoryTitle.trim() : null,
      dataset_id: params.datasetId?.trim() ? params.datasetId.trim() : null,
    }),
  });
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as DatasetEntry;
}

export async function deleteStagingEntry(
  runId: string,
  recordId: string,
): Promise<DeleteStagingResult> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}`,
    {
      method: "DELETE",
    },
  );
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as DeleteStagingResult;
}

export async function openRecordFolder(recordId: string): Promise<OpenRecordFolderResult> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/open-folder`, {
    method: "POST",
  });
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as OpenRecordFolderResult;
}

export async function fetchRunDetail(runId: string): Promise<RunDetail> {
  return fetchJson<RunDetail>(`/api/runs/${runId}`);
}

export async function fetchRecordFile(recordId: string, filePath: string): Promise<string> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/files/${filePath}`);
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return response.text();
}

export async function fetchRecordTextFile(
  recordId: string,
  filePath: string,
  options?: {
    full?: boolean;
    previewBytes?: number;
  },
): Promise<RecordTextFileResult> {
  const searchParams = new URLSearchParams();
  if (options?.full) {
    searchParams.set("full", "true");
  }
  if (options?.previewBytes != null) {
    searchParams.set("preview_bytes", String(options.previewBytes));
  }
  const query = searchParams.size > 0 ? `?${searchParams.toString()}` : "";
  return fetchJson<RecordTextFileResult>(
    `/api/records/${encodeURIComponent(recordId)}/text/${filePath}${query}`,
  );
}

export async function fetchRecordTraceFile(recordId: string, filePath: string): Promise<string> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/traces/${filePath}`);
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return response.text();
}

export async function fetchStagingEntries(): Promise<StagingEntry[]> {
  return fetchJson<StagingEntry[]>("/api/staging");
}

export async function fetchStagingFile(runId: string, recordId: string, filePath: string): Promise<string> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}/files/${filePath}`,
  );
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return response.text();
}

export async function fetchStagingTextFile(
  runId: string,
  recordId: string,
  filePath: string,
  options?: {
    full?: boolean;
    previewBytes?: number;
  },
): Promise<RecordTextFileResult> {
  const searchParams = new URLSearchParams();
  if (options?.full) {
    searchParams.set("full", "true");
  }
  if (options?.previewBytes != null) {
    searchParams.set("preview_bytes", String(options.previewBytes));
  }
  const query = searchParams.size > 0 ? `?${searchParams.toString()}` : "";
  return fetchJson<RecordTextFileResult>(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}/text/${filePath}${query}`,
  );
}

export async function fetchStagingTraceFile(runId: string, recordId: string, filePath: string): Promise<string> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}/traces/${filePath}`,
  );
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return response.text();
}

export async function openStagingFolder(runId: string, recordId: string): Promise<OpenStagingFolderResult> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}/open-folder`,
    { method: "POST" },
  );
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as OpenStagingFolderResult;
}

export async function saveRecordRating(recordId: string, rating: number): Promise<RecordRatingResponse> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/rating`, {
    method: "PUT",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ rating }),
  });
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as RecordRatingResponse;
}
