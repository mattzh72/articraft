import type {
  CostFilter,
  DeleteRecordResult,
  DeleteStagingResult,
  OpenRecordFolderResult,
  OpenStagingFolderResult,
  RatingFilter,
  RecordBrowseIdsResponse,
  RecordBrowseResponse,
  RecordHistory,
  RecordRatingResponse,
  RecordSecondaryRatingResponse,
  RecordSummary,
  RepoStats,
  RunDetail,
  SourceFilter,
  StagingEntry,
  TimeFilter,
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

export class HttpError extends Error {
  readonly status: number;

  constructor(status: number, message: string) {
    super(message);
    this.name = "HttpError";
    this.status = status;
  }
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

async function fetchJson<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, init);
  if (!response.ok) {
    throw new HttpError(response.status, await readErrorMessage(response));
  }
  return (await response.json()) as T;
}

export async function fetchBootstrap(): Promise<ViewerBootstrap> {
  return fetchJson<ViewerBootstrap>("/api/bootstrap");
}

export async function fetchRepoStats(): Promise<RepoStats> {
  return fetchJson<RepoStats>("/api/stats");
}

export async function fetchRecordSummary(recordId: string): Promise<RecordSummary> {
  return fetchJson<RecordSummary>(`/api/records/${encodeURIComponent(recordId)}/summary`);
}

export async function fetchRecordHistory(recordId: string): Promise<RecordHistory> {
  return fetchJson<RecordHistory>(`/api/records/${encodeURIComponent(recordId)}/history`);
}

type BrowseParams = {
  source: SourceFilter;
  query: string;
  runId: string | null;
  timeFilter: TimeFilter;
  modelFilter: string | null;
  sdkFilter: string | null;
  agentHarnessFilters: string[];
  authorFilters: string[];
  categoryFilters: string[];
  costFilter: CostFilter;
  ratingFilter: RatingFilter;
  secondaryRatingFilter: RatingFilter;
};

function appendBrowseParams(searchParams: URLSearchParams, params: BrowseParams): void {
  searchParams.set("source", params.source);
  if (params.query.trim()) {
    searchParams.set("q", params.query.trim());
  }
  if (params.runId) {
    searchParams.set("run_id", params.runId);
  }
  if (params.timeFilter.oldest) {
    searchParams.set("time_from", params.timeFilter.oldest);
  }
  if (params.timeFilter.newest) {
    searchParams.set("time_to", params.timeFilter.newest);
  }
  if (params.modelFilter) {
    searchParams.set("model", params.modelFilter);
  }
  if (params.sdkFilter) {
    searchParams.set("sdk", params.sdkFilter);
  }
  for (const agentHarnessFilter of params.agentHarnessFilters) {
    searchParams.append("agent_harness", agentHarnessFilter);
  }
  for (const authorFilter of params.authorFilters) {
    searchParams.append("author", authorFilter);
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
  for (const secondaryRatingFilter of params.secondaryRatingFilter) {
    searchParams.append("secondary_rating", secondaryRatingFilter);
  }
}

export async function browseRecords(
  params: BrowseParams & { offset?: number; limit?: number },
): Promise<RecordBrowseResponse> {
  const searchParams = new URLSearchParams();
  appendBrowseParams(searchParams, params);
  if (params.offset != null) {
    searchParams.set("offset", String(params.offset));
  }
  if (params.limit != null) {
    searchParams.set("limit", String(params.limit));
  }
  return fetchJson<RecordBrowseResponse>(`/api/records/browse?${searchParams.toString()}`);
}

export async function fetchBrowseRecordIds(params: BrowseParams): Promise<RecordBrowseIdsResponse> {
  const searchParams = new URLSearchParams();
  appendBrowseParams(searchParams, params);
  return fetchJson<RecordBrowseIdsResponse>(`/api/records/browse/ids?${searchParams.toString()}`);
}

export async function searchRecords(
  params: BrowseParams & { query: string; limit?: number },
): Promise<RecordSummary[]> {
  const searchParams = new URLSearchParams();
  appendBrowseParams(searchParams, params);
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
    throw new HttpError(response.status, await readErrorMessage(response));
  }
  return (await response.json()) as DeleteRecordResult;
}

export async function deleteStagingEntry(
  runId: string,
  recordId: string,
): Promise<DeleteStagingResult> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}`,
    { method: "DELETE" },
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

export async function openStagingFolder(
  runId: string,
  recordId: string,
): Promise<OpenStagingFolderResult> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}/open-folder`,
    { method: "POST" },
  );
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as OpenStagingFolderResult;
}

export async function fetchRunDetail(runId: string): Promise<RunDetail> {
  return fetchJson<RunDetail>(`/api/runs/${runId}`);
}

export async function fetchRecordFile(recordId: string, filePath: string): Promise<string> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/files/${filePath}`);
  if (!response.ok) {
    throw new HttpError(response.status, await readErrorMessage(response));
  }
  return response.text();
}

export async function fetchRecordTextFile(
  recordId: string,
  filePath: string,
  options?: { full?: boolean; previewBytes?: number },
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
    throw new HttpError(response.status, await readErrorMessage(response));
  }
  return response.text();
}

export function recordRevisionTraceUrl(
  recordId: string,
  revisionId: string,
  filePath = "trajectory.jsonl",
): string {
  return `/api/records/${encodeURIComponent(recordId)}/revisions/${encodeURIComponent(
    revisionId,
  )}/traces/${filePath}`;
}

export async function fetchStagingEntries(): Promise<StagingEntry[]> {
  return fetchJson<StagingEntry[]>("/api/staging");
}

export async function fetchStagingFile(
  runId: string,
  recordId: string,
  filePath: string,
): Promise<string> {
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
  options?: { full?: boolean; previewBytes?: number },
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

export async function fetchStagingTraceFile(
  runId: string,
  recordId: string,
  filePath: string,
): Promise<string> {
  const response = await fetch(
    `/api/staging/${encodeURIComponent(runId)}/${encodeURIComponent(recordId)}/traces/${filePath}`,
  );
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return response.text();
}

export async function saveRecordRating(
  recordId: string,
  rating: number,
): Promise<RecordRatingResponse> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/rating`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ rating }),
  });
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as RecordRatingResponse;
}

export async function saveRecordSecondaryRating(
  recordId: string,
  secondaryRating: number | null,
): Promise<RecordSecondaryRatingResponse> {
  const response = await fetch(`/api/records/${encodeURIComponent(recordId)}/secondary-rating`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ secondary_rating: secondaryRating }),
  });
  if (!response.ok) {
    throw new Error(await readErrorMessage(response));
  }
  return (await response.json()) as RecordSecondaryRatingResponse;
}
