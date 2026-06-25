import { asRecord, type EnrichedToolCall } from "./trajectory-types";

export type FindExamplesResultMatch = {
  title: string;
  description: string;
  tags: string[];
  path: string;
  content: string;
};

export type ParsedCompileModelResult = {
  status: "success" | "error" | "unknown";
  summary: string | null;
  failures: string[];
  warnings: string[];
  notes: string[];
  error: string | null;
};

export type ToolCallContext = {
  name: string;
  rawArgs: string | null;
};

export function toolCallName(call: EnrichedToolCall): string {
  if (call.function?.name) return call.function.name;
  if (call.custom?.name) return call.custom.name;
  return "unknown";
}

export function toolCallArguments(call: EnrichedToolCall): string | null {
  if (call.function?.arguments) return call.function.arguments;
  if (call.custom?.input) return call.custom.input;
  return null;
}

export function formatRelativeTime(
  ts: number | null | undefined,
  baseTs: number | null,
): string {
  if (ts == null || baseTs == null) return "";
  const delta = ts - baseTs;
  if (delta < 0) return "";
  if (delta < 60) return `+${delta.toFixed(1)}s`;
  const minutes = Math.floor(delta / 60);
  const seconds = delta % 60;
  return `+${minutes}m ${seconds.toFixed(0)}s`;
}

export function tryPrettyJson(raw: string): string | null {
  try {
    const parsed = JSON.parse(raw);
    return JSON.stringify(parsed, null, 2);
  } catch {
    return null;
  }
}

export function isPatchText(text: string): boolean {
  const trimmed = text.trimStart();
  return trimmed.startsWith("*** Begin Patch") || trimmed.startsWith("*** Update File");
}

export function parseReadFileResult(raw: string): { code: string; startLine: number } | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const result = typeof parsed.result === "string" ? parsed.result : null;
    if (!result) return null;

    const lines = result.split("\n");
    const linePattern = /^L(\d+):\s?/;
    const firstMatch = linePattern.exec(lines[0]);
    if (!firstMatch) return null;

    const startLine = parseInt(firstMatch[1], 10);
    const code = lines
      .map((line) => {
        const match = linePattern.exec(line);
        return match ? line.slice(match[0].length) : line;
      })
      .join("\n");

    return { code, startLine };
  } catch {
    return null;
  }
}

export function parseReadFileArgs(
  raw: string,
): { path: string; offset?: number; limit?: number } | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const path = typeof parsed.path === "string" ? parsed.path : null;
    if (!path) return null;
    return {
      path,
      offset: typeof parsed.offset === "number" ? parsed.offset : undefined,
      limit: typeof parsed.limit === "number" ? parsed.limit : undefined,
    };
  } catch {
    return null;
  }
}

export function parseFindExamplesResult(raw: string): FindExamplesResultMatch[] | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const result = Array.isArray(parsed.result) ? parsed.result : null;
    if (!result) return null;

    const matches = result
      .map((item) => asRecord(item))
      .filter((item): item is Record<string, unknown> => item !== null)
      .map((item) => {
        const title = typeof item.title === "string" ? item.title : null;
        const description = typeof item.description === "string" ? item.description : null;
        const path = typeof item.path === "string" ? item.path : null;
        const content = typeof item.content === "string" ? item.content : null;
        const tags = Array.isArray(item.tags)
          ? item.tags.filter((tag): tag is string => typeof tag === "string")
          : [];
        if (!title || !description || !path || !content) return null;
        return { title, description, path, content, tags };
      })
      .filter((item): item is FindExamplesResultMatch => item !== null);

    return matches.length > 0 ? matches : null;
  } catch {
    return null;
  }
}

function extractTaggedBlock(text: string, tag: string): string | null {
  const startToken = `<${tag}>`;
  const endToken = `</${tag}>`;
  const start = text.indexOf(startToken);
  if (start < 0) return null;
  const contentStart = start + startToken.length;
  const end = text.indexOf(endToken, contentStart);
  if (end < 0) return null;
  return text.slice(contentStart, end).trim();
}

function extractSignalBullets(section: string | null): string[] {
  if (!section) return [];
  return section
    .split("\n")
    .map((line) => line.trim())
    .filter((line) => line.startsWith("- "))
    .map((line) => line.slice(2).trim())
    .filter(Boolean);
}

export function parseCompileModelResult(raw: string): ParsedCompileModelResult | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const result = typeof parsed.result === "string" ? parsed.result : null;
    const compilation = asRecord(parsed.compilation);
    const status =
      compilation?.status === "success" || compilation?.status === "error"
        ? compilation.status
        : "unknown";

    return {
      status,
      summary: result ? extractTaggedBlock(result, "summary") : null,
      failures: result ? extractSignalBullets(extractTaggedBlock(result, "failures")) : [],
      warnings: result ? extractSignalBullets(extractTaggedBlock(result, "warnings")) : [],
      notes: result ? extractSignalBullets(extractTaggedBlock(result, "notes")) : [],
      error: typeof compilation?.error === "string" ? compilation.error : null,
    };
  } catch {
    return null;
  }
}

export function parseProbeModelCode(raw: string): string | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    return typeof parsed.code === "string" ? parsed.code : null;
  } catch {
    return null;
  }
}

export function parseProbeModelResult(
  raw: string,
): { ok: boolean; elapsedMs?: number; result: unknown } | null {
  try {
    const outer = JSON.parse(raw) as Record<string, unknown>;
    const inner = asRecord(outer.result);
    if (!inner) return null;
    const ok = typeof inner.ok === "boolean" ? inner.ok : true;
    const elapsedMs = typeof inner.elapsed_ms === "number" ? inner.elapsed_ms : undefined;
    return { ok, elapsedMs, result: inner.result ?? inner.error ?? null };
  } catch {
    return null;
  }
}

export function parseReadCodeResult(raw: string): string | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    return typeof parsed.result === "string" ? parsed.result : null;
  } catch {
    return null;
  }
}

export function parseEditCodeArgs(
  raw: string,
): { old_string: string; new_string: string; replace_all?: boolean } | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const old_string = typeof parsed.old_string === "string" ? parsed.old_string : null;
    const new_string = typeof parsed.new_string === "string" ? parsed.new_string : null;
    if (old_string == null || new_string == null) return null;
    const replace_all =
      typeof parsed.replace_all === "boolean" ? parsed.replace_all : undefined;
    return { old_string, new_string, replace_all };
  } catch {
    return null;
  }
}

export function parseEditCodeResult(
  raw: string,
): { message: string; status: "success" | "error" | "unknown"; error: string | null } | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const message = typeof parsed.result === "string" ? parsed.result : null;
    if (!message) return null;
    const compilation = asRecord(parsed.compilation);
    const status =
      compilation?.status === "success" || compilation?.status === "error"
        ? compilation.status
        : "unknown";
    const error = typeof compilation?.error === "string" ? compilation.error : null;
    return { message, status, error };
  } catch {
    return null;
  }
}

export function parseWriteCodeArgs(raw: string): string | null {
  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    return typeof parsed.code === "string" ? parsed.code : null;
  } catch {
    return null;
  }
}
