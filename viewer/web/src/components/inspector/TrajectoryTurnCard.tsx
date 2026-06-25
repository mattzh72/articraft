import { type JSX, type ComponentPropsWithoutRef } from "react";
import Markdown from "react-markdown";
import remarkGfm from "remark-gfm";

import { Badge } from "@/components/ui/badge";
import {
  type EnrichedCompactionEvent,
  type EnrichedGenericTraceEvent,
  type EnrichedTraceEvent,
  type EnrichedTraceTurn,
  type EnrichedTraceMessage,
  type EnrichedToolCall,
  asRecord,
  asNumber,
  formatUsd,
  formatCount,
  fullMessageText,
} from "./trajectory-types";
import {
  codeTheme,
  normalizeHighlightLanguage,
  SyntaxHighlighter,
} from "@/components/inspector/syntax-highlighting";
import {
  formatRelativeTime,
  isPatchText,
  parseCompileModelResult,
  parseEditCodeArgs,
  parseEditCodeResult,
  parseFindExamplesResult,
  parseProbeModelCode,
  parseProbeModelResult,
  parseReadCodeResult,
  parseReadFileArgs,
  parseReadFileResult,
  parseWriteCodeArgs,
  toolCallArguments,
  toolCallName,
  tryPrettyJson,
  type ToolCallContext,
} from "./trajectory-parsing";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Render inline markdown: **bold**, *italic*, `code` */
function renderInlineMarkdown(text: string): JSX.Element {
  const parts: (string | JSX.Element)[] = [];
  const pattern = /(\*\*(.+?)\*\*|\*(.+?)\*|`([^`]+)`)/g;
  let lastIndex = 0;
  let match: RegExpExecArray | null;
  let key = 0;

  while ((match = pattern.exec(text)) !== null) {
    if (match.index > lastIndex) {
      parts.push(text.slice(lastIndex, match.index));
    }
    if (match[2]) {
      parts.push(
        <strong key={key++} className="font-semibold text-[var(--text-primary)]">
          {match[2]}
        </strong>,
      );
    } else if (match[3]) {
      parts.push(<em key={key++}>{match[3]}</em>);
    } else if (match[4]) {
      parts.push(
        <code
          key={key++}
          className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px]"
        >
          {match[4]}
        </code>,
      );
    }
    lastIndex = match.index + match[0].length;
  }

  if (lastIndex < text.length) {
    parts.push(text.slice(lastIndex));
  }

  return <>{parts}</>;
}

// ---------------------------------------------------------------------------
// Sub-renderers
// ---------------------------------------------------------------------------

function ThoughtBlock({ text }: { text: string }): JSX.Element {
  // Normalize: ensure multi-word **Header** patterns get their own paragraph.
  // Only matches bold text with ≥2 words (contains a space) to avoid breaking
  // inline bold like **barrel**.
  const normalized = text.replace(/([^\n])(\s+)(\*\*[^*\n]+ [^*\n]+\*\*)/g, "$1\n\n$3");
  const paragraphs = normalized.split(/\n\n+/).filter((p) => p.trim());

  return (
    <div className="border-l-2 border-[var(--accent)] pl-3">
      <p className="mb-1.5 text-[10px] font-medium text-[var(--accent)]">Reasoning</p>
      <div className="space-y-2 text-[11px] leading-relaxed text-[var(--text-secondary)]">
        {paragraphs.map((para, i) => {
          const trimmed = para.trim();
          if (!trimmed) return null;

          // Paragraph starting with **heading**: extract as styled header
          const headingMatch = /^\*\*(.+?)\*\*(.*)$/s.exec(trimmed);
          if (headingMatch) {
            const heading = headingMatch[1];
            const body = headingMatch[2].trim();
            return (
              <div key={i}>
                <p className="font-semibold text-[var(--text-primary)]">{heading}</p>
                {body ? <p className="mt-0.5">{renderInlineMarkdown(body)}</p> : null}
              </div>
            );
          }

          return <p key={i}>{renderInlineMarkdown(trimmed)}</p>;
        })}
      </div>
    </div>
  );
}

function PatchBlock({ text }: { text: string }): JSX.Element {
  const lines = text.split("\n");
  return (
    <div className="max-h-80 min-w-0 overflow-auto rounded-md bg-[var(--surface-2)]">
      <pre className="py-1.5 font-mono text-[10px] leading-relaxed">
        {lines.map((line, i) => {
          let cls = "text-[var(--text-secondary)]";
          if (line.startsWith("+")) cls = "text-[#1a7f37] bg-[#dafbe1]";
          else if (line.startsWith("-")) cls = "text-[#cf222e] bg-[#ffebe9]";
          else if (line.startsWith("@@")) cls = "text-[#656d76] italic";
          else if (line.startsWith("***")) cls = "text-[#656d76] font-medium";
          return (
            <div key={i} className={`whitespace-pre-wrap break-words px-3 ${cls}`}>
              {line || "\u00a0"}
            </div>
          );
        })}
      </pre>
    </div>
  );
}

function CodeBlock({
  code,
  language = "json",
  maxHeight = "max-h-64",
  startingLineNumber,
}: {
  code: string;
  language?: string;
  maxHeight?: string;
  startingLineNumber?: number;
}): JSX.Element {
  const showLines = startingLineNumber != null;
  return (
    <div className={`${maxHeight} min-w-0 overflow-auto rounded-md bg-[var(--surface-2)]`}>
      <SyntaxHighlighter
        language={language}
        style={codeTheme}
        wrapLongLines
        showLineNumbers={showLines}
        startingLineNumber={startingLineNumber}
        lineNumberStyle={
          showLines
            ? {
                minWidth: "2.5rem",
                paddingRight: "0.75rem",
                textAlign: "right" as const,
                color: "var(--text-quaternary)",
                userSelect: "none" as const,
              }
            : undefined
        }
        customStyle={{
          margin: 0,
          padding: "8px 12px",
          background: "transparent",
          fontFamily: "var(--font-mono)",
          fontSize: "10px",
          lineHeight: "1.6",
        }}
        codeTagProps={{
          style: { fontFamily: "var(--font-mono)" },
        }}
      >
        {code}
      </SyntaxHighlighter>
    </div>
  );
}

function PlainBlock({
  text,
  maxHeight = "max-h-64",
}: {
  text: string;
  maxHeight?: string;
}): JSX.Element {
  return (
    <pre
      className={`${maxHeight} min-w-0 overflow-auto whitespace-pre-wrap break-words rounded-md bg-[var(--surface-2)] px-3 py-2 font-mono text-[10px] leading-relaxed text-[var(--text-secondary)]`}
    >
      {text}
    </pre>
  );
}

function DetailRow({
  label,
  value,
}: {
  label: string;
  value: string;
}): JSX.Element {
  return (
    <div className="space-y-0.5 rounded-md bg-[var(--surface-2)] px-2.5 py-2">
      <p className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-quaternary)]">
        {label}
      </p>
      <p className="break-words font-mono text-[10px] text-[var(--text-secondary)]">{value}</p>
    </div>
  );
}

/** Strip YAML frontmatter (---...---) from markdown content. */
function stripFrontmatter(md: string): string {
  const match = /^---\n[\s\S]*?\n---\n?/.exec(md);
  return match ? md.slice(match[0].length) : md;
}

/** Render markdown content with syntax-highlighted code blocks. */
function MarkdownBlock({
  text,
  maxHeight = "max-h-80",
}: {
  text: string;
  maxHeight?: string;
}): JSX.Element {
  const stripped = stripFrontmatter(text);
  return (
    <div
      className={`${maxHeight} min-w-0 overflow-auto rounded-md bg-[var(--surface-2)] px-3 py-2 text-[11px] leading-relaxed text-[var(--text-secondary)]`}
    >
      <Markdown
        remarkPlugins={[remarkGfm]}
        components={{
          h1: ({ children }: ComponentPropsWithoutRef<"h1">) => (
            <h3 className="mb-1.5 mt-2 text-[12px] font-semibold text-[var(--text-primary)] first:mt-0">
              {children}
            </h3>
          ),
          h2: ({ children }: ComponentPropsWithoutRef<"h2">) => (
            <h4 className="mb-1 mt-2 text-[11px] font-semibold text-[var(--text-primary)]">
              {children}
            </h4>
          ),
          h3: ({ children }: ComponentPropsWithoutRef<"h3">) => (
            <h5 className="mb-1 mt-1.5 text-[11px] font-medium text-[var(--text-primary)]">
              {children}
            </h5>
          ),
          p: ({ children }: ComponentPropsWithoutRef<"p">) => (
            <p className="my-1.5">{children}</p>
          ),
          ul: ({ children }: ComponentPropsWithoutRef<"ul">) => (
            <ul className="my-1.5 list-disc space-y-0.5 pl-4">{children}</ul>
          ),
          ol: ({ children }: ComponentPropsWithoutRef<"ol">) => (
            <ol className="my-1.5 list-decimal space-y-0.5 pl-4">{children}</ol>
          ),
          li: ({ children }: ComponentPropsWithoutRef<"li">) => (
            <li className="text-[11px]">{children}</li>
          ),
          code: ({
            className,
            children,
            ...rest
          }: ComponentPropsWithoutRef<"code"> & { inline?: boolean }) => {
            const langMatch = /language-(\w+)/.exec(className || "");
            const codeStr = String(children).replace(/\n$/, "");
            const language = normalizeHighlightLanguage(langMatch?.[1]);
            if (language) {
              return (
                <div className="my-2 overflow-auto rounded bg-[var(--surface-1)]">
                  <SyntaxHighlighter
                    language={language}
                    style={codeTheme}
                    wrapLongLines
                    customStyle={{
                      margin: 0,
                      padding: "8px 12px",
                      background: "transparent",
                      fontFamily: "var(--font-mono)",
                      fontSize: "10px",
                      lineHeight: "1.6",
                    }}
                    codeTagProps={{ style: { fontFamily: "var(--font-mono)" } }}
                  >
                    {codeStr}
                  </SyntaxHighlighter>
                </div>
              );
            }
            return (
              <code
                className="rounded bg-[var(--surface-1)] px-1 py-0.5 font-mono text-[10px]"
                {...rest}
              >
                {children}
              </code>
            );
          },
          pre: ({ children }: ComponentPropsWithoutRef<"pre">) => <>{children}</>,
        }}
      >
        {stripped}
      </Markdown>
    </div>
  );
}

/** Diff-style view for edit_code old_string → new_string. */
function EditCodeDiffBlock({
  old_string,
  new_string,
  replace_all,
}: {
  old_string: string;
  new_string: string;
  replace_all?: boolean;
}): JSX.Element {
  const oldLines = old_string.split("\n");
  const newLines = new_string.split("\n");
  return (
    <div className="max-h-80 min-w-0 overflow-auto rounded-md bg-[var(--surface-2)]">
      {replace_all ? (
        <div className="px-3 pt-1.5">
          <Badge variant="secondary">replace all</Badge>
        </div>
      ) : null}
      <pre className="py-1.5 font-mono text-[10px] leading-relaxed">
        {oldLines.map((line, i) => (
          <div
            key={`old-${i}`}
            className="whitespace-pre-wrap break-words px-3 text-[#cf222e] bg-[#ffebe9]"
          >
            {`-${line}`}
          </div>
        ))}
        {newLines.map((line, i) => (
          <div
            key={`new-${i}`}
            className="whitespace-pre-wrap break-words px-3 text-[#1a7f37] bg-[#dafbe1]"
          >
            {`+${line}`}
          </div>
        ))}
      </pre>
    </div>
  );
}

function ToolCallBlock({ call }: { call: EnrichedToolCall }): JSX.Element {
  const name = toolCallName(call);
  const rawArgs = toolCallArguments(call);
  const isPatch = name === "apply_patch" || (rawArgs != null && isPatchText(rawArgs));

  // probe_model: show the code snippet with Python highlighting
  if (name === "probe_model" && rawArgs) {
    const probeCode = parseProbeModelCode(rawArgs);
    return (
      <div className="min-w-0 space-y-1.5">
        <p className="text-[11px] text-[var(--text-primary)]">
          <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px]">
            {name}
          </code>
        </p>
        {probeCode ? (
          <CodeBlock code={probeCode} language="python" />
        ) : (
          <PlainBlock text={rawArgs} />
        )}
      </div>
    );
  }

  // edit_code: show old/new as a diff view
  if (name === "edit_code" && rawArgs) {
    const editArgs = parseEditCodeArgs(rawArgs);
    if (editArgs) {
      return (
        <div className="min-w-0 space-y-1.5">
          <p className="text-[11px] text-[var(--text-primary)]">
            <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px]">
              {name}
            </code>
          </p>
          <EditCodeDiffBlock
            old_string={editArgs.old_string}
            new_string={editArgs.new_string}
            replace_all={editArgs.replace_all}
          />
        </div>
      );
    }
  }

  // write_code: show code with Python highlighting
  if (name === "write_code" && rawArgs) {
    const writeCode = parseWriteCodeArgs(rawArgs);
    if (writeCode) {
      return (
        <div className="min-w-0 space-y-1.5">
          <p className="text-[11px] text-[var(--text-primary)]">
            <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px]">
              {name}
            </code>
          </p>
          <CodeBlock code={writeCode} language="python" maxHeight="max-h-80" />
        </div>
      );
    }
  }

  if (name === "read_file" && rawArgs) {
    const readArgs = parseReadFileArgs(rawArgs);
    if (readArgs) {
      return (
        <div className="min-w-0 space-y-1.5">
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[11px] text-[var(--text-primary)]">
              <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px]">
                {name}
              </code>
            </p>
            <Badge variant="secondary">{readArgs.path}</Badge>
            {readArgs.offset != null ? (
              <Badge variant="secondary">offset {readArgs.offset}</Badge>
            ) : null}
            {readArgs.limit != null ? (
              <Badge variant="secondary">limit {readArgs.limit}</Badge>
            ) : null}
          </div>
        </div>
      );
    }
  }

  const prettyArgs = !isPatch && rawArgs ? tryPrettyJson(rawArgs) : null;

  return (
    <div className="min-w-0 space-y-1.5">
      <p className="text-[11px] text-[var(--text-primary)]">
        <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px]">
          {name}
        </code>
      </p>
      {isPatch && rawArgs ? (
        <PatchBlock text={rawArgs} />
      ) : rawArgs ? (
        prettyArgs ? (
          <CodeBlock code={prettyArgs} />
        ) : (
          <PlainBlock text={rawArgs} />
        )
      ) : null}
    </div>
  );
}

function AssistantEvent({ event }: { event: EnrichedTraceMessage }): JSX.Element {
  const contentText = fullMessageText(event.content);
  return (
    <div className="min-w-0 space-y-2">
      {event.thought_summary ? <ThoughtBlock text={event.thought_summary} /> : null}
      {contentText ? (
        <p className="min-w-0 whitespace-pre-wrap break-words text-[11px] leading-relaxed text-[var(--text-secondary)]">
          {contentText}
        </p>
      ) : null}
      {event.tool_calls?.map((call, i) => (
        <ToolCallBlock key={call.id ?? i} call={call} />
      ))}
    </div>
  );
}

function ToolEvent({
  event,
  toolCallContext,
}: {
  event: EnrichedTraceMessage;
  toolCallContext?: ToolCallContext;
}): JSX.Element {
  const raw = fullMessageText(event.content);

  if (event.name === "compile_model" && raw) {
    const parsed = parseCompileModelResult(raw);
    if (parsed) {
      const statusVariant =
        parsed.status === "success"
          ? "success"
          : parsed.status === "error"
            ? "destructive"
            : "secondary";

      return (
        <div className="min-w-0 space-y-2">
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[11px] text-[var(--text-tertiary)]">
              <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
                {event.name}
              </code>{" "}
              result
            </p>
            <Badge variant={statusVariant}>
              {parsed.status === "success"
                ? "compile passed"
                : parsed.status === "error"
                  ? "compile failed"
                  : "compile"}
            </Badge>
            {parsed.failures.length > 0 ? (
              <Badge variant="destructive">{parsed.failures.length} failures</Badge>
            ) : null}
            {parsed.warnings.length > 0 ? (
              <Badge variant="secondary">{parsed.warnings.length} warnings</Badge>
            ) : null}
            {parsed.notes.length > 0 ? (
              <Badge variant="secondary">{parsed.notes.length} notes</Badge>
            ) : null}
          </div>
          {parsed.summary ? (
            <PlainBlock text={parsed.summary} maxHeight="max-h-32" />
          ) : null}
          {parsed.failures.length > 0 ? (
            <div className="space-y-1">
              <p className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--destructive)]">
                Failures
              </p>
              <PlainBlock
                text={parsed.failures.map((item) => `- ${item}`).join("\n")}
                maxHeight="max-h-48"
              />
            </div>
          ) : null}
          {parsed.warnings.length > 0 ? (
            <div className="space-y-1">
              <p className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--warning)]">
                Warnings
              </p>
              <PlainBlock
                text={parsed.warnings.map((item) => `- ${item}`).join("\n")}
                maxHeight="max-h-48"
              />
            </div>
          ) : null}
          {parsed.notes.length > 0 ? (
            <div className="space-y-1">
              <p className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">
                Notes
              </p>
              <PlainBlock
                text={parsed.notes.map((item) => `- ${item}`).join("\n")}
                maxHeight="max-h-48"
              />
            </div>
          ) : null}
          {!parsed.summary && parsed.error ? (
            <PlainBlock text={parsed.error} maxHeight="max-h-32" />
          ) : null}
        </div>
      );
    }
  }

  if (event.name === "find_examples" && raw) {
    const matches = parseFindExamplesResult(raw);
    if (matches) {
      return (
        <div className="min-w-0 space-y-2">
          <p className="text-[11px] text-[var(--text-tertiary)]">
            <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
              {event.name}
            </code>{" "}
            result
          </p>
          <div className="space-y-2">
            {matches.map((match) => (
              <div
                key={match.path}
                className="space-y-2 rounded-md border border-[var(--border)] bg-[var(--surface-1)] p-3"
              >
                <div className="space-y-1">
                  <div className="flex flex-wrap items-center gap-2">
                    <p className="text-[11px] font-medium text-[var(--text-primary)]">
                      {match.title}
                    </p>
                    <Badge variant="secondary">{match.path}</Badge>
                  </div>
                  <p className="text-[11px] leading-relaxed text-[var(--text-secondary)]">
                    {match.description}
                  </p>
                  {match.tags.length > 0 ? (
                    <div className="flex flex-wrap gap-1">
                      {match.tags.map((tag) => (
                        <Badge key={`${match.path}-${tag}`} variant="secondary">
                          {tag}
                        </Badge>
                      ))}
                    </div>
                  ) : null}
                </div>
                <MarkdownBlock text={match.content} maxHeight="max-h-80" />
              </div>
            ))}
          </div>
        </div>
      );
    }
  }

  // probe_model results: show status badge + formatted result JSON
  if (event.name === "probe_model" && raw) {
    const parsed = parseProbeModelResult(raw);
    if (parsed) {
      const resultJson = JSON.stringify(parsed.result, null, 2);
      return (
        <div className="min-w-0 space-y-1.5">
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[11px] text-[var(--text-tertiary)]">
              <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
                {event.name}
              </code>{" "}
              result
            </p>
            <Badge variant={parsed.ok ? "success" : "destructive"}>
              {parsed.ok ? "ok" : "error"}
            </Badge>
            {typeof parsed.elapsedMs === "number" ? (
              <span className="font-mono text-[10px] text-[var(--text-quaternary)]">
                {parsed.elapsedMs.toFixed(0)}ms
              </span>
            ) : null}
          </div>
          <CodeBlock code={resultJson} maxHeight="max-h-80" />
        </div>
      );
    }
  }

  // read_file results: extract code and highlight as Python
  if (event.name === "read_file" && raw) {
    const parsed = parseReadFileResult(raw);
    if (parsed) {
      const readArgs =
        toolCallContext?.name === "read_file" && toolCallContext.rawArgs
          ? parseReadFileArgs(toolCallContext.rawArgs)
          : null;
      const path = readArgs?.path ?? null;
      const isMarkdown = path?.endsWith(".md") ?? false;
      const isPython = path?.endsWith(".py") ?? false;
      return (
        <div className="min-w-0 space-y-1.5">
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[11px] text-[var(--text-tertiary)]">
              <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
                {event.name}
              </code>{" "}
              result
            </p>
            {path ? <Badge variant="secondary">{path}</Badge> : null}
          </div>
          {isMarkdown ? (
            <MarkdownBlock text={parsed.code} maxHeight="max-h-80" />
          ) : isPython ? (
            <CodeBlock
              code={parsed.code}
              language="python"
              maxHeight="max-h-80"
              startingLineNumber={parsed.startLine}
            />
          ) : (
            <PlainBlock text={parsed.code} maxHeight="max-h-80" />
          )}
        </div>
      );
    }
  }

  // read_code results: extract code and highlight as Python
  if (event.name === "read_code" && raw) {
    const parsed = parseReadCodeResult(raw);
    if (parsed) {
      return (
        <div className="min-w-0 space-y-1.5">
          <p className="text-[11px] text-[var(--text-tertiary)]">
            <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
              {event.name}
            </code>{" "}
            result
          </p>
          <CodeBlock code={parsed} language="python" maxHeight="max-h-80" />
        </div>
      );
    }
  }

  // edit_code results: show compilation status badge + error
  if ((event.name === "edit_code" || event.name === "write_code") && raw) {
    const parsed = parseEditCodeResult(raw);
    if (parsed) {
      const statusVariant =
        parsed.status === "success"
          ? "success"
          : parsed.status === "error"
            ? "destructive"
            : "secondary";
      return (
        <div className="min-w-0 space-y-1.5">
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[11px] text-[var(--text-tertiary)]">
              <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
                {event.name}
              </code>{" "}
              result
            </p>
            {parsed.status !== "unknown" ? (
              <Badge variant={statusVariant}>
                {parsed.status === "success" ? "compiled" : "compile error"}
              </Badge>
            ) : null}
          </div>
          <p className="text-[11px] text-[var(--text-secondary)]">{parsed.message}</p>
          {parsed.error ? <PlainBlock text={parsed.error} maxHeight="max-h-48" /> : null}
        </div>
      );
    }
  }

  const prettyJson = raw ? tryPrettyJson(raw) : null;

  return (
    <div className="min-w-0 space-y-1.5">
      <p className="text-[11px] text-[var(--text-tertiary)]">
        <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
          {event.name ?? "tool"}
        </code>{" "}
        result
      </p>
      {raw ? (
        prettyJson ? (
          <CodeBlock code={prettyJson} maxHeight="max-h-80" />
        ) : (
          <PlainBlock text={raw} maxHeight="max-h-80" />
        )
      ) : null}
    </div>
  );
}

function UserEvent({ event }: { event: EnrichedTraceMessage }): JSX.Element {
  const text = fullMessageText(event.content);
  return (
    <div className="min-w-0">
      <p className="mb-1 text-[10px] font-medium text-[var(--text-tertiary)]">Feedback</p>
      {text ? <PlainBlock text={text} /> : null}
    </div>
  );
}

function CompactionEvent({ event }: { event: EnrichedCompactionEvent }): JSX.Element {
  return (
    <div className="min-w-0 space-y-2 rounded-md border border-[var(--border-default)] bg-[var(--surface-1)] p-3">
      <div className="flex flex-wrap items-center gap-2">
        <p className="text-[11px] text-[var(--text-tertiary)]">
          <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
            {event.eventType}
          </code>
        </p>
        {event.trigger ? <Badge variant="warning">{event.trigger.replaceAll("_", " ")}</Badge> : null}
        {event.estimatedSavedNextInputTokens != null ? (
          <Badge variant="secondary">
            {formatCount(event.estimatedSavedNextInputTokens)} tok saved
          </Badge>
        ) : null}
        {event.billedCostUsd != null ? (
          <Badge variant="success">{formatUsd(event.billedCostUsd)}</Badge>
        ) : null}
        <Badge variant={event.previousResponseIdCleared ? "success" : "secondary"}>
          {event.previousResponseIdCleared
            ? "previous_response_id cleared"
            : "previous_response_id kept"}
        </Badge>
      </div>

      <div className="grid gap-2 sm:grid-cols-2">
        <DetailRow label="Trigger" value={event.trigger ?? "--"} />
        <DetailRow
          label="Tokens Saved Estimate"
          value={
            event.estimatedSavedNextInputTokens != null
              ? formatCount(event.estimatedSavedNextInputTokens)
              : "--"
          }
        />
        <DetailRow label="Billed Cost" value={formatUsd(event.billedCostUsd)} />
        <DetailRow
          label="Previous Response ID"
          value={event.previousResponseIdCleared ? "cleared" : "kept"}
        />
      </div>

      {event.estimateError ? (
        <p className="text-[10px] leading-relaxed text-[var(--warning)]">
          Saved-token estimate unavailable: {event.estimateError}
        </p>
      ) : null}
    </div>
  );
}

function GenericTraceEvent({ event }: { event: EnrichedGenericTraceEvent }): JSX.Element {
  const prettyJson = tryPrettyJson(JSON.stringify(event.raw));
  return (
    <div className="min-w-0 space-y-1.5 rounded-md border border-[var(--border-subtle)] bg-[var(--surface-1)] p-3">
      <div className="flex flex-wrap items-center gap-2">
        <p className="text-[11px] text-[var(--text-tertiary)]">
          <code className="rounded bg-[var(--surface-2)] px-1 py-0.5 font-mono text-[10px] text-[var(--text-secondary)]">
            {event.eventType}
          </code>
        </p>
        <Badge variant="secondary">{event.title}</Badge>
      </div>
      {prettyJson ? (
        <CodeBlock code={prettyJson} maxHeight="max-h-80" />
      ) : (
        <PlainBlock text={JSON.stringify(event.raw, null, 2)} maxHeight="max-h-80" />
      )}
    </div>
  );
}

function EventRow({
  event,
  toolCallContextById,
}: {
  event: EnrichedTraceEvent;
  toolCallContextById: Map<string, ToolCallContext>;
}): JSX.Element {
  if (event.kind === "compaction") return <CompactionEvent event={event} />;
  if (event.kind === "event") return <GenericTraceEvent event={event} />;
  if (event.role === "assistant") return <AssistantEvent event={event} />;
  if (event.role === "tool") {
    const toolCallContext =
      event.tool_call_id != null ? toolCallContextById.get(event.tool_call_id) : undefined;
    return <ToolEvent event={event} toolCallContext={toolCallContext} />;
  }
  return <UserEvent event={event} />;
}

// ---------------------------------------------------------------------------
// Main card
// ---------------------------------------------------------------------------

type TrajectoryTurnCardProps = {
  turn: EnrichedTraceTurn;
  baseTimestamp: number | null;
};

export function TrajectoryTurnCard({ turn, baseTimestamp }: TrajectoryTurnCardProps): JSX.Element {
  const turnTokens = asRecord(turn.cost?.tokens);
  const turnCosts = asRecord(turn.cost?.costs_usd);
  const relTime = formatRelativeTime(turn.timestamp, baseTimestamp);

  const eventUsage = turn.events.reduce(
    (acc, ev) => {
      if (ev.usage?.total_tokens) acc.total += ev.usage.total_tokens;
      if (ev.kind === "compaction" && ev.billedCostUsd != null) {
        acc.billedCost += ev.billedCostUsd;
      }
      return acc;
    },
    { total: 0, billedCost: 0 },
  );
  const tokenDisplay =
    asNumber(turnTokens?.total_tokens) != null
      ? formatCount(turnTokens?.total_tokens)
      : eventUsage.total > 0
        ? formatCount(eventUsage.total)
        : null;
  const costDisplay =
    asNumber(turnCosts?.total) != null
      ? formatUsd(turnCosts?.total)
      : eventUsage.billedCost > 0
        ? formatUsd(eventUsage.billedCost)
        : null;
  const toolCallContextById = new Map<string, ToolCallContext>();
  for (const event of turn.events) {
    if (event.kind !== "message" || event.role !== "assistant") continue;
    for (const call of event.tool_calls ?? []) {
      if (!call.id) continue;
      toolCallContextById.set(call.id, {
        name: toolCallName(call),
        rawArgs: toolCallArguments(call),
      });
    }
  }

  return (
    <div className="min-w-0">
      {/* Header row */}
      <div className="flex flex-wrap items-center gap-2 pb-2">
        <span className="font-mono text-[11px] font-medium text-[var(--text-primary)]">
          Turn {turn.index}
        </span>
        {costDisplay ? <Badge variant="success">{costDisplay}</Badge> : null}
        {tokenDisplay ? <Badge variant="secondary">{tokenDisplay} tok</Badge> : null}
        {relTime ? (
          <span className="font-mono text-[10px] text-[var(--text-quaternary)]">{relTime}</span>
        ) : null}
      </div>

      {/* Events — flat list */}
      {turn.events.length > 0 ? (
        <div className="min-w-0 space-y-3 pl-3">
          {turn.events.map((event, eventIndex) => (
            <div key={`${turn.index}-${eventIndex}`} className="min-w-0">
              <EventRow event={event} toolCallContextById={toolCallContextById} />
            </div>
          ))}
        </div>
      ) : (
        <p className="py-1 text-[10px] text-[var(--text-quaternary)]">
          Trace unavailable for this turn.
        </p>
      )}
    </div>
  );
}
