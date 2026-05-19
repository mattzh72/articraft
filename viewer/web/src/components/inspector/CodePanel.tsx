import { useCallback, useEffect, useMemo, useRef, useState, type JSX } from "react";

import { useViewer } from "@/lib/viewer-context";
import { fetchRecordTextFile, fetchStagingTextFile } from "@/lib/api";
import { Skeleton } from "@/components/ui/skeleton";
import { cn } from "@/lib/utils";
import { codeTheme, SyntaxHighlighter } from "@/components/inspector/syntax-highlighting";

type CodeTab = "model.py" | "model.urdf";
type CodeLanguage = "python" | "xml";
type FileState = {
  content: string | null;
  loading: boolean;
  error: string | null;
  truncated: boolean;
  byteCount: number;
  previewByteLimit: number | null;
};

const PREVIEW_BYTES = 131072;
const INITIAL_REQUEST_IDS: Record<CodeTab, number> = {
  "model.py": 0,
  "model.urdf": 0,
};

function getLanguage(tab: CodeTab): CodeLanguage {
  return tab === "model.py" ? "python" : "xml";
}

function getLineCount(content: string | null): number {
  if (!content) return 0;
  return content.split("\n").length;
}

function createInitialFileState(): Record<CodeTab, FileState> {
  return {
    "model.py": {
      content: null,
      loading: false,
      error: null,
      truncated: false,
      byteCount: 0,
      previewByteLimit: null,
    },
    "model.urdf": {
      content: null,
      loading: false,
      error: null,
      truncated: false,
      byteCount: 0,
      previewByteLimit: null,
    },
  };
}

function formatBytes(byteCount: number): string {
  if (byteCount < 1024) {
    return `${byteCount} B`;
  }
  if (byteCount < 1024 * 1024) {
    return `${(byteCount / 1024).toFixed(1)} KB`;
  }
  return `${(byteCount / (1024 * 1024)).toFixed(1)} MB`;
}

export function CodePanel(): JSX.Element {
  const { bootstrap, selection } = useViewer();
  const selectedStagingEntry = useMemo(() => {
    if (!bootstrap || selection?.kind !== "staging") {
      return null;
    }
    return (
      bootstrap.staging_entries.find(
        (entry) => entry.run_id === selection.runId && entry.record_id === selection.recordId,
      ) ?? null
    );
  }, [bootstrap, selection]);
  const stagingRevisionKey = selectedStagingEntry
    ? `${selectedStagingEntry.model_script_updated_at ?? ""}|${selectedStagingEntry.checkpoint_updated_at ?? ""}`
    : "";
  const selectionKey = selection
    ? selection.kind === "record"
      ? selection.recordId
      : `staging:${selection.runId}:${selection.recordId}:${stagingRevisionKey}`
    : null;

  const [activeTab, setActiveTab] = useState<CodeTab>("model.py");
  const [fileStates, setFileStates] = useState<Record<CodeTab, FileState>>(createInitialFileState);
  const requestIdsRef = useRef<Record<CodeTab, number>>({ ...INITIAL_REQUEST_IDS });
  const currentSelectionKeyRef = useRef<string | null>(selectionKey);

  const loadTab = useCallback(
    async (tab: CodeTab, full = false) => {
      if (!selection) {
        return;
      }

      const capturedKey = selectionKey;
      requestIdsRef.current[tab] += 1;
      const requestId = requestIdsRef.current[tab];

      setFileStates((prev) => ({
        ...prev,
        [tab]: {
          ...prev[tab],
          loading: true,
          error: null,
        },
      }));

      try {
        const payload = selection.kind === "staging"
          ? await fetchStagingTextFile(selection.runId, selection.recordId, tab, {
              full,
              previewBytes: PREVIEW_BYTES,
            })
          : await fetchRecordTextFile(selection.recordId, tab, {
              full,
              previewBytes: PREVIEW_BYTES,
            });
        if (currentSelectionKeyRef.current !== capturedKey || requestIdsRef.current[tab] !== requestId) {
          return;
        }
        setFileStates((prev) => ({
          ...prev,
          [tab]: {
            content: payload.content,
            loading: false,
            error: null,
            truncated: payload.truncated,
            byteCount: payload.byte_count,
            previewByteLimit: payload.preview_byte_limit,
          },
        }));
      } catch (err) {
        if (currentSelectionKeyRef.current !== capturedKey || requestIdsRef.current[tab] !== requestId) {
          return;
        }
        setFileStates((prev) => ({
          ...prev,
          [tab]: {
            ...prev[tab],
            loading: false,
            error: err instanceof Error ? err.message : "Failed to load",
          },
        }));
      }
    },
    [selection, selectionKey],
  );

  useEffect(() => {
    currentSelectionKeyRef.current = selectionKey;
    requestIdsRef.current = { ...INITIAL_REQUEST_IDS };
    setFileStates(createInitialFileState());
  }, [selectionKey]);

  useEffect(() => {
    if (!selection) {
      return;
    }
    const activeState = fileStates[activeTab];
    if (activeState.loading || activeState.content !== null || activeState.error !== null) {
      return;
    }
    void loadTab(activeTab, false);
  }, [activeTab, fileStates, loadTab, selection]);

  if (!selection) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[10px] text-[var(--text-quaternary)]">Select a record</p>
      </div>
    );
  }

  const activeState = fileStates[activeTab];
  const isLoading = activeState.loading;
  const content = activeState.content;
  const error = activeState.error;
  const lineCount = getLineCount(content);
  const showSkeleton = isLoading && !content;
  const previewLabel =
    activeState.truncated && activeState.previewByteLimit != null
      ? `preview ${formatBytes(activeState.previewByteLimit)} / ${formatBytes(activeState.byteCount)}`
      : null;

  return (
    <div className="flex h-full flex-col overflow-hidden rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)]">
      <div className="flex items-center border-b border-[var(--border-default)] px-2">
        <div className="flex items-center gap-0.5">
          {(["model.py", "model.urdf"] as const).map((tab) => (
            <button
              key={tab}
              type="button"
              onClick={() => setActiveTab(tab)}
              className={cn(
                "relative px-2.5 py-2 font-mono text-[10px] transition-colors duration-150",
                activeTab === tab
                  ? "text-[var(--text-primary)] after:absolute after:bottom-0 after:left-2.5 after:right-2.5 after:h-[1.5px] after:rounded-full after:bg-[var(--text-primary)]"
                  : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]",
              )}
            >
              {tab}
            </button>
          ))}
        </div>
        <div className="ml-auto flex items-center gap-2 font-mono text-[9px] text-[var(--text-quaternary)]">
          {previewLabel ? <span>{previewLabel}</span> : null}
          {activeState.truncated ? (
            <button
              type="button"
              onClick={() => void loadTab(activeTab, true)}
              disabled={isLoading}
              className="rounded border border-[var(--border-default)] px-1.5 py-0.5 text-[var(--text-secondary)] transition-colors duration-150 hover:text-[var(--text-primary)] disabled:cursor-wait disabled:opacity-60"
            >
              {isLoading ? "Loading..." : "Load full"}
            </button>
          ) : null}
          <span className="tabular-nums">{lineCount} lines</span>
        </div>
      </div>

      <div className="min-h-0 flex-1">
        {showSkeleton ? (
          <div className="space-y-1.5 p-3">
            <Skeleton className="h-3 w-3/4" />
            <Skeleton className="h-3 w-full" />
            <Skeleton className="h-3 w-5/6" />
            <Skeleton className="h-3 w-2/3" />
          </div>
        ) : error ? (
          <div className="flex h-24 items-center justify-center">
            <p className="px-3 text-center text-[10px] text-[var(--text-quaternary)]">{error}</p>
          </div>
        ) : content ? (
          <div className="code-panel-scroll min-h-0 h-full overflow-auto bg-[var(--surface-0)]">
            <div className="min-w-max">
              <SyntaxHighlighter
                language={getLanguage(activeTab)}
                style={codeTheme}
                showLineNumbers
                showInlineLineNumbers
                customStyle={{
                  margin: 0,
                  padding: "12px 0",
                  background: "transparent",
                  fontFamily: "var(--font-mono)",
                  fontSize: "10px",
                  lineHeight: "1.7",
                  minWidth: "100%",
                }}
                codeTagProps={{
                  style: {
                    fontFamily: "var(--font-mono)",
                    whiteSpace: "pre",
                  },
                }}
                lineNumberStyle={{
                  minWidth: "3rem",
                  paddingRight: "1rem",
                  textAlign: "right",
                  color: "var(--text-quaternary)",
                  userSelect: "none",
                  borderRight: "1px solid var(--border-subtle)",
                  marginRight: "1rem",
                }}
                lineProps={() => ({
                  style: {
                    display: "block",
                    paddingRight: "1rem",
                    whiteSpace: "pre",
                  },
                })}
              >
                {content}
              </SyntaxHighlighter>
            </div>
          </div>
        ) : (
          <div className="flex h-24 items-center justify-center">
            <p className="text-[10px] text-[var(--text-quaternary)]">No content</p>
          </div>
        )}
      </div>
    </div>
  );
}
