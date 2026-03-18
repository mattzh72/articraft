import { useEffect, useState, type JSX } from "react";
import { LightAsync as SyntaxHighlighter } from "react-syntax-highlighter";
import python from "react-syntax-highlighter/dist/esm/languages/hljs/python";
import xml from "react-syntax-highlighter/dist/esm/languages/hljs/xml";
import { atomOneLight } from "react-syntax-highlighter/dist/esm/styles/hljs";

import { useViewer } from "@/lib/viewer-context";
import { fetchRecordFile } from "@/lib/api";
import { Skeleton } from "@/components/ui/skeleton";
import { cn } from "@/lib/utils";

type CodeTab = "model.py" | "model.urdf";
type CodeLanguage = "python" | "xml";

const codeTheme = {
  ...atomOneLight,
  hljs: {
    ...atomOneLight.hljs,
    background: "transparent",
    color: "#1f2937",
  },
};

SyntaxHighlighter.registerLanguage("python", python);
SyntaxHighlighter.registerLanguage("xml", xml);

function getLanguage(tab: CodeTab): CodeLanguage {
  return tab === "model.py" ? "python" : "xml";
}

function getLineCount(content: string | null): number {
  if (!content) return 0;
  return content.split("\n").length;
}

export function CodePanel(): JSX.Element {
  const { selectedRecordId } = useViewer();
  const [activeTab, setActiveTab] = useState<CodeTab>("model.py");
  const [pyContent, setPyContent] = useState<string | null>(null);
  const [urdfContent, setUrdfContent] = useState<string | null>(null);
  const [pyLoading, setPyLoading] = useState(false);
  const [urdfLoading, setUrdfLoading] = useState(false);
  const [pyError, setPyError] = useState<string | null>(null);
  const [urdfError, setUrdfError] = useState<string | null>(null);

  useEffect(() => {
    if (!selectedRecordId) {
      setPyContent(null);
      setUrdfContent(null);
      setPyError(null);
      setUrdfError(null);
      return;
    }

    let cancelled = false;

    setPyLoading(true);
    setUrdfLoading(true);
    setPyError(null);
    setUrdfError(null);

    fetchRecordFile(selectedRecordId, "model.py")
      .then((text) => {
        if (!cancelled) setPyContent(text);
      })
      .catch((err) => {
        if (!cancelled) {
          setPyContent(null);
          setPyError(err instanceof Error ? err.message : "Failed to load");
        }
      })
      .finally(() => {
        if (!cancelled) setPyLoading(false);
      });

    fetchRecordFile(selectedRecordId, "model.urdf")
      .then((text) => {
        if (!cancelled) setUrdfContent(text);
      })
      .catch((err) => {
        if (!cancelled) {
          setUrdfContent(null);
          setUrdfError(err instanceof Error ? err.message : "Failed to load");
        }
      })
      .finally(() => {
        if (!cancelled) setUrdfLoading(false);
      });

    return () => {
      cancelled = true;
    };
  }, [selectedRecordId]);

  if (!selectedRecordId) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[10px] text-[var(--text-quaternary)]">Select a record</p>
      </div>
    );
  }

  const isLoading = activeTab === "model.py" ? pyLoading : urdfLoading;
  const content = activeTab === "model.py" ? pyContent : urdfContent;
  const error = activeTab === "model.py" ? pyError : urdfError;
  const lineCount = getLineCount(content);

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
        <div className="ml-auto font-mono text-[9px] tabular-nums text-[var(--text-quaternary)]">
          {lineCount} lines
        </div>
      </div>

      <div className="min-h-0 flex-1">
        {isLoading ? (
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
