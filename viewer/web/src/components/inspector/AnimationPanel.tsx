import { useEffect, useState, type JSX } from "react";
import { Pause, Play, RotateCcw, SkipBack, SkipForward } from "lucide-react";

import { fetchRecordAnimation } from "@/lib/api";
import type { RecordAnimation } from "@/lib/types";
import { useViewer } from "@/lib/viewer-context";
import { useAnimation } from "@/lib/animation-context";
import { Button } from "@/components/ui/button";
import { Skeleton } from "@/components/ui/skeleton";
import { codeTheme, SyntaxHighlighter } from "@/components/inspector/syntax-highlighting";

const FRAME_MS = 500;

function formatChangeLabel(toolName: string, traceLine: number): string {
  return `${toolName} · trace line ${traceLine}`;
}

function formatFrameStatus(status: string): string | null {
  return status === "fallback" ? "unvalidated render" : null;
}

export function AnimationPanel(): JSX.Element {
  const { selection } = useViewer();
  const { setActiveFrame } = useAnimation();
  const recordId = selection?.kind === "record" ? selection.recordId : null;
  const [animation, setAnimation] = useState<RecordAnimation | null>(null);
  const [activeIndex, setActiveIndex] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    setAnimation(null);
    setActiveIndex(0);
    setPlaying(false);
    setError(null);

    if (!recordId) {
      return;
    }

    setLoading(true);
    fetchRecordAnimation(recordId)
      .then((payload) => {
        if (cancelled) return;
        setAnimation(payload);
      })
      .catch((err: unknown) => {
        if (cancelled) return;
        setError(err instanceof Error ? err.message : "Failed to build animation frames");
      })
      .finally(() => {
        if (!cancelled) {
          setLoading(false);
        }
      });

    return () => {
      cancelled = true;
    };
  }, [recordId]);

  useEffect(() => {
    if (!playing || !animation || animation.frames.length <= 1) {
      return;
    }
    const timer = window.setInterval(() => {
      setActiveIndex((current) => (current + 1) % animation.frames.length);
    }, FRAME_MS);
    return () => window.clearInterval(timer);
  }, [animation, playing]);

  const frame = animation?.frames[activeIndex] ?? null;

  useEffect(() => {
    setActiveFrame(frame);
  }, [frame, setActiveFrame]);

  useEffect(() => {
    return () => setActiveFrame(null);
  }, [recordId, setActiveFrame]);

  if (!recordId) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[10px] text-[var(--text-quaternary)]">Animation is available for records</p>
      </div>
    );
  }

  if (loading && !animation) {
    return (
      <div className="space-y-2">
        <Skeleton className="h-8 w-full" />
        <Skeleton className="h-52 w-full" />
      </div>
    );
  }

  if (error) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="px-3 text-center text-[10px] text-[var(--text-quaternary)]">{error}</p>
      </div>
    );
  }

  if (!animation || animation.frames.length === 0 || !frame) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="px-3 text-center text-[10px] text-[var(--text-quaternary)]">
          No primitive-level changes detected in trace.
        </p>
      </div>
    );
  }

  const hasMultipleFrames = animation.frames.length > 1;
  const lastIndex = animation.frames.length - 1;
  const frameStatus = formatFrameStatus(frame.compile_status);

  return (
    <div className="flex h-full min-h-0 flex-col gap-2">
      <div className="flex shrink-0 items-center gap-2 rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] px-2 py-2">
        <Button
          type="button"
          variant="ghost"
          size="sm"
          className="h-7 w-7 p-0"
          aria-label="Restart"
          onClick={() => setActiveIndex(0)}
          disabled={!hasMultipleFrames}
        >
          <RotateCcw className="size-3.5" />
        </Button>
        <Button
          type="button"
          variant="ghost"
          size="sm"
          className="h-7 w-7 p-0"
          aria-label="Previous frame"
          onClick={() => setActiveIndex((c) => Math.max(0, c - 1))}
          disabled={!hasMultipleFrames}
        >
          <SkipBack className="size-3.5" />
        </Button>
        <Button
          type="button"
          variant="ghost"
          size="sm"
          className="h-7 w-7 p-0"
          aria-label={playing ? "Pause" : "Play"}
          onClick={() => setPlaying((v) => !v)}
          disabled={!hasMultipleFrames}
        >
          {playing ? <Pause className="size-3.5" /> : <Play className="size-3.5" />}
        </Button>
        <Button
          type="button"
          variant="ghost"
          size="sm"
          className="h-7 w-7 p-0"
          aria-label="Next frame"
          onClick={() => setActiveIndex((c) => Math.min(lastIndex, c + 1))}
          disabled={!hasMultipleFrames}
        >
          <SkipForward className="size-3.5" />
        </Button>
        <input
          type="range"
          min={0}
          max={lastIndex}
          value={activeIndex}
          onChange={(event) => setActiveIndex(Number(event.target.value))}
          className="min-w-0 flex-1"
          aria-label="Animation frame"
          disabled={!hasMultipleFrames}
        />
        <span className="w-14 text-right font-mono text-[10px] text-[var(--text-tertiary)]">
          {activeIndex + 1} / {animation.frames.length}
        </span>
      </div>

      <div className="flex min-h-0 flex-1 flex-col overflow-hidden rounded-md border border-[var(--border-default)] bg-[var(--surface-0)]">
        <div className="flex h-8 shrink-0 items-center border-b border-[var(--border-default)] px-3">
          <span className="font-mono text-[10px] text-[var(--text-tertiary)]">
            {formatChangeLabel(frame.tool_name, frame.trace_line)}
          </span>
          {frameStatus ? (
            <span
              className="ml-2 truncate text-[10px] text-[var(--text-quaternary)]"
              title={frame.compile_error ?? undefined}
            >
              {frameStatus}
            </span>
          ) : null}
          {animation.skipped_count > 0 ? (
            <span className="ml-auto text-[10px] text-[var(--text-quaternary)]">
              {animation.skipped_count} skipped
            </span>
          ) : null}
        </div>
        <div className="code-panel-scroll min-h-0 flex-1 overflow-auto">
          <SyntaxHighlighter
            language="python"
            style={codeTheme}
            customStyle={{
              margin: 0,
              padding: "12px",
              background: "transparent",
              fontFamily: "var(--font-mono)",
              fontSize: "11px",
              lineHeight: "1.6",
              minWidth: "100%",
            }}
            codeTagProps={{
              style: {
                fontFamily: "var(--font-mono)",
                whiteSpace: "pre",
              },
            }}
          >
            {frame.code_snippet}
          </SyntaxHighlighter>
        </div>
      </div>
    </div>
  );
}
