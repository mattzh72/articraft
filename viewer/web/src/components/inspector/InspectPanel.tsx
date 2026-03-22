import { useCallback, useEffect, useMemo, useState, type JSX } from "react";
import { Copy, FolderOpen, RotateCcw, Star } from "lucide-react";

import { fetchRecordFile, fetchStagingFile, openRecordFolder, openStagingFolder, saveRecordRating } from "@/lib/api";
import { buildRecordPath, copyTextToClipboard } from "@/lib/record-path";
import { findRecordInBootstrap, findStagingEntryInBootstrap } from "@/lib/record-summary";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Skeleton } from "@/components/ui/skeleton";
import { Slider } from "@/components/ui/slider";
import { Tooltip, TooltipTrigger, TooltipContent } from "@/components/ui/tooltip";
import type { UrdfJoint } from "@/components/inspector/JointSlider";

type InspectPanelProps = {
  urdfSpec: { joints: UrdfJoint[] } | null;
  jointValues: Map<string, number>;
  onJointChange: (name: string, value: number) => void;
  onResetAll: () => void;
};

function isMovableJoint(joint: UrdfJoint): boolean {
  return joint.type !== "fixed";
}

function jointRange(joint: UrdfJoint): [number, number] {
  if (joint.type === "continuous") {
    return [-Math.PI, Math.PI];
  }
  return [joint.limit?.lower ?? 0, joint.limit?.upper ?? 0];
}

function formatJointValue(joint: UrdfJoint, value: number): string {
  if (joint.type === "revolute" || joint.type === "continuous") {
    return `${(value * (180 / Math.PI)).toFixed(1)}°`;
  }
  return `${value.toFixed(3)}m`;
}

function jointBadgeVariant(joint: UrdfJoint): "secondary" | "success" {
  return isMovableJoint(joint) ? "success" : "secondary";
}

function SectionLabel({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <div className="flex items-center gap-2 pb-2">
      <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">{children}</span>
      <div className="h-px flex-1 bg-[var(--border-subtle)]" />
    </div>
  );
}

function isEditableTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) {
    return false;
  }
  const tagName = target.tagName.toLowerCase();
  return target.isContentEditable || tagName === "input" || tagName === "textarea" || tagName === "select";
}

function buildRootLinks(joints: UrdfJoint[]): string[] {
  const parents = new Set<string>();
  const children = new Set<string>();

  for (const joint of joints) {
    parents.add(joint.parent);
    children.add(joint.child);
  }

  const roots = Array.from(parents).filter((name) => !children.has(name));
  if (roots.length > 0) return roots;
  return Array.from(new Set(joints.map((joint) => joint.parent)));
}

type LinkBranchProps = {
  linkName: string;
  jointsByParent: Map<string, UrdfJoint[]>;
  jointValues: Map<string, number>;
  onJointChange: (name: string, value: number) => void;
  seen: Set<string>;
};

function LinkBranch({ linkName, jointsByParent, jointValues, onJointChange, seen }: LinkBranchProps): JSX.Element {
  const childJoints = jointsByParent.get(linkName) ?? [];

  return (
    <div className="space-y-1">
      {/* Link node */}
      <div className="flex items-center gap-2 py-1">
        <span className="size-[5px] shrink-0 rounded-full bg-[var(--border-strong)]" />
        <span className="truncate font-mono text-[11px] text-[var(--text-primary)]" title={linkName}>
          {linkName}
        </span>
        <span className="ml-auto shrink-0 font-mono text-[9.5px] tabular-nums text-[var(--text-quaternary)]">
          {childJoints.length > 0 ? `${childJoints.length}` : ""}
        </span>
      </div>

      {childJoints.length > 0 && (
        <div className="ml-1 space-y-1 border-l border-[var(--border-subtle)] pl-4">
          {childJoints.map((joint) => {
            const nextSeen = new Set(seen);
            const canDescend = !seen.has(joint.child);
            nextSeen.add(joint.child);
            const [min, max] = jointRange(joint);
            const value = jointValues.get(joint.name) ?? 0;

            return (
              <div key={joint.name} className="space-y-1">
                {/* Joint node */}
                <div className="rounded-lg bg-[var(--surface-1)] px-3 py-2">
                  <div className="flex items-center justify-between gap-2">
                    <div className="min-w-0">
                      <p className="truncate font-mono text-[11px] text-[var(--text-primary)]" title={joint.name}>
                        {joint.name}
                      </p>
                    </div>
                    <Badge variant={jointBadgeVariant(joint)}>{joint.type}</Badge>
                  </div>

                  {isMovableJoint(joint) ? (
                    <div className="mt-2.5 space-y-1.5">
                      <Slider
                        min={min}
                        max={max}
                        step={0.01}
                        value={[value]}
                        onValueChange={([nextValue]) => onJointChange(joint.name, nextValue)}
                      />
                      <div className="flex justify-between font-mono text-[9.5px] text-[var(--text-quaternary)]">
                        <span>{formatJointValue(joint, min)}</span>
                        <span className="text-[var(--text-secondary)]">{formatJointValue(joint, value)}</span>
                        <span>{formatJointValue(joint, max)}</span>
                      </div>
                    </div>
                  ) : null}
                </div>

                {canDescend ? (
                  <LinkBranch
                    linkName={joint.child}
                    jointsByParent={jointsByParent}
                    jointValues={jointValues}
                    onJointChange={onJointChange}
                    seen={nextSeen}
                  />
                ) : (
                  <div className="ml-4 py-1 text-[10px] text-[var(--text-quaternary)]">
                    Cycle detected at {joint.child}
                  </div>
                )}
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}

export function InspectPanel({
  urdfSpec,
  jointValues,
  onJointChange,
  onResetAll,
}: InspectPanelProps): JSX.Element {
  const { bootstrap, selectedRecordId, selection } = useViewer();
  const dispatch = useViewerDispatch();
  const [promptText, setPromptText] = useState<string | null>(null);
  const [promptStatus, setPromptStatus] = useState<"idle" | "loading" | "loaded" | "unavailable">("idle");
  const [hoveredRating, setHoveredRating] = useState<number | null>(null);
  const [savingRating, setSavingRating] = useState(false);
  const [ratingError, setRatingError] = useState<string | null>(null);
  const [copyState, setCopyState] = useState<"idle" | "copied" | "error">("idle");
  const [openState, setOpenState] = useState<"idle" | "opened" | "error">("idle");

  const isStaging = selection?.kind === "staging";
  const stagingEntry = isStaging
    ? findStagingEntryInBootstrap(bootstrap, selection.runId, selection.recordId)
    : null;
  const record = selectedRecordId ? findRecordInBootstrap(bootstrap, selectedRecordId) : null;
  const joints = useMemo(() => urdfSpec?.joints ?? [], [urdfSpec?.joints]);
  const movableJointCount = joints.filter(isMovableJoint).length;
  const recordPath = bootstrap && selectedRecordId ? buildRecordPath(bootstrap.repo_root, selectedRecordId) : null;
  const stagingSelectionKey =
    selection?.kind === "staging" ? `${selection.runId}:${selection.recordId}` : null;
  const recordSelectionKey = selection?.kind === "record" ? selection.recordId : null;
  const hasSelectedRecord = Boolean(selectedRecordId && record);
  const stagingRecordId = stagingEntry?.record_id ?? null;
  const stagingRunId = stagingEntry?.run_id ?? null;
  const stagingHasPrompt = stagingEntry?.has_prompt ?? false;

  const { rootLinks, jointsByParent } = useMemo(() => {
    const nextMap = new Map<string, UrdfJoint[]>();
    for (const joint of joints) {
      const current = nextMap.get(joint.parent) ?? [];
      current.push(joint);
      nextMap.set(joint.parent, current);
    }
    return {
      rootLinks: buildRootLinks(joints),
      jointsByParent: nextMap,
    };
  }, [joints]);

  useEffect(() => {
    if (isStaging && stagingRunId && stagingRecordId) {
      // Load prompt for staging entry
      if (!stagingHasPrompt) {
        setPromptText(null);
        setPromptStatus("unavailable");
        return;
      }

      let cancelled = false;
      setPromptText(null);
      setPromptStatus("loading");

      fetchStagingFile(stagingRunId, stagingRecordId, "prompt.txt")
        .then((text) => {
          if (cancelled) return;
          const normalized = text.trim();
          setPromptText(normalized || null);
          setPromptStatus(normalized ? "loaded" : "unavailable");
        })
        .catch(() => {
          if (!cancelled) {
            setPromptText(null);
            setPromptStatus("unavailable");
          }
        });

      return () => { cancelled = true; };
    }

    if (!selectedRecordId || !hasSelectedRecord) {
      setPromptText(null);
      setPromptStatus("idle");
      return;
    }

    let cancelled = false;
    setPromptText(null);
    setPromptStatus("loading");

    fetchRecordFile(selectedRecordId, "prompt.txt")
      .then((text) => {
        if (cancelled) {
          return;
        }
        const normalized = text.trim();
        setPromptText(normalized || null);
        setPromptStatus(normalized ? "loaded" : "unavailable");
      })
      .catch(() => {
        if (!cancelled) {
          setPromptText(null);
          setPromptStatus("unavailable");
        }
      });

    return () => {
      cancelled = true;
    };
  }, [
    hasSelectedRecord,
    isStaging,
    recordSelectionKey,
    selectedRecordId,
    stagingHasPrompt,
    stagingRecordId,
    stagingRunId,
    stagingSelectionKey,
  ]);

  useEffect(() => {
    setHoveredRating(null);
    setSavingRating(false);
    setRatingError(null);
    setCopyState("idle");
    setOpenState("idle");
  }, [selection]);

  useEffect(() => {
    if (copyState === "idle") {
      return;
    }

    const timeoutId = window.setTimeout(() => {
      setCopyState("idle");
    }, 1800);

    return () => {
      window.clearTimeout(timeoutId);
    };
  }, [copyState]);

  useEffect(() => {
    if (openState === "idle") {
      return;
    }

    const timeoutId = window.setTimeout(() => {
      setOpenState("idle");
    }, 1800);

    return () => {
      window.clearTimeout(timeoutId);
    };
  }, [openState]);

  const handleRatingSelect = useCallback(async (nextRating: number): Promise<void> => {
    if (!selectedRecordId || !record || savingRating) {
      return;
    }

    setSavingRating(true);
    setRatingError(null);

    try {
      const updated = await saveRecordRating(selectedRecordId, nextRating);
      dispatch({
        type: "UPDATE_RECORD_RATING",
        payload: {
          recordId: updated.record_id,
          rating: updated.rating,
          updatedAt: updated.updated_at,
        },
      });
    } catch (error) {
      setRatingError(error instanceof Error ? error.message : "Failed to save rating.");
    } finally {
      setSavingRating(false);
      setHoveredRating(null);
    }
  }, [dispatch, record, savingRating, selectedRecordId]);

  async function handleCopyRecordPath(): Promise<void> {
    if (!recordPath) {
      setCopyState("error");
      return;
    }

    try {
      await copyTextToClipboard(recordPath);
      setCopyState("copied");
    } catch {
      setCopyState("error");
    }
  }

  async function handleOpenRecordFolder(): Promise<void> {
    if (isStaging && stagingEntry) {
      try {
        await openStagingFolder(stagingEntry.run_id, stagingEntry.record_id);
        setOpenState("opened");
      } catch {
        setOpenState("error");
      }
      return;
    }

    if (!selectedRecordId) {
      setOpenState("error");
      return;
    }

    try {
      await openRecordFolder(selectedRecordId);
      setOpenState("opened");
    } catch {
      setOpenState("error");
    }
  }

  useEffect(() => {
    if (!hasSelectedRecord) {
      return;
    }

    const keyToRating = new Map<string, number>([
      ["Digit1", 1],
      ["Digit2", 2],
      ["Digit3", 3],
      ["Digit4", 4],
      ["Digit5", 5],
      ["Numpad1", 1],
      ["Numpad2", 2],
      ["Numpad3", 3],
      ["Numpad4", 4],
      ["Numpad5", 5],
    ]);

    const handleKeyDown = (event: KeyboardEvent) => {
      const nextRating = keyToRating.get(event.code);
      if (
        !nextRating ||
        event.repeat ||
        event.altKey ||
        event.ctrlKey ||
        event.metaKey ||
        event.shiftKey ||
        isEditableTarget(event.target)
      ) {
        return;
      }
      event.preventDefault();
      void handleRatingSelect(nextRating);
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [handleRatingSelect, hasSelectedRecord]);

  if (!selection) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">Select a record</p>
      </div>
    );
  }

  // ── Staging inspect view ──
  if (isStaging && stagingEntry) {
    return (
      <ScrollArea className="h-full min-w-0">
        <div className="min-w-0 space-y-5 pb-3">
          {/* Prompt */}
          <section>
            <SectionLabel>Prompt</SectionLabel>
            {promptStatus === "idle" || promptStatus === "loading" ? (
              <div className="space-y-1.5">
                <Skeleton className="h-3 w-full" />
                <Skeleton className="h-3 w-[92%]" />
                <Skeleton className="h-3 w-[85%]" />
              </div>
            ) : promptText ? (
              <p className="whitespace-pre-wrap break-words text-[12px] leading-[1.6] text-[var(--text-secondary)] [overflow-wrap:anywhere]">
                {promptText}
              </p>
            ) : (
              <p className="text-[11px] text-[var(--text-quaternary)]">Prompt unavailable</p>
            )}
          </section>

          {/* Staging info */}
          <section>
            <SectionLabel>Info</SectionLabel>
            <div className="space-y-3">
              <div className="flex flex-wrap items-center gap-1.5">
                <Badge variant={stagingEntry.status === "failed" ? "destructive" : "success"}>
                  {stagingEntry.status ?? "unknown"}
                </Badge>
              </div>
              <div className="space-y-0">
                <div className="prop-row">
                  <span className="prop-label">Run ID</span>
                  <span className="prop-value font-mono text-[10px]">{stagingEntry.run_id}</span>
                </div>
                <div className="prop-row">
                  <span className="prop-label">Record ID</span>
                  <span className="prop-value font-mono text-[10px]">{stagingEntry.record_id}</span>
                </div>
                <div className="prop-row">
                  <span className="prop-label">Provider</span>
                  <span className="prop-value">{stagingEntry.provider || "--"}</span>
                </div>
                <div className="prop-row">
                  <span className="prop-label">Model</span>
                  <span className="prop-value font-mono text-[10px]">{stagingEntry.model_id || "--"}</span>
                </div>
                <div className="prop-row">
                  <span className="prop-label">Thinking</span>
                  <span className="prop-value">{stagingEntry.thinking_level || "--"}</span>
                </div>
                <div className="prop-row">
                  <span className="prop-label">SDK</span>
                  <span className="prop-value font-mono text-[10px]">{stagingEntry.sdk_package || "--"}</span>
                </div>
                <div className="prop-row">
                  <span className="prop-label">Turns</span>
                  <span className="prop-value font-mono">{stagingEntry.turn_count != null ? String(stagingEntry.turn_count) : "--"}</span>
                </div>
              </div>
              <button
                type="button"
                onClick={() => void handleOpenRecordFolder()}
                className="flex w-full items-center gap-2 rounded-lg border border-[var(--border-subtle)] bg-[var(--surface-1)] px-3 py-2 text-left text-[11px] font-medium text-[var(--text-secondary)] transition-colors hover:bg-[var(--surface-0)] hover:text-[var(--text-primary)]"
              >
                <FolderOpen className={`size-3.5 ${openState === "opened" ? "text-[var(--success)]" : ""}`} />
                <span>
                  {openState === "opened"
                    ? "Opened staging folder"
                    : openState === "error"
                      ? "Open failed"
                      : "Open staging folder"}
                </span>
              </button>
            </div>
          </section>

          {/* Kinematic Tree */}
          <section>
            <div className="flex items-center justify-between pb-2">
              <div className="flex items-center gap-2">
                <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Kinematic Tree</span>
                <div className="h-px flex-1 bg-[var(--border-subtle)]" />
              </div>
              <div className="flex items-center gap-2">
                <span className="font-mono text-[9.5px] tabular-nums text-[var(--text-quaternary)]">
                  {joints.length} joint{joints.length === 1 ? "" : "s"} · {movableJointCount} movable
                </span>
                <Button
                  variant="ghost"
                  size="sm"
                  onClick={onResetAll}
                  className="h-5 gap-1 px-1.5 text-[10px] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
                >
                  <RotateCcw className="size-2.5" />
                  Reset
                </Button>
              </div>
            </div>

            {joints.length === 0 ? (
              <div className="flex h-20 items-center justify-center">
                <p className="text-[11px] text-[var(--text-quaternary)]">
                  {stagingEntry.has_checkpoint_urdf ? "No joints detected" : "Awaiting first compile"}
                </p>
              </div>
            ) : (
              <div className="space-y-1">
                {rootLinks.map((rootLink) => (
                  <LinkBranch
                    key={rootLink}
                    linkName={rootLink}
                    jointsByParent={jointsByParent}
                    jointValues={jointValues}
                    onJointChange={onJointChange}
                    seen={new Set([rootLink])}
                  />
                ))}
              </div>
            )}
          </section>
        </div>
      </ScrollArea>
    );
  }

  // ── Record inspect view (existing) ──
  if (!selectedRecordId) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">Select a record</p>
      </div>
    );
  }

  if (!record) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">Record not found</p>
      </div>
    );
  }

  return (
    <ScrollArea className="h-full min-w-0">
      <div className="min-w-0 space-y-5 pb-3">
        {/* Prompt */}
        <section>
          <SectionLabel>Prompt</SectionLabel>
          {promptStatus === "idle" || promptStatus === "loading" ? (
            <div className="space-y-1.5">
              <Skeleton className="h-3 w-full" />
              <Skeleton className="h-3 w-[92%]" />
              <Skeleton className="h-3 w-[85%]" />
            </div>
          ) : promptText ? (
            <p className="whitespace-pre-wrap break-words text-[12px] leading-[1.6] text-[var(--text-secondary)] [overflow-wrap:anywhere]">
              {promptText}
            </p>
          ) : (
            <p className="text-[11px] text-[var(--text-quaternary)]">Prompt unavailable</p>
          )}
        </section>

        {/* Rating */}
        <section>
          <SectionLabel>Rating</SectionLabel>
          <div className="flex items-center justify-between">
            <div
              className="flex items-center gap-0.5"
              onMouseLeave={() => setHoveredRating(null)}
            >
              {[1, 2, 3, 4, 5].map((starValue) => {
                const activeRating = hoveredRating ?? record.rating ?? 0;
                const isActive = starValue <= activeRating;

                return (
                  <button
                    key={starValue}
                    type="button"
                    aria-label={`Rate ${starValue} star${starValue === 1 ? "" : "s"}`}
                    aria-pressed={record.rating === starValue}
                    disabled={savingRating}
                    className="rounded-md p-1 text-[var(--border-strong)] transition-colors duration-100 hover:text-[#e0a100] focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)] disabled:cursor-not-allowed"
                    onMouseEnter={() => setHoveredRating(starValue)}
                    onClick={() => void handleRatingSelect(starValue)}
                  >
                    <Star
                      className={`size-3.5 ${isActive ? "fill-[#e0a100] text-[#e0a100]" : "fill-transparent text-current"}`}
                    />
                  </button>
                );
              })}
            </div>
            <span className="text-[10px] text-[var(--text-tertiary)]">
              {savingRating
                ? "Saving…"
                : record.rating
                  ? `${record.rating} / 5`
                  : "Unrated"}
            </span>
          </div>
          {ratingError ? <p className="mt-1.5 text-[10px] text-[var(--destructive)]">{ratingError}</p> : null}
        </section>

        {/* Record context */}
        <section>
          <SectionLabel>Info</SectionLabel>
          <div className="space-y-3">
            <div className="rounded-lg border border-[var(--border-subtle)] bg-[var(--surface-1)] px-3 py-2.5">
              <div className="flex items-center justify-between gap-2">
                <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">
                  Path
                </span>
                <div className="flex shrink-0 items-center gap-0.5">
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <button
                        type="button"
                        aria-label={copyState === "copied" ? "Copied object path" : "Copy object path"}
                        className="flex size-5 cursor-pointer items-center justify-center rounded-md text-[var(--text-quaternary)] transition-colors duration-100 hover:bg-[var(--surface-0)] hover:text-[var(--text-secondary)] focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]"
                        onClick={() => void handleCopyRecordPath()}
                      >
                        <Copy className={`size-3 ${copyState === "copied" ? "text-[var(--success)]" : ""}`} />
                      </button>
                    </TooltipTrigger>
                    <TooltipContent side="top">
                      {copyState === "copied" ? "Copied!" : copyState === "error" ? "Failed" : "Copy path"}
                    </TooltipContent>
                  </Tooltip>
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <button
                        type="button"
                        aria-label={openState === "opened" ? "Opened object folder" : "Open object folder"}
                        className="flex size-5 cursor-pointer items-center justify-center rounded-md text-[var(--text-quaternary)] transition-colors duration-100 hover:bg-[var(--surface-0)] hover:text-[var(--text-secondary)] focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]"
                        onClick={() => void handleOpenRecordFolder()}
                      >
                        <FolderOpen className={`size-3 ${openState === "opened" ? "text-[var(--success)]" : ""}`} />
                      </button>
                    </TooltipTrigger>
                    <TooltipContent side="top">
                      {openState === "opened" ? "Opened!" : openState === "error" ? "Failed" : "Open folder"}
                    </TooltipContent>
                  </Tooltip>
                </div>
              </div>
              <p
                className="mt-2 break-all font-mono text-[10px] leading-[1.5] text-[var(--text-secondary)]"
                title={recordPath ?? undefined}
              >
                {recordPath ?? "--"}
              </p>
            </div>
            <div className="space-y-0">
              <div className="prop-row">
                <span className="prop-label">SDK</span>
                <span className="prop-value font-mono text-[10px]">{record.sdk_package || "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Provider</span>
                <span className="prop-value">{record.provider || "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Model</span>
                <span className="prop-value font-mono text-[10px]">{record.model_id || "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Thinking</span>
                <span className="prop-value">{record.thinking_level || "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Category</span>
                <span className="prop-value">{record.category_slug || "--"}</span>
              </div>
              <div className="prop-row">
                <span className="prop-label">Updated</span>
                <span className="prop-value font-mono text-[10px]">
                  {record.updated_at
                    ? new Date(record.updated_at).toLocaleString(undefined, {
                        year: "numeric",
                        month: "short",
                        day: "numeric",
                        hour: "2-digit",
                        minute: "2-digit",
                        second: "2-digit",
                      })
                    : "--"}
                </span>
              </div>
            </div>
          </div>
        </section>

        {/* Kinematic Tree */}
        <section>
          <div className="flex items-center justify-between pb-2">
            <div className="flex items-center gap-2">
              <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Kinematic Tree</span>
              <div className="h-px flex-1 bg-[var(--border-subtle)]" />
            </div>
            <div className="flex items-center gap-2">
              <span className="font-mono text-[9.5px] tabular-nums text-[var(--text-quaternary)]">
                {joints.length} joint{joints.length === 1 ? "" : "s"} · {movableJointCount} movable
              </span>
              <Button
                variant="ghost"
                size="sm"
                onClick={onResetAll}
                className="h-5 gap-1 px-1.5 text-[10px] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
              >
                <RotateCcw className="size-2.5" />
                Reset
              </Button>
            </div>
          </div>

          {joints.length === 0 ? (
            <div className="flex h-20 items-center justify-center">
              <p className="text-[11px] text-[var(--text-quaternary)]">No joints detected</p>
            </div>
          ) : (
            <div className="space-y-1">
              {rootLinks.map((rootLink) => (
                <LinkBranch
                  key={rootLink}
                  linkName={rootLink}
                  jointsByParent={jointsByParent}
                  jointValues={jointValues}
                  onJointChange={onJointChange}
                  seen={new Set([rootLink])}
                />
              ))}
            </div>
          )}
        </section>
      </div>
    </ScrollArea>
  );
}
