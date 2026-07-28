import { useCallback, useEffect, useMemo, useState, type JSX } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  ArrowLeft,
  ArrowRight,
  Check,
  ExternalLink,
  RefreshCw,
  X,
} from "lucide-react";
import type { Object3D } from "three";

import { SceneCanvas } from "@/components/viewer3d/SceneCanvas";
import { useJointController } from "@/components/viewer3d/useJointController";
import { defaultRenderOptions } from "@/components/viewer3d/useRenderOptions";
import type { UrdfJoint, UrdfSpec } from "@/components/viewer3d/urdf-parser";
import { formatCost } from "@/lib/viewer-format";
import { bootstrapQueryOptions } from "@/lib/viewer-queries";
import type { RecordSummary } from "@/lib/types";

const CONDITION_ORDER = [
  "full_baseline",
  "primitives_only",
  "no_compile_feedback",
  "no_example_retrieval",
  "single_pass",
] as const;

type ConditionId = (typeof CONDITION_ORDER)[number];

type ConditionDefinition = {
  id: ConditionId;
  label: string;
  accent: string;
};

type ObjectGroup = {
  promptId: string;
  label: string;
  records: Partial<Record<ConditionId, RecordSummary>>;
};

const CONDITIONS: ConditionDefinition[] = [
  {
    id: "full_baseline",
    label: "Full baseline",
    accent: "#d8ff5b",
  },
  {
    id: "primitives_only",
    label: "Primitives only",
    accent: "#62e6b5",
  },
  {
    id: "no_compile_feedback",
    label: "No compile feedback",
    accent: "#ff8f70",
  },
  {
    id: "no_example_retrieval",
    label: "No example retrieval",
    accent: "#68b8ff",
  },
  {
    id: "single_pass",
    label: "Single pass",
    accent: "#c7a3ff",
  },
];

const PROMPT_ORDER = [
  "compact_excavator",
  "powered_hospital_bed",
  "folding_bicycle",
  "sliding_compound_miter_saw",
  "dishwasher",
  "communications_satellite",
  "benchtop_cnc_mill",
  "wall_bed",
  "self_propelled_crop_sprayer",
  "video_tripod",
];

const EMPTY_JOINT_POSE = new Map<string, number>();
const ARTICULATION_MIN_CYCLE_SECONDS = 3.2;
const ARTICULATION_LINEAR_SPEED_MPS = 0.08;
const ARTICULATION_ANGULAR_SPEED_RAD_PER_SECOND = Math.PI / 5;
const COMPARISON_RENDER_OPTIONS = {
  ...defaultRenderOptions,
  showEdges: true,
  showGrid: true,
  doubleSided: true,
  fancyGraphics: false,
  autoAnimate: true,
};

type JointMotion = {
  joint: UrdfJoint;
  cycleSeconds: number;
  phaseOffset: number;
};

function isArticulatedJoint(joint: UrdfJoint): boolean {
  return (
    !joint.mimic &&
    (joint.type === "revolute" ||
      joint.type === "continuous" ||
      joint.type === "prismatic")
  );
}

function jointTravelSpan(joint: UrdfJoint): number {
  if (joint.type === "continuous") {
    return Math.PI * 2;
  }

  const lower = joint.limit?.lower;
  const upper = joint.limit?.upper;
  if (
    typeof lower === "number" &&
    Number.isFinite(lower) &&
    typeof upper === "number" &&
    Number.isFinite(upper) &&
    upper > lower
  ) {
    return upper - lower;
  }
  return joint.type === "prismatic" ? 0.24 : (Math.PI * 2) / 3;
}

function jointPhaseOffset(jointName: string, index: number): number {
  let hash = 0;
  for (const character of jointName) {
    hash = (hash * 33 + character.charCodeAt(0)) % 4096;
  }
  return ((hash / 4096) + index * 0.61803398875) % 1;
}

function buildJointMotions(spec: UrdfSpec): JointMotion[] {
  return spec.joints.filter(isArticulatedJoint).map((joint, index) => {
    const span = jointTravelSpan(joint);
    const speed =
      joint.type === "prismatic"
        ? ARTICULATION_LINEAR_SPEED_MPS
        : ARTICULATION_ANGULAR_SPEED_RAD_PER_SECOND;
    return {
      joint,
      cycleSeconds: Math.max(ARTICULATION_MIN_CYCLE_SECONDS, (span * 2) / speed),
      phaseOffset: jointPhaseOffset(joint.name, index),
    };
  });
}

function articulatedJointValue(joint: UrdfJoint, phase: number): number {
  if (joint.type === "continuous") {
    return ((phase * Math.PI * 2 + Math.PI) % (Math.PI * 2)) - Math.PI;
  }

  const lower = joint.limit?.lower;
  const upper = joint.limit?.upper;
  if (
    typeof lower === "number" &&
    Number.isFinite(lower) &&
    typeof upper === "number" &&
    Number.isFinite(upper) &&
    upper > lower
  ) {
    const normalized = 0.5 - 0.5 * Math.cos(phase * Math.PI * 2);
    return lower + (upper - lower) * normalized;
  }

  const wave = Math.sin(phase * Math.PI * 2);
  return joint.type === "prismatic" ? wave * 0.12 : wave * (Math.PI / 3);
}

function AutoArticulatingScene({
  record,
}: {
  record: RecordSummary;
}): JSX.Element {
  const [urdfSpec, setUrdfSpec] = useState<UrdfSpec | null>(null);
  const [jointNodes, setJointNodes] = useState<Map<string, Object3D> | null>(null);
  const { applyJointValues } = useJointController(jointNodes, urdfSpec);

  const handleUrdfSpecChange = useCallback(
    (spec: UrdfSpec | null, nodes: Map<string, Object3D> | null) => {
      setUrdfSpec(spec);
      setJointNodes(nodes);
    },
    [],
  );

  useEffect(() => {
    if (
      !urdfSpec ||
      window.matchMedia("(prefers-reduced-motion: reduce)").matches
    ) {
      return;
    }

    const motions = buildJointMotions(urdfSpec);
    if (motions.length === 0) {
      return;
    }

    let frameId = 0;
    const tick = (now: number): void => {
      const elapsedSeconds = now / 1000;
      const values = new Map<string, number>();
      for (const motion of motions) {
        const phase =
          ((elapsedSeconds / motion.cycleSeconds) + motion.phaseOffset) % 1;
        values.set(motion.joint.name, articulatedJointValue(motion.joint, phase));
      }
      applyJointValues(values);
      frameId = requestAnimationFrame(tick);
    };

    frameId = requestAnimationFrame(tick);
    return () => {
      cancelAnimationFrame(frameId);
      applyJointValues(EMPTY_JOINT_POSE);
    };
  }, [applyJointValues, urdfSpec]);

  return (
    <SceneCanvas
      baseFileUrl={`/api/records/${record.record_id}/files`}
      assetRevisionKey={record.viewer_asset_updated_at}
      selectionKey={record.record_id}
      jointPoseSignal={EMPTY_JOINT_POSE}
      renderOptions={COMPARISON_RENDER_OPTIONS}
      onUrdfSpecChange={handleUrdfSpecChange}
    />
  );
}

function displayName(promptId: string): string {
  const words = promptId.replaceAll("_", " ");
  return words
    .replace(/\bcnc\b/gi, "CNC")
    .replace(/\b\w/, (letter) => letter.toUpperCase());
}

function parseRecordIdentity(
  record: RecordSummary,
): { promptId: string; conditionId: ConditionId } | null {
  const prefix = "rec_ablation_";
  if (!record.record_id.startsWith(prefix)) {
    return null;
  }

  const identity = record.record_id.slice(prefix.length);
  for (const conditionId of CONDITION_ORDER) {
    const suffix = `__${conditionId}`;
    if (identity.endsWith(suffix)) {
      return {
        promptId: identity.slice(0, -suffix.length),
        conditionId,
      };
    }
  }
  return null;
}

export function groupAblationRecords(records: RecordSummary[]): ObjectGroup[] {
  const groups = new Map<string, ObjectGroup>();

  for (const record of records) {
    const identity = parseRecordIdentity(record);
    if (!identity) {
      continue;
    }
    const group = groups.get(identity.promptId) ?? {
      promptId: identity.promptId,
      label: displayName(identity.promptId),
      records: {},
    };
    group.records[identity.conditionId] = record;
    groups.set(identity.promptId, group);
  }

  return Array.from(groups.values()).sort((left, right) => {
    const leftIndex = PROMPT_ORDER.indexOf(left.promptId);
    const rightIndex = PROMPT_ORDER.indexOf(right.promptId);
    if (leftIndex >= 0 && rightIndex >= 0) {
      return leftIndex - rightIndex;
    }
    if (leftIndex >= 0) return -1;
    if (rightIndex >= 0) return 1;
    return left.label.localeCompare(right.label);
  });
}

function readPromptIdFromUrl(): string | null {
  return new URLSearchParams(window.location.search).get("object");
}

function syncPromptIdToUrl(promptId: string): void {
  const url = new URL(window.location.href);
  url.pathname = "/compare";
  url.searchParams.set("object", promptId);
  window.history.replaceState(null, "", url);
}

function StatusMark({
  success,
  label,
}: {
  success: boolean;
  label: string;
}): JSX.Element {
  return (
    <span
      className={[
        "inline-flex items-center gap-1.5 rounded-full border px-2 py-1 font-mono text-[9px] font-medium uppercase tracking-[0.08em]",
        success
          ? "border-emerald-400/25 bg-emerald-400/10 text-emerald-300"
          : "border-rose-400/25 bg-rose-400/10 text-rose-300",
      ].join(" ")}
    >
      {success ? <Check className="size-2.5" /> : <X className="size-2.5" />}
      {label}
    </span>
  );
}

function Metric({
  label,
  value,
}: {
  label: string;
  value: string;
}): JSX.Element {
  return (
    <div className="border-r border-white/[0.07] px-3 py-2.5 last:border-r-0">
      <p className="font-mono text-[8px] uppercase tracking-[0.12em] text-white/35">{label}</p>
      <p className="mt-1 truncate text-[11px] font-medium text-white/80" title={value}>
        {value}
      </p>
    </div>
  );
}

function ConditionCard({
  condition,
  record,
}: {
  condition: ConditionDefinition;
  record: RecordSummary | undefined;
}): JSX.Element {
  const modelOnly = record?.tags.includes("model-only-package") ?? false;
  const exportSucceeded =
    record?.materialization_status === "available" &&
    (modelOnly || record.tags.includes("export-success"));
  const generationSucceeded = modelOnly ? exportSucceeded : record?.run_status === "success";
  const qcSucceeded = record?.tags.includes("qc-success") ?? false;

  return (
    <article
      className="group flex min-h-0 flex-col overflow-hidden rounded-[14px] border border-white/[0.09] bg-[#151a18] shadow-[0_24px_60px_rgba(0,0,0,0.28)]"
      style={{ "--condition-accent": condition.accent } as React.CSSProperties}
    >
      <header className="relative border-b border-white/[0.08] px-3 py-4">
        <div
          className="absolute inset-x-0 top-0 h-[3px]"
          style={{ background: condition.accent }}
        />
        <h2 className="text-[15px] font-semibold tracking-[-0.02em] text-white">
          {condition.label}
        </h2>
      </header>

      <div className="relative min-h-[220px] flex-1 bg-[#edf0ec]">
        {record && exportSucceeded ? (
          <AutoArticulatingScene record={record} />
        ) : (
          <div className="absolute inset-0 flex items-center justify-center bg-[repeating-linear-gradient(135deg,#edf0ec,#edf0ec_12px,#e6e9e5_12px,#e6e9e5_24px)] p-6">
            <div className="max-w-[220px] rounded-xl border border-rose-900/15 bg-white/90 px-5 py-4 text-center shadow-sm backdrop-blur">
              <span className="mx-auto flex size-8 items-center justify-center rounded-full bg-rose-50 text-rose-600">
                <X className="size-4" />
              </span>
              <p className="mt-3 text-[12px] font-semibold text-[#351b1b]">
                {!record
                  ? "Missing record"
                  : modelOnly
                    ? "Model unavailable"
                  : generationSucceeded
                    ? "Compile failed"
                    : "Request failed"}
              </p>
            </div>
          </div>
        )}
        <div className="pointer-events-none absolute inset-x-0 bottom-0 h-12 bg-gradient-to-t from-black/10 to-transparent" />
      </div>

      {modelOnly ? null : (
        <footer className="border-t border-white/[0.08] bg-[#101412]">
          <div className="flex min-h-[42px] flex-wrap items-center gap-1.5 border-b border-white/[0.07] px-3 py-2">
            <StatusMark success={generationSucceeded} label="Gen" />
            <StatusMark success={exportSucceeded} label="URDF" />
            <StatusMark success={qcSucceeded} label="QC" />
          </div>
          <div className="grid grid-cols-3">
            <Metric label="Turns" value={record?.turn_count == null ? "None" : String(record.turn_count)} />
            <Metric label="Cost" value={formatCost(record?.total_cost_usd ?? null)} />
            <Metric label="SDK" value={record?.sdk_package ?? "Unknown"} />
          </div>
          {record ? (
            <a
              href={`/viewer?record=${encodeURIComponent(record.record_id)}`}
              className="flex h-9 items-center justify-between border-t border-white/[0.07] px-3 font-mono text-[9px] uppercase tracking-[0.1em] text-white/45 transition hover:bg-white/[0.04] hover:text-white/80 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-inset focus-visible:ring-[var(--condition-accent)]"
            >
              Inspect this record
              <ExternalLink className="size-3" />
            </a>
          ) : null}
        </footer>
      )}
    </article>
  );
}

export default function AblationComparePage(): JSX.Element {
  const bootstrapQuery = useQuery(bootstrapQueryOptions());
  const groups = useMemo(
    () => groupAblationRecords(bootstrapQuery.data?.library_records ?? []),
    [bootstrapQuery.data?.library_records],
  );
  const [selectedPromptId, setSelectedPromptId] = useState<string | null>(readPromptIdFromUrl);

  useEffect(() => {
    if (groups.length === 0) {
      return;
    }
    const validSelection = groups.some((group) => group.promptId === selectedPromptId);
    const nextPromptId = validSelection ? selectedPromptId : groups[0].promptId;
    if (!nextPromptId) {
      return;
    }
    if (nextPromptId !== selectedPromptId) {
      setSelectedPromptId(nextPromptId);
    }
    syncPromptIdToUrl(nextPromptId);
  }, [groups, selectedPromptId]);

  const selectedIndex = Math.max(
    0,
    groups.findIndex((group) => group.promptId === selectedPromptId),
  );
  const selectedGroup = groups[selectedIndex] ?? null;
  const modelOnlyPackage =
    groups.length > 0 &&
    groups.every((group) =>
      Object.values(group.records).every((record) =>
        record?.tags.includes("model-only-package"),
      ),
    );

  const selectGroup = (index: number): void => {
    const group = groups[index];
    if (!group) return;
    setSelectedPromptId(group.promptId);
    syncPromptIdToUrl(group.promptId);
  };

  if (bootstrapQuery.isLoading) {
    return (
      <main className="flex h-screen items-center justify-center bg-[#0c100e] text-white">
        <p className="font-mono text-[11px] uppercase tracking-[0.14em] text-white/45">
          Loading comparison
        </p>
      </main>
    );
  }

  if (bootstrapQuery.error || groups.length === 0) {
    return (
      <main className="flex h-screen items-center justify-center bg-[#0c100e] p-6 text-white">
        <div className="max-w-md rounded-2xl border border-white/10 bg-white/[0.04] p-6 text-center">
          <h1 className="text-xl font-semibold">Comparison data is unavailable</h1>
          <p className="mt-2 text-[12px] leading-5 text-white/50">
            This page expects records from the agent component ablation experiment.
          </p>
          <button
            type="button"
            onClick={() => void bootstrapQuery.refetch()}
            className="mt-5 inline-flex h-9 items-center gap-2 rounded-lg bg-[#d8ff5b] px-4 text-[11px] font-semibold text-[#11150f]"
          >
            <RefreshCw className="size-3.5" />
            Try again
          </button>
        </div>
      </main>
    );
  }

  return (
    <main className="relative flex h-screen min-h-0 flex-col overflow-hidden bg-[#0c100e] text-white">
      <div
        className="pointer-events-none absolute inset-0 opacity-40"
        style={{
          backgroundImage:
            "linear-gradient(rgba(255,255,255,0.025) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,0.025) 1px, transparent 1px)",
          backgroundSize: "32px 32px",
        }}
      />

      <header className="relative z-10 border-b border-white/[0.08] bg-[#0c100e]/95 px-5 py-4 backdrop-blur">
        <div className="flex flex-wrap items-end justify-between gap-4">
          <div>
            <h1 className="text-[clamp(20px,2vw,30px)] font-semibold tracking-[-0.035em]">
              Agent component ablations
            </h1>
          </div>

          {modelOnlyPackage ? null : (
            <div className="flex items-center gap-2">
              <span className="rounded-md border border-white/10 bg-white/[0.04] px-2.5 py-1.5 font-mono text-[9px] uppercase tracking-[0.1em] text-white/45">
                {groups.length} objects
              </span>
              <span className="rounded-md border border-white/10 bg-white/[0.04] px-2.5 py-1.5 font-mono text-[9px] uppercase tracking-[0.1em] text-white/45">
                {bootstrapQuery.data?.library_records.length ?? 0} cells
              </span>
              <a
                href="/"
                className="rounded-md border border-white/10 bg-white/[0.04] px-2.5 py-1.5 font-mono text-[9px] uppercase tracking-[0.1em] text-white/55 transition hover:bg-white/[0.08] hover:text-white"
              >
                Library viewer
              </a>
            </div>
          )}
        </div>

        <div className="mt-4 flex items-center gap-2">
          <button
            type="button"
            onClick={() => selectGroup(Math.max(0, selectedIndex - 1))}
            disabled={selectedIndex === 0}
            className="flex size-9 items-center justify-center rounded-lg border border-white/10 bg-white/[0.04] text-white/60 transition hover:bg-white/[0.08] hover:text-white disabled:opacity-25"
            aria-label="Previous object"
          >
            <ArrowLeft className="size-4" />
          </button>
          <label className="min-w-0 flex-1">
            <span className="sr-only">Object to compare</span>
            <select
              value={selectedGroup?.promptId ?? ""}
              onChange={(event) => {
                const nextIndex = groups.findIndex(
                  (group) => group.promptId === event.target.value,
                );
                selectGroup(nextIndex);
              }}
              className="h-9 w-full appearance-none rounded-lg border border-white/10 bg-[#151a18] px-3 text-[12px] font-medium text-white outline-none transition focus:border-[#d8ff5b]/60 focus:ring-2 focus:ring-[#d8ff5b]/15"
            >
              {groups.map((group, index) => (
                <option key={group.promptId} value={group.promptId}>
                  {String(index + 1).padStart(2, "0")}  {group.label}
                </option>
              ))}
            </select>
          </label>
          <button
            type="button"
            onClick={() => selectGroup(Math.min(groups.length - 1, selectedIndex + 1))}
            disabled={selectedIndex === groups.length - 1}
            className="flex size-9 items-center justify-center rounded-lg border border-white/10 bg-white/[0.04] text-white/60 transition hover:bg-white/[0.08] hover:text-white disabled:opacity-25"
            aria-label="Next object"
          >
            <ArrowRight className="size-4" />
          </button>
          <div className="hidden items-center gap-1.5 pl-2 xl:flex">
            {groups.map((group, index) => (
              <button
                key={group.promptId}
                type="button"
                onClick={() => selectGroup(index)}
                className={[
                  "h-2 rounded-full transition-all",
                  group.promptId === selectedGroup?.promptId
                    ? "w-7 bg-[#d8ff5b]"
                    : "w-2 bg-white/20 hover:bg-white/40",
                ].join(" ")}
                aria-label={`Compare ${group.label}`}
                aria-pressed={group.promptId === selectedGroup?.promptId}
              />
            ))}
          </div>
        </div>
      </header>

      <section
        className="custom-scrollbar relative z-10 min-h-0 flex-1 overflow-auto p-4"
        aria-label={selectedGroup ? `Comparison for ${selectedGroup.label}` : "Comparison"}
      >
        <div className="grid h-full min-h-[590px] min-w-[900px] grid-cols-3 grid-rows-2 gap-3">
          {CONDITIONS.map((condition) => (
            <ConditionCard
              key={condition.id}
              condition={condition}
              record={selectedGroup?.records[condition.id]}
            />
          ))}
        </div>
      </section>
    </main>
  );
}
