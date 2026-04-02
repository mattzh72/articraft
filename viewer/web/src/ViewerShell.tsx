import { startTransition, useState, useCallback, useEffect, useMemo, useRef, type JSX } from "react";
import { AlertTriangle, ChevronLeft } from "lucide-react";
import type { PanelImperativeHandle, PanelSize } from "react-resizable-panels";
import * as THREE from "three";

import { useViewer } from "@/lib/viewer-context";
import { updateUrlSearchParams } from "@/lib/url";
import {
  ResizablePanelGroup,
  ResizablePanel,
  ResizableHandle,
} from "@/components/ui/resizable";
import { Sidebar } from "@/components/layout/Sidebar";
import { Inspector } from "@/components/layout/Inspector";
import { ViewportPanel } from "@/components/layout/ViewportPanel";
import { InspectorTabs } from "@/components/inspector/InspectorTabs";
import type { RenderOptions as InspectorRenderOptions } from "@/components/inspector/RenderOptionsPanel";
import { RecordBrowser } from "@/components/browser/RecordBrowser";
import { Button } from "@/components/ui/button";
import { useRenderOptions } from "@/components/viewer3d/useRenderOptions";
import { useJointController } from "@/components/viewer3d/useJointController";
import type { UrdfJoint, UrdfSpec } from "@/components/viewer3d/urdf-parser";
import { MissingArtifactsOverlay } from "@/components/layout/MissingArtifactsOverlay";

const JOINT_POSE_QUERY_PARAM = "pose";
const INSPECTOR_QUERY_PARAM = "inspector";
const JOINT_POSE_PRECISION = 1e5;
const PREVIEW_UI_SYNC_MS = 100;
const PREVIEW_MIN_CYCLE_SECONDS = 2.6;
const PREVIEW_LINEAR_SPEED_MPS = 0.08;
const PREVIEW_ANGULAR_SPEED_RAD_PER_SECOND = Math.PI / 4;
const PREVIEW_CONTINUOUS_ANGULAR_SPEED_RAD_PER_SECOND = Math.PI / 5;
const PREVIEW_FALLBACK_PRISMATIC_TRAVEL_METERS = 0.24;
const PREVIEW_FALLBACK_REVOLUTE_TRAVEL_RADIANS = (Math.PI * 2) / 3;

type JointPoseSnapshot = {
  recordId: string;
  values: Record<string, number>;
};

type CollisionSupportState = {
  available: boolean;
  summary: string;
  detail: string;
  compileCommand: string | null;
};

type PreviewJointMotion = {
  joint: UrdfJoint;
  cycleSeconds: number;
  phaseOffset: number;
};

function readInspectorCollapsedFromUrl(): boolean {
  if (typeof window === "undefined") {
    return false;
  }

  try {
    return new URLSearchParams(window.location.search).get(INSPECTOR_QUERY_PARAM) === "closed";
  } catch {
    return false;
  }
}

function syncInspectorCollapsedToUrl(collapsed: boolean): void {
  updateUrlSearchParams((params) => {
    if (collapsed) {
      params.set(INSPECTOR_QUERY_PARAM, "closed");
    } else {
      params.delete(INSPECTOR_QUERY_PARAM);
    }
  });
}

function roundJointValue(value: number): number {
  return Math.round(value * JOINT_POSE_PRECISION) / JOINT_POSE_PRECISION;
}

function readJointPoseFromUrl(
  recordId: string | null,
  validJointNames: Set<string>,
): Map<string, number> {
  if (typeof window === "undefined" || !recordId) {
    return new Map();
  }

  try {
    const raw = new URLSearchParams(window.location.search).get(JOINT_POSE_QUERY_PARAM);
    if (!raw) {
      return new Map();
    }

    const parsed = JSON.parse(raw) as Partial<JointPoseSnapshot>;
    if (parsed.recordId !== recordId || !parsed.values || typeof parsed.values !== "object") {
      return new Map();
    }

    const nextValues = new Map<string, number>();
    for (const [jointName, value] of Object.entries(parsed.values)) {
      if (!validJointNames.has(jointName) || typeof value !== "number" || !Number.isFinite(value)) {
        continue;
      }
      if (Math.abs(value) <= 1 / JOINT_POSE_PRECISION) {
        continue;
      }
      nextValues.set(jointName, roundJointValue(value));
    }

    return nextValues;
  } catch {
    return new Map();
  }
}

function syncJointPoseToUrl(recordId: string | null, jointValues: Map<string, number>): void {
  updateUrlSearchParams((params) => {
    if (!recordId) {
      params.delete(JOINT_POSE_QUERY_PARAM);
      return;
    }

    const values = Object.fromEntries(
      Array.from(jointValues.entries())
        .map(([jointName, value]) => [jointName, roundJointValue(value)] as const)
        .filter(([, value]) => Number.isFinite(value) && Math.abs(value) > 1 / JOINT_POSE_PRECISION),
    );

    if (Object.keys(values).length === 0) {
      params.delete(JOINT_POSE_QUERY_PARAM);
      return;
    }

    params.set(
      JOINT_POSE_QUERY_PARAM,
      JSON.stringify({
        recordId,
        values,
      } satisfies JointPoseSnapshot),
    );
  });
}

function isPreviewJoint(joint: UrdfJoint): boolean {
  return joint.type === "revolute" || joint.type === "continuous" || joint.type === "prismatic";
}

function buildRootLinks(joints: UrdfJoint[]): string[] {
  const parents = new Set<string>();
  const children = new Set<string>();

  for (const joint of joints) {
    parents.add(joint.parent);
    children.add(joint.child);
  }

  const roots = Array.from(parents).filter((name) => !children.has(name));
  return roots.length > 0 ? roots : Array.from(new Set(joints.map((joint) => joint.parent)));
}

function buildPreviewJointSequence(urdfSpec: UrdfSpec): UrdfJoint[] {
  const jointsByParent = new Map<string, UrdfJoint[]>();

  for (const joint of urdfSpec.joints) {
    const current = jointsByParent.get(joint.parent) ?? [];
    current.push(joint);
    jointsByParent.set(joint.parent, current);
  }

  const orderedJoints: UrdfJoint[] = [];
  const seenLinks = new Set<string>();

  const visitLink = (linkName: string): void => {
    if (seenLinks.has(linkName)) {
      return;
    }
    seenLinks.add(linkName);

    for (const joint of jointsByParent.get(linkName) ?? []) {
      if (isPreviewJoint(joint)) {
        orderedJoints.push(joint);
      }
      visitLink(joint.child);
    }
  };

  for (const rootLink of buildRootLinks(urdfSpec.joints)) {
    visitLink(rootLink);
  }

  const seenJointNames = new Set(orderedJoints.map((joint) => joint.name));
  for (const joint of urdfSpec.joints) {
    if (isPreviewJoint(joint) && !seenJointNames.has(joint.name)) {
      orderedJoints.push(joint);
    }
  }

  return orderedJoints;
}

function previewJointTravelSpan(joint: UrdfJoint): number {
  if (joint.type === "continuous") {
    return Math.PI * 2;
  }

  const lower = joint.limit?.lower;
  const upper = joint.limit?.upper;
  const hasRange =
    typeof lower === "number"
    && Number.isFinite(lower)
    && typeof upper === "number"
    && Number.isFinite(upper)
    && upper > lower;

  if (hasRange) {
    return upper - lower;
  }

  if (joint.type === "prismatic") {
    return PREVIEW_FALLBACK_PRISMATIC_TRAVEL_METERS;
  }

  return PREVIEW_FALLBACK_REVOLUTE_TRAVEL_RADIANS;
}

function previewJointCycleSeconds(joint: UrdfJoint): number {
  const span = previewJointTravelSpan(joint);

  if (joint.type === "continuous") {
    return span / PREVIEW_CONTINUOUS_ANGULAR_SPEED_RAD_PER_SECOND;
  }

  const speed =
    joint.type === "prismatic" ? PREVIEW_LINEAR_SPEED_MPS : PREVIEW_ANGULAR_SPEED_RAD_PER_SECOND;
  return Math.max(PREVIEW_MIN_CYCLE_SECONDS, (span * 2) / speed);
}

function previewJointPhaseOffset(jointName: string, index: number): number {
  let hash = 0;
  for (const character of jointName) {
    hash = (hash * 33 + character.charCodeAt(0)) % 4096;
  }

  return THREE.MathUtils.euclideanModulo((hash / 4096) + (index * 0.61803398875), 1);
}

function buildPreviewJointMotions(urdfSpec: UrdfSpec): PreviewJointMotion[] {
  return buildPreviewJointSequence(urdfSpec).map((joint, index) => ({
    joint,
    cycleSeconds: previewJointCycleSeconds(joint),
    phaseOffset: previewJointPhaseOffset(joint.name, index),
  }));
}

function previewJointValue(joint: UrdfJoint, phase: number): number {
  if (joint.type === "continuous") {
    return THREE.MathUtils.euclideanModulo((phase * Math.PI * 2) + Math.PI, Math.PI * 2) - Math.PI;
  }

  const lower = joint.limit?.lower;
  const upper = joint.limit?.upper;
  const hasRange =
    typeof lower === "number"
    && Number.isFinite(lower)
    && typeof upper === "number"
    && Number.isFinite(upper)
    && upper > lower;

  if (hasRange) {
    const normalized = 0.5 - (0.5 * Math.cos(phase * Math.PI * 2));
    return THREE.MathUtils.lerp(lower, upper, normalized);
  }

  const wave = Math.sin(phase * Math.PI * 2);

  if (joint.type === "prismatic") {
    return wave * 0.12;
  }

  return wave * (Math.PI / 3);
}

export default function ViewerShell(): JSX.Element {
  const { bootstrap, selectedRecordId, selectedRecordSummary, selection } = useViewer();
  const { options: renderOptions, setOption: setRenderOption } = useRenderOptions();

  const [urdfSpec, setUrdfSpec] = useState<UrdfSpec | null>(null);
  const [jointNodes, setJointNodes] = useState<Map<string, THREE.Object3D> | null>(null);
  const [previewJointValues, setPreviewJointValues] = useState<Map<string, number>>(new Map());
  const [inspectorCollapsed, setInspectorCollapsed] = useState(readInspectorCollapsedFromUrl);
  const [modelLoadState, setModelLoadState] = useState<{
    loading: boolean;
    error: string | null;
    missingArtifacts: boolean;
  }>({
    loading: false,
    error: null,
    missingArtifacts: false,
  });
  const inspectorPanelRef = useRef<PanelImperativeHandle | null>(null);
  const initialInspectorCollapsedRef = useRef(inspectorCollapsed);
  const jointValuesRef = useRef<Map<string, number>>(new Map());
  const previewJointValuesRef = useRef<Map<string, number>>(new Map());
  const previewUiLastSyncRef = useRef(0);
  const viewportInvalidateRef = useRef<(() => void) | null>(null);

  const { jointValues, setJointValue, applyJointValues, resetAll } = useJointController(jointNodes, urdfSpec);

  const handleUrdfSpecChange = useCallback(
    (spec: UrdfSpec | null, nodes: Map<string, THREE.Object3D> | null) => {
      setUrdfSpec(spec);
      setJointNodes(nodes);
    },
    [],
  );

  const handleRenderOptionChange = useCallback(
    <K extends keyof InspectorRenderOptions>(key: K, value: InspectorRenderOptions[K]) => {
      setRenderOption(key, value);
    },
    [setRenderOption],
  );

  const handleInspectorResize = useCallback((size: PanelSize) => {
    setInspectorCollapsed(size.inPixels <= 24);
  }, []);

  const handleInspectorExpand = useCallback(() => {
    setInspectorCollapsed(false);
    inspectorPanelRef.current?.expand();
  }, []);

  const handleViewportInvalidateReady = useCallback((invalidate: (() => void) | null) => {
    viewportInvalidateRef.current = invalidate;
  }, []);

  useEffect(() => {
    jointValuesRef.current = jointValues;
  }, [jointValues]);

  useEffect(() => {
    if (!initialInspectorCollapsedRef.current) {
      return;
    }

    inspectorPanelRef.current?.collapse();
  }, []);

  useEffect(() => {
    syncInspectorCollapsedToUrl(inspectorCollapsed);
  }, [inspectorCollapsed]);

  useEffect(() => {
    if (!selectedRecordId || !urdfSpec) {
      return;
    }

    const validJointNames = new Set(urdfSpec.joints.map((joint) => joint.name));
    const nextPose = readJointPoseFromUrl(selectedRecordId, validJointNames);
    applyJointValues(nextPose, { commit: true });
  }, [applyJointValues, selectedRecordId, urdfSpec]);

  useEffect(() => {
    syncJointPoseToUrl(selectedRecordId, jointValues);
  }, [jointValues, selectedRecordId]);

  useEffect(() => {
    if (!urdfSpec || !renderOptions.autoAnimate) {
      previewJointValuesRef.current = new Map();
      previewUiLastSyncRef.current = 0;
      setPreviewJointValues(new Map());
      applyJointValues(jointValuesRef.current);
      return;
    }

    const previewMotions = buildPreviewJointMotions(urdfSpec);
    if (previewMotions.length === 0) {
      setPreviewJointValues(new Map());
      return;
    }

    let frameId = 0;

    const tick = (now: number) => {
      const elapsedSeconds = now / 1000;
      const nextValues = new Map<string, number>();

      for (const motion of previewMotions) {
        const jointPhase = THREE.MathUtils.euclideanModulo(
          (elapsedSeconds / motion.cycleSeconds) + motion.phaseOffset,
          1,
        );
        nextValues.set(motion.joint.name, previewJointValue(motion.joint, jointPhase));
      }

      previewJointValuesRef.current = nextValues;
      if (now - previewUiLastSyncRef.current >= PREVIEW_UI_SYNC_MS) {
        previewUiLastSyncRef.current = now;
        startTransition(() => {
          setPreviewJointValues(new Map(nextValues));
        });
      }
      applyJointValues(nextValues);
      viewportInvalidateRef.current?.();
      frameId = requestAnimationFrame(tick);
    };

    frameId = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(frameId);
      previewJointValuesRef.current = new Map();
      previewUiLastSyncRef.current = 0;
      setPreviewJointValues(new Map());
      applyJointValues(jointValuesRef.current);
      viewportInvalidateRef.current?.();
    };
  }, [applyJointValues, renderOptions.autoAnimate, urdfSpec]);

  const inspectorRenderOptions = {
    showEdges: renderOptions.showEdges,
    showGrid: renderOptions.showGrid,
    showCollisions: renderOptions.showCollisions,
    showSegmentColors: renderOptions.showSegmentColors,
    showSurfaceSamples: renderOptions.showSurfaceSamples,
    doubleSided: renderOptions.doubleSided,
    autoAnimate: renderOptions.autoAnimate,
    showJointOverlay: renderOptions.showJointOverlay,
  };

  const baseFileUrl = selection
    ? selection.kind === "record"
      ? `/api/records/${selection.recordId}/files`
      : `/api/staging/${selection.runId}/${selection.recordId}/files`
    : null;
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
  const selectedRecord = useMemo(() => {
    return selection?.kind === "record" ? selectedRecordSummary : null;
  }, [selectedRecordSummary, selection]);
  const assetRevisionKey = selection
    ? selection.kind === "staging"
      ? selectedStagingEntry?.checkpoint_updated_at ?? selectedStagingEntry?.updated_at ?? null
      : selectedRecord?.viewer_asset_updated_at ?? null
    : null;
  const selectionKey = selection
    ? selection.kind === "record"
      ? selection.recordId
      : `staging:${selection.runId}:${selection.recordId}`
    : null;
  const missingArtifactsState = useMemo(() => {
    if (selection?.kind !== "record" || !selectedRecord) {
      return null;
    }
    if (!modelLoadState.missingArtifacts) {
      return null;
    }

    const detail = modelLoadState.error
      ? `Viewer request failed: ${modelLoadState.error}`
      : "Materialized files are not currently available for this persisted record.";

    return {
      recordId: selectedRecord.record_id,
      hasCompileReport: selectedRecord.has_compile_report,
      detail,
    };
  }, [modelLoadState.error, modelLoadState.missingArtifacts, selectedRecord, selection]);

  const inspectorUrdfSpec = urdfSpec ? { joints: urdfSpec.joints } : null;
  const displayedJointValues = renderOptions.autoAnimate ? previewJointValues : jointValues;
  const collisionSupport = useMemo<CollisionSupportState | null>(() => {
    if (!selection || !urdfSpec || modelLoadState.loading || modelLoadState.error) {
      return null;
    }

    const collisionCount = urdfSpec.links.reduce((total, link) => total + link.collisions.length, 0);
    if (collisionCount > 0) {
      return {
        available: true,
        summary: `${collisionCount} collision ${collisionCount === 1 ? "mesh" : "meshes"} available`,
        detail: "Collision geometry is available for this asset.",
        compileCommand: null,
      };
    }

    const compileRecordId =
      selection.kind === "record"
        ? selection.recordId
        : selectedStagingEntry?.persisted_record?.record_id ?? null;

    return {
      available: false,
      summary: "No collision geometry in this URDF",
      detail: "This asset currently shows visual meshes only.",
      compileCommand: compileRecordId ? `just compile data/records/${compileRecordId}` : null,
    };
  }, [modelLoadState.error, modelLoadState.loading, selectedStagingEntry?.persisted_record?.record_id, selection, urdfSpec]);

  useEffect(() => {
    if (!collisionSupport || collisionSupport.available || !renderOptions.showCollisions) {
      return;
    }
    setRenderOption("showCollisions", false);
  }, [collisionSupport, renderOptions.showCollisions, setRenderOption]);

  const collisionNotice = collisionSupport && !collisionSupport.available && !missingArtifactsState ? (
    <div className="group relative inline-flex cursor-default">
      <div
        className="inline-flex items-center gap-1.5 rounded-md bg-amber-500/[0.08] px-2.5 py-1 text-[11px] font-medium text-amber-600 shadow-sm backdrop-blur-sm"
        aria-label="Missing collisions. See Render Tab on the right."
      >
        <AlertTriangle className="size-3 shrink-0 opacity-70" />
        <span>Missing Collisions</span>
      </div>
      <span className="pointer-events-none absolute bottom-full left-1/2 z-50 mb-1.5 min-w-max -translate-x-1/2 -translate-y-0.5 rounded-md bg-amber-500/[0.08] px-2.5 py-1.5 text-[11px] font-medium text-amber-600 opacity-0 shadow-sm backdrop-blur-sm transition duration-150 ease-out group-hover:translate-y-0 group-hover:opacity-100">
        See Render Tab on the right
      </span>
    </div>
  ) : null;

  useEffect(() => {
    setModelLoadState({
      loading: false,
      error: null,
      missingArtifacts: false,
    });
  }, [selectionKey]);

  return (
    <div className="relative min-h-0 flex-1">
      <ResizablePanelGroup orientation="horizontal" className="h-full">
        <ResizablePanel defaultSize="20%" minSize="15%" maxSize="30%" className="min-w-0">
          <Sidebar>
            <RecordBrowser />
          </Sidebar>
        </ResizablePanel>
        <ResizableHandle withHandle />
        <ResizablePanel defaultSize="55%" className="min-w-0">
          <ViewportPanel
            baseFileUrl={baseFileUrl}
            assetRevisionKey={assetRevisionKey}
            selectionKey={selectionKey}
            jointPoseSignal={displayedJointValues}
            renderOptions={renderOptions}
            onUrdfSpecChange={handleUrdfSpecChange}
            onInvalidateReady={handleViewportInvalidateReady}
            onLoadStateChange={setModelLoadState}
            overlayNotice={collisionNotice}
            disabledOverlay={
              missingArtifactsState ? (
                <MissingArtifactsOverlay
                  recordId={missingArtifactsState.recordId}
                  hasCompileReport={missingArtifactsState.hasCompileReport}
                  detail={missingArtifactsState.detail}
                />
              ) : undefined
            }
          />
        </ResizablePanel>
        <ResizableHandle withHandle />
        <ResizablePanel
          defaultSize="25%"
          minSize="15%"
          maxSize="35%"
          className="min-w-0"
          collapsible
          panelRef={inspectorPanelRef}
          onResize={handleInspectorResize}
        >
          <Inspector
            disabledOverlay={
              missingArtifactsState ? (
                <MissingArtifactsOverlay
                  recordId={missingArtifactsState.recordId}
                  hasCompileReport={missingArtifactsState.hasCompileReport}
                  detail={missingArtifactsState.detail}
                  compact
                />
              ) : undefined
            }
          >
            <InspectorTabs
              urdfSpec={inspectorUrdfSpec}
              jointValues={displayedJointValues}
              onJointChange={setJointValue}
              onResetAll={resetAll}
              renderOptions={inspectorRenderOptions}
              onRenderOptionChange={handleRenderOptionChange}
              collisionSupport={collisionSupport}
            />
          </Inspector>
        </ResizablePanel>
      </ResizablePanelGroup>
      {inspectorCollapsed ? (
        <Button
          type="button"
          variant="ghost"
          size="sm"
          onClick={handleInspectorExpand}
          aria-label="Expand inspector panel"
          className="absolute right-3 top-1/2 z-20 h-8 w-8 -translate-y-1/2 rounded-full bg-[var(--surface-0)] p-0 text-[var(--text-tertiary)] shadow-[0_2px_8px_rgba(0,0,0,0.08)] hover:text-[var(--text-primary)]"
        >
          <ChevronLeft className="size-4" />
        </Button>
      ) : null}
    </div>
  );
}
