import { useState, useCallback, useEffect, useRef, type JSX } from "react";
import { ChevronLeft } from "lucide-react";
import type { PanelImperativeHandle, PanelSize } from "react-resizable-panels";
import * as THREE from "three";

import { useViewer, ViewerProvider } from "@/lib/viewer-context";
import { updateUrlSearchParams } from "@/lib/url";
import {
  ResizablePanelGroup,
  ResizablePanel,
  ResizableHandle,
} from "@/components/ui/resizable";
import { TooltipProvider } from "@/components/ui/tooltip";
import { AppHeader } from "@/components/layout/AppHeader";
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

const JOINT_POSE_QUERY_PARAM = "pose";
const INSPECTOR_QUERY_PARAM = "inspector";
const JOINT_POSE_PRECISION = 1e5;

type JointPoseSnapshot = {
  recordId: string;
  values: Record<string, number>;
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

function previewJointValue(joint: UrdfJoint, phase: number): number {
  const wave = Math.sin(phase * Math.PI * 2);

  if (joint.type === "continuous") {
    return THREE.MathUtils.euclideanModulo((phase * Math.PI * 2) + Math.PI, Math.PI * 2) - Math.PI;
  }

  const lower = joint.limit?.lower;
  const upper = joint.limit?.upper;
  const hasRange = Number.isFinite(lower) && Number.isFinite(upper) && lower !== undefined && upper !== undefined && upper > lower;

  if (hasRange) {
    const center = (lower + upper) / 2;
    const amplitude = (upper - lower) * 0.46;
    return center + wave * amplitude;
  }

  if (joint.type === "prismatic") {
    return wave * 0.12;
  }

  return wave * (Math.PI / 3);
}

function ViewerShell(): JSX.Element {
  const { selectedRecordId } = useViewer();
  const { options: renderOptions, setOption: setRenderOption } = useRenderOptions();

  const [urdfSpec, setUrdfSpec] = useState<UrdfSpec | null>(null);
  const [jointNodes, setJointNodes] = useState<Map<string, THREE.Object3D> | null>(null);
  const [previewJointValues, setPreviewJointValues] = useState<Map<string, number>>(new Map());
  const [inspectorCollapsed, setInspectorCollapsed] = useState(readInspectorCollapsedFromUrl);
  const inspectorPanelRef = useRef<PanelImperativeHandle | null>(null);
  const initialInspectorCollapsedRef = useRef(inspectorCollapsed);

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
      if (key === "environmentLighting") {
        setRenderOption("envLighting", value);
        return;
      }

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
      setPreviewJointValues(new Map());
      applyJointValues(jointValues);
      return;
    }

    const previewJoints = buildPreviewJointSequence(urdfSpec);
    if (previewJoints.length === 0) {
      setPreviewJointValues(new Map());
      return;
    }

    const secondsPerCycle = THREE.MathUtils.clamp(previewJoints.length * 0.9, 3.2, 10);
    let frameId = 0;

    const tick = (now: number) => {
      const elapsedSeconds = now / 1000;
      const basePhase = (elapsedSeconds % secondsPerCycle) / secondsPerCycle;
      const nextValues = new Map<string, number>();

      for (const [index, joint] of previewJoints.entries()) {
        const jointPhase = THREE.MathUtils.euclideanModulo(
          basePhase - index / previewJoints.length,
          1,
        );
        nextValues.set(joint.name, previewJointValue(joint, jointPhase));
      }

      setPreviewJointValues(nextValues);
      applyJointValues(nextValues);
      frameId = requestAnimationFrame(tick);
    };

    frameId = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(frameId);
      setPreviewJointValues(new Map());
      applyJointValues(jointValues);
    };
  }, [applyJointValues, jointValues, renderOptions.autoAnimate, urdfSpec]);

  const inspectorRenderOptions = {
    showEdges: renderOptions.showEdges,
    showGrid: renderOptions.showGrid,
    showCollisions: renderOptions.showCollisions,
    doubleSided: renderOptions.doubleSided,
    environmentLighting: renderOptions.envLighting,
    autoAnimate: renderOptions.autoAnimate,
    showJointOverlay: renderOptions.showJointOverlay,
  };

  const inspectorUrdfSpec = urdfSpec ? { joints: urdfSpec.joints } : null;
  const displayedJointValues = renderOptions.autoAnimate ? previewJointValues : jointValues;

  return (
    <div className="flex h-screen flex-col bg-[var(--surface-2)]">
      <AppHeader />
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
              recordId={selectedRecordId}
              renderOptions={renderOptions}
              onUrdfSpecChange={handleUrdfSpecChange}
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
            <Inspector>
              <InspectorTabs
                urdfSpec={inspectorUrdfSpec}
                jointValues={displayedJointValues}
                onJointChange={setJointValue}
                onResetAll={resetAll}
                renderOptions={inspectorRenderOptions}
                onRenderOptionChange={handleRenderOptionChange}
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
    </div>
  );
}

export default function App(): JSX.Element {
  return (
    <ViewerProvider>
      <TooltipProvider>
        <ViewerShell />
      </TooltipProvider>
    </ViewerProvider>
  );
}
