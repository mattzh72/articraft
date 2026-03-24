import { type JSX, type ReactNode } from "react";
import type * as THREE from "three";

import { SceneCanvas } from "@/components/viewer3d/SceneCanvas";
import type { RenderOptions } from "@/components/viewer3d/useRenderOptions";
import type { UrdfSpec } from "@/components/viewer3d/urdf-parser";
import { cn } from "@/lib/utils";

interface ViewportPanelProps {
  baseFileUrl: string | null;
  assetRevisionKey: string | null;
  selectionKey: string | null;
  jointPoseSignal: Map<string, number>;
  renderOptions: RenderOptions;
  onUrdfSpecChange: (spec: UrdfSpec | null, jointNodes: Map<string, THREE.Object3D> | null) => void;
  onInvalidateReady?: (invalidate: (() => void) | null) => void;
  onLoadStateChange?: (state: {
    loading: boolean;
    error: string | null;
    missingArtifacts: boolean;
  }) => void;
  overlayNotice?: ReactNode;
  disabledOverlay?: ReactNode;
}

export function ViewportPanel({
  baseFileUrl,
  assetRevisionKey,
  selectionKey,
  jointPoseSignal,
  renderOptions,
  onUrdfSpecChange,
  onInvalidateReady,
  onLoadStateChange,
  overlayNotice,
  disabledOverlay,
}: ViewportPanelProps): JSX.Element {
  // Map from useRenderOptions format to RenderOptionsPanel format for SceneCanvas
  const sceneRenderOptions = {
    showEdges: renderOptions.showEdges,
    showGrid: renderOptions.showGrid,
    showCollisions: renderOptions.showCollisions,
    showSegmentColors: renderOptions.showSegmentColors,
    showSurfaceSamples: renderOptions.showSurfaceSamples,
    doubleSided: renderOptions.doubleSided,
    environmentLighting: renderOptions.envLighting,
    autoAnimate: renderOptions.autoAnimate,
    showJointOverlay: renderOptions.showJointOverlay,
  };

  return (
    <section className="relative h-full w-full min-w-0 bg-[var(--surface-2)]">
      <div className="absolute inset-0 overflow-hidden">
        <div
          className={cn(
            "h-full w-full transition duration-200",
            disabledOverlay ? "pointer-events-none select-none blur-[8px] saturate-50 opacity-35" : "",
          )}
        >
          <SceneCanvas
            baseFileUrl={baseFileUrl}
            assetRevisionKey={assetRevisionKey}
            selectionKey={selectionKey}
            jointPoseSignal={jointPoseSignal}
            renderOptions={sceneRenderOptions}
            onUrdfSpecChange={onUrdfSpecChange}
            onInvalidateReady={onInvalidateReady}
            onLoadStateChange={onLoadStateChange}
          />
        </div>
      </div>
      {overlayNotice ? <div className="absolute inset-x-0 bottom-0 z-[5] p-3">{overlayNotice}</div> : null}
      {disabledOverlay ? <div className="absolute inset-0 z-10">{disabledOverlay}</div> : null}
    </section>
  );
}
