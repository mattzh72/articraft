import { useRef, useEffect, type JSX } from 'react';
import * as THREE from 'three';
import { useThreeScene } from './useThreeScene';
import { useUrdfLoader } from './useUrdfLoader';
import { createEdgeLines } from './materials';
import { createEnvironmentMap } from './lighting';
import { attachJointOverlay, disposeOverlayObjects } from './joint-overlay';
import type { RenderOptions } from '@/components/inspector/RenderOptionsPanel';
import { updateUrlSearchParams } from '@/lib/url';
import type { UrdfSpec } from './urdf-parser';

const CAMERA_QUERY_PARAM = 'cam';

export interface SceneCanvasProps {
  recordId: string | null;
  renderOptions: RenderOptions;
  onUrdfSpecChange?: (spec: UrdfSpec | null, jointNodes: Map<string, THREE.Object3D> | null) => void;
}

export function SceneCanvas({ recordId, renderOptions, onUrdfSpecChange }: SceneCanvasProps): JSX.Element {
  const containerRef = useRef<HTMLDivElement | null>(null);

  const { scene, camera, renderer, controls, gridGroup, axisGroup, sceneReady } = useThreeScene(containerRef);
  const { urdfSpec, jointNodes, jointFrames, loading, error } = useUrdfLoader(
    recordId,
    scene,
    camera,
    controls,
    gridGroup,
    axisGroup,
  );

  useEffect(() => {
    onUrdfSpecChange?.(urdfSpec, jointNodes);
  }, [urdfSpec, jointNodes, onUrdfSpecChange]);

  useEffect(() => {
    updateUrlSearchParams((params) => {
      params.delete(CAMERA_QUERY_PARAM);
    });
  }, []);

  const edgeLinesRef = useRef<THREE.LineSegments[]>([]);
  const envMapRef = useRef<THREE.Texture | null>(null);
  const jointOverlayRef = useRef<THREE.Object3D[]>([]);

  // Show/hide grid
  useEffect(() => {
    if (gridGroup) {
      gridGroup.visible = renderOptions.showGrid;
    }
  }, [gridGroup, renderOptions.showGrid]);

  // Show edges
  useEffect(() => {
    if (!scene) return;

    for (const line of edgeLinesRef.current) {
      line.parent?.remove(line);
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
    }
    edgeLinesRef.current = [];

    if (!renderOptions.showEdges || renderOptions.showCollisions) return;

    const robot = scene.getObjectByName('__articraft_robot__');
    if (!robot) return;

    const newLines: THREE.LineSegments[] = [];
    robot.traverse((obj) => {
      if (obj instanceof THREE.Mesh && obj.geometry && obj.userData.articraftVisual === true && obj.visible) {
        const edges = createEdgeLines(obj.geometry);
        obj.add(edges);
        newLines.push(edges);
      }
    });
    edgeLinesRef.current = newLines;
  }, [scene, renderOptions.showEdges, renderOptions.showCollisions, urdfSpec]);

  // Double-sided materials
  useEffect(() => {
    if (!scene) return;
    const robot = scene.getObjectByName('__articraft_robot__');
    if (!robot) return;

    robot.traverse((obj) => {
      if (obj instanceof THREE.Mesh) {
        const mat = obj.material;
        const side = obj.userData.articraftCollision === true || renderOptions.doubleSided
          ? THREE.DoubleSide
          : THREE.FrontSide;
        if (Array.isArray(mat)) {
          for (const m of mat) {
            m.side = side;
            m.needsUpdate = true;
          }
        } else if (mat) {
          mat.side = side;
          mat.needsUpdate = true;
        }
      }
    });
  }, [scene, renderOptions.doubleSided, urdfSpec]);

  // Environment lighting
  useEffect(() => {
    if (!scene || !renderer) return;

    if (renderOptions.environmentLighting) {
      if (!envMapRef.current) {
        envMapRef.current = createEnvironmentMap(renderer);
      }
      scene.environment = envMapRef.current;
    } else {
      scene.environment = null;
    }
  }, [scene, renderer, renderOptions.environmentLighting]);

  // Show collisions
  useEffect(() => {
    if (!scene) return;
    const robot = scene.getObjectByName('__articraft_robot__');
    if (!robot) return;

    robot.traverse((obj) => {
      if (obj.userData.articraftVisual === true) {
        obj.visible = !renderOptions.showCollisions;
      }
      if (obj.userData.articraftCollision === true) {
        obj.visible = renderOptions.showCollisions;
      }
    });
  }, [scene, renderOptions.showCollisions, urdfSpec]);

  useEffect(() => {
    disposeOverlayObjects(jointOverlayRef.current);
    jointOverlayRef.current = [];

    if (!renderOptions.showJointOverlay || !urdfSpec || !jointFrames || !scene) {
      return;
    }

    const robot = scene.getObjectByName('__articraft_robot__');
    if (!robot) {
      return;
    }

    const size = new THREE.Box3().setFromObject(robot).getSize(new THREE.Vector3());
    const extent = Math.max(size.x, size.y, size.z, 0.6);
    jointOverlayRef.current = attachJointOverlay(urdfSpec, jointFrames, extent);

    return () => {
      disposeOverlayObjects(jointOverlayRef.current);
      jointOverlayRef.current = [];
    };
  }, [jointFrames, renderOptions.showJointOverlay, scene, urdfSpec]);

  return (
    <div className="relative h-full w-full">
      <div ref={containerRef} className="h-full w-full" />

      {loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#f3f3f3]/70">
          <p className="text-[11px] text-[#999]">Loading…</p>
        </div>
      )}

      {error && !loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#f3f3f3]/70">
          <div className="max-w-xs px-3 py-2 text-center">
            <p className="text-[11px] font-medium text-red-600">Failed to load model</p>
            <p className="mt-0.5 font-mono text-[10px] text-red-400">{error}</p>
          </div>
        </div>
      )}

      {!recordId && sceneReady && !loading && (
        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
          <p className="text-[11px] text-[#bbb]">
            Select a record to view its 3D model
          </p>
        </div>
      )}
    </div>
  );
}
