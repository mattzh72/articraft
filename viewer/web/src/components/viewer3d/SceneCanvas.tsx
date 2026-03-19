import { useRef, useEffect, useMemo, useState, type JSX } from 'react';
import * as THREE from 'three';
import { PartLegend, type PartLegendItem } from './PartLegend';
import { useThreeScene } from './useThreeScene';
import { useUrdfLoader } from './useUrdfLoader';
import { createEdgeLines } from './materials';
import { createEnvironmentMap } from './lighting';
import { attachJointOverlay, disposeOverlayObjects } from './joint-overlay';
import type { RenderOptions } from '@/components/inspector/RenderOptionsPanel';
import { updateUrlSearchParams } from '@/lib/url';
import type { UrdfSpec } from './urdf-parser';

const CAMERA_QUERY_PARAM = 'cam';
const ROBOT_GROUP_NAME = '__articraft_robot__';
const CLICK_MOVE_THRESHOLD_PX = 5;
const SEGMENTATION_PALETTE = [
  '#ff5a36',
  '#00b3ff',
  '#ffd400',
  '#16c47f',
  '#ff2f92',
  '#7c5cff',
  '#ff8a00',
  '#00c2a8',
  '#ff6b6b',
  '#145af2',
  '#c4ff0e',
  '#ff3d77',
] as const;
const SEGMENTATION_SNAPSHOT_KEY = '__articraftSegmentColorSnapshot__';

type MaterialWithColor = THREE.Material & {
  color?: THREE.Color;
  emissive?: THREE.Color;
  map?: THREE.Texture | null;
  emissiveMap?: THREE.Texture | null;
  metalness?: number;
  roughness?: number;
  depthWrite?: boolean;
};

type SegmentMaterialSnapshot = {
  color?: THREE.Color;
  emissive?: THREE.Color;
  map?: THREE.Texture | null;
  emissiveMap?: THREE.Texture | null;
  metalness?: number;
  roughness?: number;
  transparent: boolean;
  opacity: number;
  depthWrite?: boolean;
};

type SegmentEmphasis = 'default' | 'selected' | 'dimmed';

function segmentColorForIndex(index: number): THREE.Color {
  const base = new THREE.Color(SEGMENTATION_PALETTE[index % SEGMENTATION_PALETTE.length]);
  const hueOffset = Math.floor(index / SEGMENTATION_PALETTE.length) * 0.07;
  return base.offsetHSL(hueOffset, 0.06, 0);
}

function storeSegmentMaterialSnapshot(material: MaterialWithColor): void {
  if (material.userData[SEGMENTATION_SNAPSHOT_KEY]) {
    return;
  }

  material.userData[SEGMENTATION_SNAPSHOT_KEY] = {
    color: material.color?.clone(),
    emissive: material.emissive?.clone(),
    map: material.map ?? null,
    emissiveMap: material.emissiveMap ?? null,
    metalness: material.metalness,
    roughness: material.roughness,
    transparent: material.transparent,
    opacity: material.opacity,
    depthWrite: material.depthWrite,
  } satisfies SegmentMaterialSnapshot;
}

function applySegmentColor(material: MaterialWithColor, color: THREE.Color, emphasis: SegmentEmphasis): void {
  storeSegmentMaterialSnapshot(material);
  const tint = color.clone();

  if (emphasis === 'dimmed') {
    tint.lerp(new THREE.Color('#d8d8d8'), 0.72);
  }

  if (material.color) {
    material.color.copy(tint);
  }
  if (material.emissive) {
    if (emphasis === 'selected') {
      material.emissive.setRGB(0.09, 0.09, 0.09);
    } else if (emphasis === 'dimmed') {
      material.emissive.setRGB(0.018, 0.018, 0.018);
    } else {
      material.emissive.setRGB(0.06, 0.06, 0.06);
    }
  }
  if ('map' in material) {
    material.map = null;
  }
  if ('emissiveMap' in material) {
    material.emissiveMap = null;
  }
  if ('metalness' in material && typeof material.metalness === 'number') {
    material.metalness = emphasis === 'dimmed' ? 0.02 : 0.14;
  }
  if ('roughness' in material && typeof material.roughness === 'number') {
    material.roughness = emphasis === 'dimmed' ? 0.9 : 0.72;
  }
  material.transparent = emphasis === 'dimmed';
  material.opacity = emphasis === 'dimmed' ? 0.1 : 1;
  if ('depthWrite' in material) {
    material.depthWrite = emphasis !== 'dimmed';
  }
  material.needsUpdate = true;
}

function restoreSegmentMaterial(material: MaterialWithColor): void {
  const snapshot = material.userData[SEGMENTATION_SNAPSHOT_KEY] as SegmentMaterialSnapshot | undefined;
  if (!snapshot) {
    return;
  }

  if (material.color && snapshot.color) {
    material.color.copy(snapshot.color);
  }
  if (material.emissive) {
    if (snapshot.emissive) {
      material.emissive.copy(snapshot.emissive);
    } else {
      material.emissive.setRGB(0, 0, 0);
    }
  }
  if ('map' in material) {
    material.map = snapshot.map ?? null;
  }
  if ('emissiveMap' in material) {
    material.emissiveMap = snapshot.emissiveMap ?? null;
  }
  if ('metalness' in material && typeof snapshot.metalness === 'number') {
    material.metalness = snapshot.metalness;
  }
  if ('roughness' in material && typeof snapshot.roughness === 'number') {
    material.roughness = snapshot.roughness;
  }
  material.transparent = snapshot.transparent;
  material.opacity = snapshot.opacity;
  if ('depthWrite' in material && typeof snapshot.depthWrite === 'boolean') {
    material.depthWrite = snapshot.depthWrite;
  }
  material.needsUpdate = true;
  delete material.userData[SEGMENTATION_SNAPSHOT_KEY];
}

function withMeshMaterials(
  mesh: THREE.Mesh,
  visit: (material: MaterialWithColor) => void,
): void {
  const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
  for (const material of materials) {
    if (material) {
      visit(material as MaterialWithColor);
    }
  }
}

function findLinkName(object: THREE.Object3D | null): string | null {
  let current = object;

  while (current) {
    if (current.name.startsWith('link:')) {
      return current.name.slice(5);
    }
    current = current.parent;
  }

  return null;
}

export interface SceneCanvasProps {
  baseFileUrl: string | null;
  assetRevisionKey: string | null;
  selectionKey: string | null;
  renderOptions: RenderOptions;
  onUrdfSpecChange?: (spec: UrdfSpec | null, jointNodes: Map<string, THREE.Object3D> | null) => void;
  onLoadStateChange?: (state: {
    loading: boolean;
    error: string | null;
    missingArtifacts: boolean;
  }) => void;
}

function isMissingArtifactsError(error: string | null): boolean {
  if (!error) {
    return false;
  }
  return /404|not found|file not found/i.test(error);
}

export function SceneCanvas({
  baseFileUrl,
  assetRevisionKey,
  selectionKey,
  renderOptions,
  onUrdfSpecChange,
  onLoadStateChange,
}: SceneCanvasProps): JSX.Element {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const edgeLinesRef = useRef<THREE.LineSegments[]>([]);
  const envMapRef = useRef<THREE.Texture | null>(null);
  const jointOverlayRef = useRef<THREE.Object3D[]>([]);
  const partHighlightRef = useRef<THREE.LineSegments[]>([]);
  const pointerDownRef = useRef<{ x: number; y: number; pointerId: number } | null>(null);
  const [selectedPartName, setSelectedPartName] = useState<string | null>(null);
  const isStagingSelection = selectionKey?.startsWith("staging:") ?? false;

  const { scene, camera, renderer, controls, gridGroup, axisGroup, sceneReady } = useThreeScene(containerRef);
  const { urdfSpec, jointNodes, jointFrames, loading, error } = useUrdfLoader(
    baseFileUrl,
    assetRevisionKey,
    scene,
    camera,
    controls,
    gridGroup,
    axisGroup,
  );

  const partLegendItems = useMemo<PartLegendItem[]>(
    () => (
      urdfSpec
        ? urdfSpec.links
            .map((link, index) => ({
              name: link.name,
              color: `#${segmentColorForIndex(index).getHexString()}`,
              visualCount: link.visuals.length,
            }))
            .filter((item) => item.visualCount > 0)
            .map(({ name, color }) => ({ name, color }))
        : []
    ),
    [urdfSpec],
  );
  const shouldShowPartLegend =
    Boolean(selectionKey) &&
    renderOptions.showSegmentColors &&
    !renderOptions.showCollisions &&
    partLegendItems.length > 0 &&
    !loading &&
    !error;
  const isStagingBuffering = isStagingSelection && error?.startsWith("Failed to fetch URDF: 404");

  useEffect(() => {
    onUrdfSpecChange?.(urdfSpec, jointNodes);
  }, [urdfSpec, jointNodes, onUrdfSpecChange]);

  useEffect(() => {
    onLoadStateChange?.({
      loading,
      error,
      missingArtifacts: isMissingArtifactsError(error),
    });
  }, [error, loading, onLoadStateChange]);

  useEffect(() => {
    updateUrlSearchParams((params) => {
      params.delete(CAMERA_QUERY_PARAM);
    });
  }, []);

  useEffect(() => {
    setSelectedPartName(null);
  }, [selectionKey]);

  useEffect(() => {
    if (selectedPartName && !partLegendItems.some((item) => item.name === selectedPartName)) {
      setSelectedPartName(null);
    }
  }, [partLegendItems, selectedPartName]);

  useEffect(() => {
    if (renderOptions.showSegmentColors && !renderOptions.showCollisions) {
      return;
    }

    setSelectedPartName(null);
  }, [renderOptions.showCollisions, renderOptions.showSegmentColors]);

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

    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
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
    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
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
    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
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
    if (!scene || !urdfSpec) {
      return;
    }

    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
    if (!robot) {
      return;
    }

    for (const [index, link] of urdfSpec.links.entries()) {
      const linkGroup = robot.getObjectByName(`link:${link.name}`);
      if (!linkGroup) {
        continue;
      }

      const linkColor = segmentColorForIndex(index);
      linkGroup.traverse((obj) => {
        if (!(obj instanceof THREE.Mesh) || obj.userData.articraftVisual !== true) {
          return;
        }

        const emphasis: SegmentEmphasis =
          selectedPartName == null
            ? 'default'
            : link.name === selectedPartName
              ? 'selected'
              : 'dimmed';

        withMeshMaterials(obj, (material) => {
          if (renderOptions.showSegmentColors) {
            applySegmentColor(material, linkColor, emphasis);
          } else {
            restoreSegmentMaterial(material);
          }
        });
      });
    }

    return () => {
      for (const link of urdfSpec.links) {
        const linkGroup = robot.getObjectByName(`link:${link.name}`);
        if (!linkGroup) {
          continue;
        }

        linkGroup.traverse((obj) => {
          if (!(obj instanceof THREE.Mesh) || obj.userData.articraftVisual !== true) {
            return;
          }

          withMeshMaterials(obj, (material) => {
            restoreSegmentMaterial(material);
          });
        });
      }
    };
  }, [scene, renderOptions.showSegmentColors, selectedPartName, urdfSpec]);

  useEffect(() => {
    for (const highlight of partHighlightRef.current) {
      highlight.parent?.remove(highlight);
      highlight.geometry.dispose();
      (highlight.material as THREE.Material).dispose();
    }
    partHighlightRef.current = [];

    if (!scene || !selectedPartName || !shouldShowPartLegend) {
      return;
    }

    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
    const linkGroup = robot?.getObjectByName(`link:${selectedPartName}`);
    if (!linkGroup) {
      return;
    }

    const nextHighlights: THREE.LineSegments[] = [];
    linkGroup.traverse((obj) => {
      if (!(obj instanceof THREE.Mesh) || obj.userData.articraftVisual !== true || !obj.visible) {
        return;
      }

      const highlight = createEdgeLines(obj.geometry, 0xffffff);
      const material = highlight.material as THREE.LineBasicMaterial;
      material.opacity = 0.72;
      highlight.renderOrder = 20;
      obj.add(highlight);
      nextHighlights.push(highlight);
    });

    partHighlightRef.current = nextHighlights;

    return () => {
      for (const highlight of nextHighlights) {
        highlight.parent?.remove(highlight);
        highlight.geometry.dispose();
        (highlight.material as THREE.Material).dispose();
      }
      if (partHighlightRef.current === nextHighlights) {
        partHighlightRef.current = [];
      }
    };
  }, [scene, selectedPartName, shouldShowPartLegend, urdfSpec]);

  useEffect(() => {
    if (!scene || !camera || !renderer || !shouldShowPartLegend) {
      return;
    }

    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
    if (!robot) {
      return;
    }

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();

    const handlePointerDown = (event: PointerEvent): void => {
      pointerDownRef.current = {
        x: event.clientX,
        y: event.clientY,
        pointerId: event.pointerId,
      };
    };

    const clearPointerDown = (): void => {
      pointerDownRef.current = null;
    };

    const handlePointerUp = (event: PointerEvent): void => {
      const start = pointerDownRef.current;
      pointerDownRef.current = null;

      if (!start || start.pointerId !== event.pointerId) {
        return;
      }

      const movement = Math.hypot(event.clientX - start.x, event.clientY - start.y);
      if (movement > CLICK_MOVE_THRESHOLD_PX) {
        return;
      }

      const bounds = renderer.domElement.getBoundingClientRect();
      if (bounds.width <= 0 || bounds.height <= 0) {
        return;
      }

      pointer.x = ((event.clientX - bounds.left) / bounds.width) * 2 - 1;
      pointer.y = -((event.clientY - bounds.top) / bounds.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);

      const intersections = raycaster.intersectObject(robot, true);
      const hit = intersections.find(
        ({ object }) => object instanceof THREE.Mesh && object.userData.articraftVisual === true && object.visible,
      );
      const linkName = hit ? findLinkName(hit.object) : null;
      if (!linkName) {
        return;
      }

      setSelectedPartName((current) => (current === linkName ? null : linkName));
    };

    renderer.domElement.addEventListener('pointerdown', handlePointerDown);
    renderer.domElement.addEventListener('pointerup', handlePointerUp);
    renderer.domElement.addEventListener('pointercancel', clearPointerDown);
    renderer.domElement.addEventListener('pointerleave', clearPointerDown);
    return () => {
      renderer.domElement.removeEventListener('pointerdown', handlePointerDown);
      renderer.domElement.removeEventListener('pointerup', handlePointerUp);
      renderer.domElement.removeEventListener('pointercancel', clearPointerDown);
      renderer.domElement.removeEventListener('pointerleave', clearPointerDown);
    };
  }, [camera, renderer, scene, shouldShowPartLegend]);

  useEffect(() => {
    disposeOverlayObjects(jointOverlayRef.current);
    jointOverlayRef.current = [];

    if (!renderOptions.showJointOverlay || !urdfSpec || !jointFrames || !scene) {
      return;
    }

    const robot = scene.getObjectByName(ROBOT_GROUP_NAME);
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

      {shouldShowPartLegend ? (
        <PartLegend
          items={partLegendItems}
          selectedPartName={selectedPartName}
          onSelectPart={setSelectedPartName}
        />
      ) : null}

      {loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#f3f3f3]/70">
          <p className="text-[11px] text-[#999]">{isStagingSelection ? "Buffering...." : "Loading…"}</p>
        </div>
      )}

      {isStagingBuffering && !loading && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#f3f3f3]/70">
          <p className="text-[11px] text-[#999]">Buffering....</p>
        </div>
      )}

      {error && !loading && !isStagingBuffering && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#f3f3f3]/70">
          <div className="max-w-xs px-3 py-2 text-center">
            <p className="text-[11px] font-medium text-red-600">Failed to load model</p>
            <p className="mt-0.5 font-mono text-[10px] text-red-400">{error}</p>
          </div>
        </div>
      )}

      {!selectionKey && sceneReady && !loading && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center">
          <p className="text-[11px] text-[#bbb]">
            Select a record to view its 3D model
          </p>
        </div>
      )}
    </div>
  );
}
