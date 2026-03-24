import { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { describeLinkVisuals, originToMatrix4, parseUrdf, rewriteAbsoluteMeshFilenames } from './urdf-parser';
import type { UrdfSpec } from './urdf-parser';
import { buildRobotSceneGraph, collisionColorForIndex } from './scene-graph-builder';
import { computeFit, preserveViewAcrossModelSwitch, updateCameraClipping } from './camera-utils';
import { positionGroundHelpers } from './lighting';
import { loadGeometryObject } from './geometry-loader';
import { depthBiasForOrdinal, resolveVisualMaterialSpec } from './materials';

/** Sentinel name attached to the robot group so we can find and remove it later. */
const ROBOT_GROUP_NAME = '__articraft_robot__';

function buildClippingTargets(
  robotGroup: THREE.Object3D,
  gridGroup: THREE.Group | null,
  axisGroup: THREE.Group | null,
): THREE.Object3D[] {
  const targets: THREE.Object3D[] = [robotGroup];
  if (gridGroup) {
    targets.push(gridGroup);
  }
  if (axisGroup) {
    targets.push(axisGroup);
  }
  return targets;
}

export interface UrdfLoaderState {
  urdfSpec: UrdfSpec | null;
  jointNodes: Map<string, THREE.Object3D> | null;
  jointFrames: Map<string, THREE.Group> | null;
  loading: boolean;
  error: string | null;
}

/**
 * Hook that fetches and loads a URDF from a base file URL.
 *
 * When baseFileUrl changes the hook:
 * 1. Fetches the URDF from {baseFileUrl}/model.urdf
 * 2. Parses with parseUrdf, rewrites absolute mesh filenames
 * 3. Builds the scene graph with buildRobotSceneGraph
 * 4. Removes the previous robot group from the scene and adds the new one
 * 5. Fits the camera to the new model with computeFit
 * 6. Positions the grid and axis helpers underneath the model
 */
export function useUrdfLoader(
  baseFileUrl: string | null,
  assetRevisionKey: string | null,
  jointPoseSignal: Map<string, number>,
  scene: THREE.Scene | null,
  camera: THREE.PerspectiveCamera | null,
  controls: OrbitControls | null,
  gridGroup: THREE.Group | null,
  axisGroup: THREE.Group | null,
  invalidate: () => void,
): UrdfLoaderState {
  const [urdfSpec, setUrdfSpec] = useState<UrdfSpec | null>(null);
  const [jointNodes, setJointNodes] = useState<Map<string, THREE.Object3D> | null>(null);
  const [jointFrames, setJointFrames] = useState<Map<string, THREE.Group> | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Track the current robot group so we can remove it when swapping models.
  const robotGroupRef = useRef<THREE.Group | null>(null);

  useEffect(() => {
    // Reset when no base URL is available.
    if (!baseFileUrl || !scene) {
      setUrdfSpec(null);
      setJointNodes(null);
      setJointFrames(null);
      setError(null);
      return;
    }

    let cancelled = false;

    const load = async () => {
      setLoading(true);
      setError(null);

      try {
        // 1. Fetch URDF text.
        const urdfUrl = new URL(`${baseFileUrl}/model.urdf`, window.location.origin);
        if (assetRevisionKey) {
          urdfUrl.searchParams.set('rev', assetRevisionKey);
        }

        const response = await fetch(urdfUrl.toString(), {
          cache: assetRevisionKey ? 'no-store' : 'default',
        });
        if (!response.ok) {
          throw new Error(`Failed to fetch URDF: ${response.status} ${response.statusText}`);
        }
        const urdfXml = await response.text();
        if (cancelled) return;

        // 2. Parse and rewrite mesh filenames.
        const rawSpec = parseUrdf(urdfXml);
        const spec = rewriteAbsoluteMeshFilenames(rawSpec);

        // 3. Build scene graph.
        // Always include collision geometry in the scene graph so the render toggle
        // can reveal it without rebuilding the whole robot.
        const { root, jointNodes: joints, jointFrames: frames, linkNodes } = buildRobotSceneGraph(spec, { showCollisions: true });
        await attachMeshGeometry(spec, linkNodes, baseFileUrl, assetRevisionKey);
        if (cancelled) return;

        // 4. Swap robot group in scene.
        const hadPreviousRobot = robotGroupRef.current !== null;
        if (robotGroupRef.current) {
          scene.remove(robotGroupRef.current);
          disposeGroup(robotGroupRef.current);
        }

        const robotGroup = new THREE.Group();
        robotGroup.name = ROBOT_GROUP_NAME;
        robotGroup.rotation.x = -Math.PI / 2;
        robotGroup.add(root);
        scene.add(robotGroup);
        robotGroupRef.current = robotGroup;

        // 5. Fit camera on first load, otherwise preserve the current view direction
        // while re-centering and scaling to the new object.
        if (camera && controls) {
          if (hadPreviousRobot) {
            preserveViewAcrossModelSwitch(camera, controls, robotGroup);
          } else {
            const fit = computeFit(camera, robotGroup);
            camera.position.copy(fit.position);
            controls.target.copy(fit.target);
            camera.near = fit.near;
            camera.far = fit.far;
            camera.updateProjectionMatrix();
            controls.update();
          }
        }

        // 6. Position grid and axis under the model.
        if (gridGroup && axisGroup) {
          positionGroundHelpers(gridGroup, axisGroup, robotGroup);
        }
        if (camera) {
          updateCameraClipping(camera, buildClippingTargets(robotGroup, gridGroup, axisGroup));
        }
        invalidate();

        if (!cancelled) {
          setUrdfSpec(spec);
          setJointNodes(joints);
          setJointFrames(frames);
        }
      } catch (err) {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : 'Unknown error loading URDF');
          setUrdfSpec(null);
          setJointNodes(null);
          setJointFrames(null);
        }
      } finally {
        if (!cancelled) {
          setLoading(false);
        }
      }
    };

    void load();

    return () => {
      cancelled = true;
    };
  }, [baseFileUrl, assetRevisionKey, scene, camera, controls, gridGroup, axisGroup, invalidate]);

  useEffect(() => {
    if (!camera || !controls || !urdfSpec || !robotGroupRef.current) {
      return;
    }

    const robotGroup = robotGroupRef.current;
    const refreshClipping = () => {
      updateCameraClipping(camera, buildClippingTargets(robotGroup, gridGroup, axisGroup));
      invalidate();
    };

    refreshClipping();
    controls.addEventListener('end', refreshClipping);

    return () => {
      controls.removeEventListener('end', refreshClipping);
    };
  }, [camera, controls, urdfSpec, gridGroup, axisGroup, invalidate]);

  useEffect(() => {
    if (!camera || !urdfSpec || !robotGroupRef.current) {
      return;
    }

    updateCameraClipping(
      camera,
      buildClippingTargets(robotGroupRef.current, gridGroup, axisGroup),
    );
    invalidate();
  }, [camera, urdfSpec, gridGroup, axisGroup, jointPoseSignal, invalidate]);

  return { urdfSpec, jointNodes, jointFrames, loading, error };
}

async function attachMeshGeometry(
  spec: UrdfSpec,
  linkNodes: Map<string, THREE.Group>,
  baseUrl: string,
  assetRevisionKey: string | null,
): Promise<void> {
  const pending: Array<Promise<void>> = [];
  let collisionIndex = 0;
  let visualIndex = 0;

  for (const link of spec.links) {
    const linkGroup = linkNodes.get(link.name);
    if (!linkGroup) {
      continue;
    }
    const visualDescriptors = describeLinkVisuals(link);

    for (const [visualIndexWithinLink, visual] of link.visuals.entries()) {
      const depthBias = depthBiasForOrdinal(visualIndex);
      visualIndex += 1;
      if (visual.geometry.type !== 'mesh') {
        continue;
      }

      const visualDescriptor = visualDescriptors[visualIndexWithinLink];
      pending.push(
        loadGeometryObject(visual.geometry, baseUrl, {
          kind: 'visual',
          assetRevisionKey,
          depthBias,
          materialSpec: resolveVisualMaterialSpec(visual),
        }).then((meshGroup) => {
          if (visual.origin) {
            meshGroup.applyMatrix4(originToMatrix4(visual.origin));
          }
          meshGroup.userData.articraftLinkName = link.name;
          meshGroup.userData.articraftVisualKey = visualDescriptor.key;
          meshGroup.userData.articraftVisualLabel = visualDescriptor.label;
          meshGroup.name = visual.name ?? `visual:${link.name}`;
          linkGroup.add(meshGroup);
        }),
      );
    }

    for (const collision of link.collisions) {
      const depthBias = depthBiasForOrdinal(collisionIndex);
      if (collision.geometry.type !== 'mesh') {
        collisionIndex += 1;
        continue;
      }

      const color = collisionColorForIndex(collisionIndex);
      collisionIndex += 1;
      pending.push(
        loadGeometryObject(collision.geometry, baseUrl, {
          kind: 'collision',
          assetRevisionKey,
          depthBias,
          materialSpec: {
            name: 'collision_debug',
            color,
            opacity: 0.88,
            metalness: 0,
            roughness: 1,
            transmission: 0,
            thickness: 0,
            ior: 1.45,
            clearcoat: 0,
            clearcoatRoughness: 0.2,
            envMapIntensity: 0.8,
          },
        }).then((meshGroup) => {
          if (collision.origin) {
            meshGroup.applyMatrix4(originToMatrix4(collision.origin));
          }
          meshGroup.name = collision.name ?? `collision:${link.name}`;
          linkGroup.add(meshGroup);
        }),
      );
    }
  }

  await Promise.all(pending);
}

/**
 * Recursively dispose all geometry and materials within a group.
 */
function disposeGroup(group: THREE.Object3D): void {
  group.traverse((object) => {
    if ("geometry" in object) {
      const geometry = (object as { geometry?: THREE.BufferGeometry }).geometry;
      geometry?.dispose();
    }

    if ("material" in object) {
      const material = (object as { material?: THREE.Material | THREE.Material[] }).material;
      if (Array.isArray(material)) {
        material.forEach((entry) => {
          if (entry instanceof THREE.SpriteMaterial) {
            entry.map?.dispose();
          }
          entry.dispose();
        });
      } else if (material) {
        if (material instanceof THREE.SpriteMaterial) {
          material.map?.dispose();
        }
        material.dispose();
      }
    }
  });
}
