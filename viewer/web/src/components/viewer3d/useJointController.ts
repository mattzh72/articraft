import { useState, useCallback, useRef, useEffect } from 'react';
import * as THREE from 'three';
import type { UrdfSpec, UrdfJoint } from './urdf-parser';

export interface JointControllerState {
  jointValues: Map<string, number>;
  setJointValue: (name: string, value: number) => void;
  applyJointValues: (values: Map<string, number>, options?: { commit?: boolean }) => void;
  resetAll: () => void;
}

/**
 * Hook for managing joint values and applying them to the Three.js scene graph.
 *
 * Maintains a Map<string, number> of joint name to current value.
 * setJointValue applies the appropriate rotation (revolute/continuous) or
 * translation (prismatic) to the joint node based on the URDF axis definition.
 */
export function useJointController(
  jointNodes: Map<string, THREE.Object3D> | null,
  urdfSpec: UrdfSpec | null,
): JointControllerState {
  const [jointValues, setJointValues] = useState<Map<string, number>>(new Map());

  // Cache the joint spec lookup for quick access.
  const jointSpecMap = useRef<Map<string, UrdfJoint>>(new Map());

  // Rebuild the spec lookup whenever the urdfSpec changes.
  useEffect(() => {
    const map = new Map<string, UrdfJoint>();
    if (urdfSpec) {
      for (const joint of urdfSpec.joints) {
        map.set(joint.name, joint);
      }
    }
    jointSpecMap.current = map;

    // Reset values when the spec changes (new model loaded).
    setJointValues(new Map());
  }, [urdfSpec]);

  const setJointValue = useCallback(
    (name: string, value: number) => {
      if (!jointNodes || !jointSpecMap.current) return;

      const node = jointNodes.get(name);
      const spec = jointSpecMap.current.get(name);
      if (!node || !spec) return;

      // Default axis is Z (0, 0, 1) per URDF specification.
      const axis = new THREE.Vector3(...(spec.axis ?? [0, 0, 1])).normalize();

      applyJointValue(node, spec, axis, value);

      setJointValues((prev) => {
        const next = new Map(prev);
        next.set(name, value);
        return next;
      });
    },
    [jointNodes],
  );

  const applyJointValues = useCallback(
    (values: Map<string, number>, options?: { commit?: boolean }) => {
      if (!jointNodes || !jointSpecMap.current) return;

      for (const [name, node] of jointNodes) {
        const spec = jointSpecMap.current.get(name);
        if (!spec) {
          continue;
        }

        const axis = new THREE.Vector3(...(spec.axis ?? [0, 0, 1])).normalize();
        applyJointValue(node, spec, axis, values.get(name) ?? 0);
      }

      if (options?.commit) {
        setJointValues(new Map(values));
      }
    },
    [jointNodes],
  );

  const resetAll = useCallback(() => {
    applyJointValues(new Map(), { commit: true });
  }, [applyJointValues]);

  return { jointValues, setJointValue, applyJointValues, resetAll };
}

/**
 * Apply a joint value to a Three.js node.
 *
 * For revolute/continuous joints, a quaternion rotation around the joint axis is applied.
 * For prismatic joints, a translation along the joint axis is applied.
 */
function applyJointValue(
  node: THREE.Object3D,
  spec: UrdfJoint,
  axis: THREE.Vector3,
  value: number,
): void {
  switch (spec.type) {
    case 'revolute':
    case 'continuous': {
      const quaternion = new THREE.Quaternion();
      quaternion.setFromAxisAngle(axis, value);
      node.quaternion.copy(quaternion);
      break;
    }
    case 'prismatic': {
      // Reset to origin translation then apply offset along axis.
      const offset = axis.clone().multiplyScalar(value);
      node.position.set(offset.x, offset.y, offset.z);
      break;
    }
    default:
      // Fixed/floating/planar joints are not user-controllable.
      break;
  }
}
