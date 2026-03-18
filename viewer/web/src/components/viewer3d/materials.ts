import * as THREE from 'three';
import type { UrdfVisual } from './urdf-parser';

export interface MaterialSpec {
  color: THREE.Color;
  opacity: number;
  metalness: number;
  roughness: number;
}

/**
 * Resolve visual material specification from URDF data
 * Defaults to a neutral gray if no material specified
 */
export function resolveVisualMaterialSpec(visual: UrdfVisual): MaterialSpec {
  const defaultSpec: MaterialSpec = {
    color: new THREE.Color(0.75, 0.75, 0.75),
    opacity: 1.0,
    metalness: 0.05,
    roughness: 0.48,
  };

  if (!visual.material) {
    return defaultSpec;
  }

  const spec = { ...defaultSpec };

  if (visual.material.color) {
    const [r, g, b, a] = visual.material.color.rgba;
    spec.color = new THREE.Color(r, g, b);
    spec.opacity = a;
  }

  return spec;
}

/**
 * Create a THREE.js material from a material spec
 */
export function createMaterial(spec: MaterialSpec, options?: {
  wireframe?: boolean;
  transparent?: boolean;
  side?: THREE.Side;
}): THREE.MeshStandardMaterial {
  const transparent = options?.transparent ?? (spec.opacity < 0.999);
  const side = options?.side ?? THREE.FrontSide;

  return new THREE.MeshStandardMaterial({
    color: spec.color,
    opacity: spec.opacity,
    metalness: spec.metalness,
    roughness: spec.roughness,
    envMapIntensity: 0.8,
    transparent,
    depthWrite: options?.transparent ? false : spec.opacity >= 0.999,
    forceSinglePass: transparent,
    wireframe: options?.wireframe ?? false,
    side,
  });
}

/**
 * Create edge lines for a geometry
 */
export function createEdgeLines(geometry: THREE.BufferGeometry, color: number = 0x000000): THREE.LineSegments {
  const edges = new THREE.EdgesGeometry(geometry, 15); // 15 degree threshold
  const lineMaterial = new THREE.LineBasicMaterial({ color, opacity: 0.3, transparent: true });
  return new THREE.LineSegments(edges, lineMaterial);
}
