import * as THREE from 'three';
import type { UrdfVisual } from './urdf-parser';

export interface MaterialSpec {
  name?: string;
  color: THREE.Color;
  opacity: number;
  metalness: number;
  roughness: number;
  transmission: number;
  thickness: number;
  ior: number;
  clearcoat: number;
  clearcoatRoughness: number;
  envMapIntensity: number;
}

export function depthBiasForOrdinal(ordinal: number): number {
  return 1 + ordinal * 0.25;
}

/**
 * Resolve visual material specification from URDF data
 * Defaults to a neutral gray if no material specified
 */
export function resolveVisualMaterialSpec(visual: UrdfVisual): MaterialSpec {
  const defaultSpec: MaterialSpec = {
    name: undefined,
    color: new THREE.Color(0.75, 0.75, 0.75),
    opacity: 1.0,
    metalness: 0.05,
    roughness: 0.48,
    transmission: 0,
    thickness: 0,
    ior: 1.45,
    clearcoat: 0,
    clearcoatRoughness: 0.2,
    envMapIntensity: 0.8,
  };

  if (!visual.material) {
    return defaultSpec;
  }

  const spec = { ...defaultSpec };
  spec.name = visual.material.name;

  if (visual.material.color) {
    const [r, g, b, a] = visual.material.color.rgba;
    spec.color = new THREE.Color(r, g, b);
    spec.opacity = a;
  }

  return applyNamedMaterialPreset(spec);
}

function applyNamedMaterialPreset(spec: MaterialSpec): MaterialSpec {
  const name = spec.name?.toLowerCase() ?? '';
  const tuned = { ...spec };
  const tokens = name.split(/[^a-z0-9]+/).filter(Boolean);
  const hasToken = (token: string): boolean => tokens.includes(token);
  const hasAnyToken = (...candidates: string[]): boolean => candidates.some(hasToken);
  const mentionsTransparentDescriptor = hasAnyToken(
    'clear',
    'glass',
    'transparent',
    'translucent',
    'smoked',
    'frosted',
  );
  const mentionsTransparentPlasticFamily = hasAnyToken(
    'polycarbonate',
    'acrylic',
    'plexi',
    'plexiglass',
    'lexan',
  );
  const isClearRigidMaterial = (
    mentionsTransparentDescriptor
    || (mentionsTransparentPlasticFamily && tuned.opacity < 0.999)
  );

  if (isClearRigidMaterial) {
    tuned.metalness = 0.0;
    tuned.roughness = 0.08;
    tuned.transmission = Math.max(0.45, 1.0 - tuned.opacity * 0.55);
    tuned.thickness = 0.006;
    tuned.ior = 1.45;
    tuned.clearcoat = 0.9;
    tuned.clearcoatRoughness = 0.06;
    tuned.envMapIntensity = 1.25;
    return tuned;
  }

  if (name.includes('reflector')) {
    tuned.metalness = 0.9;
    tuned.roughness = 0.18;
    tuned.opacity = Math.max(tuned.opacity, 0.78);
    tuned.clearcoat = 0.18;
    tuned.clearcoatRoughness = 0.12;
    tuned.envMapIntensity = 1.4;
    return tuned;
  }

  if (name.includes('brass') || name.includes('metal')) {
    tuned.metalness = 0.88;
    tuned.roughness = 0.22;
    tuned.clearcoat = 0.16;
    tuned.clearcoatRoughness = 0.18;
    tuned.envMapIntensity = 1.3;
    return tuned;
  }

  if (name.includes('bakelite')) {
    tuned.metalness = 0.02;
    tuned.roughness = 0.38;
    tuned.clearcoat = 0.24;
    tuned.clearcoatRoughness = 0.2;
    tuned.envMapIntensity = 0.7;
    return tuned;
  }

  if (name.includes('felt')) {
    tuned.metalness = 0.0;
    tuned.roughness = 0.92;
    tuned.clearcoat = 0.0;
    tuned.envMapIntensity = 0.25;
  }

  return tuned;
}

/**
 * Create a THREE.js material from a material spec
 */
export function createMaterial(spec: MaterialSpec, options?: {
  wireframe?: boolean;
  transparent?: boolean;
  side?: THREE.Side;
  depthBias?: number;
}): THREE.MeshPhysicalMaterial {
  const transparent = options?.transparent ?? (spec.opacity < 0.999 || spec.transmission > 0.01);
  const side = options?.side ?? THREE.FrontSide;
  const depthBias = options?.depthBias ?? 0;

  return new THREE.MeshPhysicalMaterial({
    color: spec.color,
    opacity: spec.opacity,
    metalness: spec.metalness,
    roughness: spec.roughness,
    transmission: spec.transmission,
    thickness: spec.thickness,
    ior: spec.ior,
    clearcoat: spec.clearcoat,
    clearcoatRoughness: spec.clearcoatRoughness,
    envMapIntensity: spec.envMapIntensity,
    transparent,
    depthWrite: !transparent,
    forceSinglePass: transparent,
    wireframe: options?.wireframe ?? false,
    side,
    polygonOffset: depthBias !== 0,
    polygonOffsetFactor: 0,
    polygonOffsetUnits: depthBias,
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
