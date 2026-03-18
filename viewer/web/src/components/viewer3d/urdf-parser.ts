import * as THREE from 'three';

export interface UrdfJoint {
  name: string;
  type: 'fixed' | 'revolute' | 'continuous' | 'prismatic' | 'floating' | 'planar';
  parent: string;
  child: string;
  origin?: {
    xyz?: [number, number, number];
    rpy?: [number, number, number];
  };
  axis?: [number, number, number];
  limit?: {
    lower?: number;
    upper?: number;
    effort?: number;
    velocity?: number;
  };
}

export interface UrdfVisualGeometry {
  type: 'box' | 'cylinder' | 'sphere' | 'mesh';
  size?: [number, number, number]; // box
  radius?: number; // cylinder, sphere
  length?: number; // cylinder
  filename?: string; // mesh
  scale?: [number, number, number]; // mesh
}

export interface UrdfVisual {
  name?: string;
  origin?: {
    xyz?: [number, number, number];
    rpy?: [number, number, number];
  };
  geometry: UrdfVisualGeometry;
  material?: {
    name?: string;
    color?: { rgba: [number, number, number, number] };
    texture?: { filename: string };
  };
}

type UrdfMaterial = NonNullable<UrdfVisual['material']>;

export interface UrdfLink {
  name: string;
  visuals: UrdfVisual[];
  collisions: UrdfVisual[]; // Collision has same structure as visual
}

export interface UrdfSpec {
  name: string;
  links: UrdfLink[];
  joints: UrdfJoint[];
}

/**
 * Parse a vec3 string like "1 2 3" or "1.0 2.0 3.0"
 */
export function parseVec3(str: string | undefined): [number, number, number] {
  if (!str) return [0, 0, 0];
  const parts = str.trim().split(/\s+/).map(parseFloat);
  return [
    Number.isFinite(parts[0]) ? parts[0] : 0,
    Number.isFinite(parts[1]) ? parts[1] : 0,
    Number.isFinite(parts[2]) ? parts[2] : 0,
  ];
}

/**
 * Parse a vec4 string like "1 0 0 1" for RGBA
 */
export function parseVec4(str: string | undefined): [number, number, number, number] {
  if (!str) return [1, 1, 1, 1];
  const parts = str.trim().split(/\s+/).map(parseFloat);
  return [
    Number.isFinite(parts[0]) ? parts[0] : 1,
    Number.isFinite(parts[1]) ? parts[1] : 1,
    Number.isFinite(parts[2]) ? parts[2] : 1,
    Number.isFinite(parts[3]) ? parts[3] : 1,
  ];
}

/**
 * Convert URDF fixed-axis RPY (roll, pitch, yaw) to THREE.Matrix4.
 * This matches the legacy synth-urdf viewer: Rz(yaw) * Ry(pitch) * Rx(roll).
 */
export function rpyToMatrix4(rpy: [number, number, number]): THREE.Matrix4 {
  const [roll, pitch, yaw] = rpy;
  const mx = new THREE.Matrix4().makeRotationX(roll);
  const my = new THREE.Matrix4().makeRotationY(pitch);
  const mz = new THREE.Matrix4().makeRotationZ(yaw);
  return new THREE.Matrix4().multiplyMatrices(mz, new THREE.Matrix4().multiplyMatrices(my, mx));
}

/**
 * Convert URDF origin (xyz + rpy) to THREE.Matrix4
 */
export function originToMatrix4(origin?: { xyz?: [number, number, number]; rpy?: [number, number, number] }): THREE.Matrix4 {
  const xyz = origin?.xyz ?? [0, 0, 0];
  const rpy = origin?.rpy ?? [0, 0, 0];
  const translation = new THREE.Matrix4().makeTranslation(xyz[0], xyz[1], xyz[2]);
  const rotation = rpyToMatrix4(rpy);
  return new THREE.Matrix4().multiplyMatrices(translation, rotation);
}

/**
 * Parse <origin> element
 */
function parseOrigin(originEl: Element | null): { xyz?: [number, number, number]; rpy?: [number, number, number] } | undefined {
  if (!originEl) return undefined;

  const xyzStr = originEl.getAttribute('xyz') ?? undefined;
  const rpyStr = originEl.getAttribute('rpy') ?? undefined;

  const result: { xyz?: [number, number, number]; rpy?: [number, number, number] } = {};

  if (xyzStr) result.xyz = parseVec3(xyzStr);
  if (rpyStr) result.rpy = parseVec3(rpyStr);

  return Object.keys(result).length > 0 ? result : undefined;
}

/**
 * Parse <geometry> element
 */
function parseGeometry(geomEl: Element | null): UrdfVisualGeometry | null {
  if (!geomEl) return null;

  const boxEl = geomEl.querySelector('box');
  if (boxEl) {
    const sizeStr = boxEl.getAttribute('size') ?? undefined;
    return {
      type: 'box',
      size: parseVec3(sizeStr),
    };
  }

  const cylinderEl = geomEl.querySelector('cylinder');
  if (cylinderEl) {
    return {
      type: 'cylinder',
      radius: parseFloat(cylinderEl.getAttribute('radius') || '1'),
      length: parseFloat(cylinderEl.getAttribute('length') || '1'),
    };
  }

  const sphereEl = geomEl.querySelector('sphere');
  if (sphereEl) {
    return {
      type: 'sphere',
      radius: parseFloat(sphereEl.getAttribute('radius') || '1'),
    };
  }

  const meshEl = geomEl.querySelector('mesh');
  if (meshEl) {
    const scaleStr = meshEl.getAttribute('scale') ?? undefined;
    return {
      type: 'mesh',
      filename: meshEl.getAttribute('filename') || '',
      scale: scaleStr ? parseVec3(scaleStr) : undefined,
    };
  }

  return null;
}

/**
 * Parse <material> element
 */
function parseMaterial(matEl: Element | null): UrdfVisual['material'] | undefined {
  if (!matEl) return undefined;

  const name = matEl.getAttribute('name') || undefined;
  const colorEl = matEl.querySelector('color');
  const textureEl = matEl.querySelector('texture');

  const material: UrdfVisual['material'] = { name };

  if (colorEl) {
    const rgbaStr = colorEl.getAttribute('rgba') ?? undefined;
    material.color = { rgba: parseVec4(rgbaStr) };
  }

  if (textureEl) {
    material.texture = { filename: textureEl.getAttribute('filename') || '' };
  }

  return material;
}

function resolveMaterial(
  material: UrdfVisual['material'],
  materialLibrary: Map<string, UrdfMaterial>,
): UrdfVisual['material'] {
  if (!material?.name) {
    return material;
  }

  const libraryMaterial = materialLibrary.get(material.name);
  if (!libraryMaterial) {
    return material;
  }

  return {
    ...libraryMaterial,
    ...material,
    color: material.color ?? libraryMaterial.color,
    texture: material.texture ?? libraryMaterial.texture,
  };
}

/**
 * Parse <visual> or <collision> element
 */
function parseVisual(visualEl: Element, materialLibrary: Map<string, UrdfMaterial>): UrdfVisual {
  const name = visualEl.getAttribute('name') || undefined;
  const originEl = visualEl.querySelector('origin');
  const geomEl = visualEl.querySelector('geometry');
  const matEl = visualEl.querySelector('material');

  const geometry = parseGeometry(geomEl);
  if (!geometry) {
    throw new Error('Visual element missing valid geometry');
  }

  return {
    name,
    origin: parseOrigin(originEl),
    geometry,
    material: resolveMaterial(parseMaterial(matEl), materialLibrary),
  };
}

/**
 * Parse <link> element
 */
function parseLink(linkEl: Element, materialLibrary: Map<string, UrdfMaterial>): UrdfLink {
  const name = linkEl.getAttribute('name') || '';

  const visuals: UrdfVisual[] = [];
  for (const visualEl of linkEl.querySelectorAll(':scope > visual')) {
    visuals.push(parseVisual(visualEl, materialLibrary));
  }

  const collisions: UrdfVisual[] = [];
  for (const collisionEl of linkEl.querySelectorAll(':scope > collision')) {
    collisions.push(parseVisual(collisionEl, materialLibrary));
  }

  return { name, visuals, collisions };
}

/**
 * Parse <joint> element
 */
function parseJoint(jointEl: Element): UrdfJoint {
  const name = jointEl.getAttribute('name') || '';
  const type = (jointEl.getAttribute('type') || 'fixed') as UrdfJoint['type'];

  const parentEl = jointEl.querySelector('parent');
  const childEl = jointEl.querySelector('child');

  const parent = parentEl?.getAttribute('link') || '';
  const child = childEl?.getAttribute('link') || '';

  const originEl = jointEl.querySelector('origin');
  const axisEl = jointEl.querySelector('axis');
  const limitEl = jointEl.querySelector('limit');

  const joint: UrdfJoint = {
    name,
    type,
    parent,
    child,
  };

  if (originEl) {
    joint.origin = parseOrigin(originEl);
  }

  if (axisEl) {
    const xyzStr = axisEl.getAttribute('xyz') ?? undefined;
    joint.axis = parseVec3(xyzStr);
  }

  if (limitEl) {
    joint.limit = {
      lower: parseFloat(limitEl.getAttribute('lower') || '0'),
      upper: parseFloat(limitEl.getAttribute('upper') || '0'),
      effort: parseFloat(limitEl.getAttribute('effort') || '0'),
      velocity: parseFloat(limitEl.getAttribute('velocity') || '0'),
    };
  }

  return joint;
}

/**
 * Rewrite mesh filenames from absolute package:// paths to relative paths
 * e.g., "package://robot/meshes/link.dae" -> "meshes/link.dae"
 */
export function rewriteAbsoluteMeshFilenames(urdfSpec: UrdfSpec): UrdfSpec {
  const rewrite = (filename: string): string => {
    if (filename.startsWith('package://')) {
      // Extract everything after the first package name
      const match = filename.match(/^package:\/\/[^/]+\/(.+)$/);
      return match ? match[1] : filename;
    }
    return filename;
  };

  const rewriteVisuals = (visuals: UrdfVisual[]): UrdfVisual[] => {
    return visuals.map(v => {
      if (v.geometry.type === 'mesh' && v.geometry.filename) {
        return {
          ...v,
          geometry: {
            ...v.geometry,
            filename: rewrite(v.geometry.filename),
          },
        };
      }
      return v;
    });
  };

  return {
    ...urdfSpec,
    links: urdfSpec.links.map(link => ({
      ...link,
      visuals: rewriteVisuals(link.visuals),
      collisions: rewriteVisuals(link.collisions),
    })),
  };
}

/**
 * Find the root link (link that is not a child of any joint)
 */
export function findRootLink(urdfSpec: UrdfSpec): string | null {
  const childLinks = new Set(urdfSpec.joints.map(j => j.child));
  const rootLink = urdfSpec.links.find(link => !childLinks.has(link.name));
  return rootLink?.name || null;
}

/**
 * Parse URDF XML string into structured spec
 */
export function parseUrdf(urdfXml: string): UrdfSpec {
  const parser = new DOMParser();
  const doc = parser.parseFromString(urdfXml, 'application/xml');

  const robotEl = doc.querySelector('robot');
  if (!robotEl) {
    throw new Error('URDF missing <robot> root element');
  }

  const name = robotEl.getAttribute('name') || 'robot';
  const materialLibrary = new Map<string, UrdfMaterial>();

  for (const materialEl of robotEl.querySelectorAll(':scope > material')) {
    const material = parseMaterial(materialEl);
    if (!material?.name) {
      continue;
    }
    materialLibrary.set(material.name, material);
  }

  const links: UrdfLink[] = [];
  for (const linkEl of robotEl.querySelectorAll(':scope > link')) {
    links.push(parseLink(linkEl, materialLibrary));
  }

  const joints: UrdfJoint[] = [];
  for (const jointEl of robotEl.querySelectorAll(':scope > joint')) {
    joints.push(parseJoint(jointEl));
  }

  return { name, links, joints };
}
