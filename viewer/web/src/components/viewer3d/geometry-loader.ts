import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import type { UrdfVisualGeometry } from './urdf-parser';
import type { MaterialSpec } from './materials';
import { createMaterial, createEdgeLines } from './materials';

export interface LoadGeometryOptions {
  kind?: 'visual' | 'collision';
  materialSpec?: MaterialSpec;
  doubleSided?: boolean;
}

/**
 * Build a Three.js mesh from a URDF primitive geometry spec (box, cylinder, sphere).
 * Returns null for mesh-type geometries (use loadGeometryObject instead).
 */
export function buildPrimitiveMesh(
  geometry: UrdfVisualGeometry,
  materialSpec: MaterialSpec,
): THREE.Mesh | null {
  let bufferGeometry: THREE.BufferGeometry;

  switch (geometry.type) {
    case 'box': {
      const [x, y, z] = geometry.size ?? [1, 1, 1];
      bufferGeometry = new THREE.BoxGeometry(x, y, z);
      break;
    }
    case 'cylinder': {
      const radius = geometry.radius ?? 1;
      const length = geometry.length ?? 1;
      bufferGeometry = new THREE.CylinderGeometry(radius, radius, length, 32);
      // URDF cylinders are aligned to the Z axis; Three.js uses Y.
      bufferGeometry.rotateX(Math.PI / 2);
      break;
    }
    case 'sphere': {
      const radius = geometry.radius ?? 1;
      bufferGeometry = new THREE.SphereGeometry(radius, 32, 16);
      break;
    }
    default:
      return null;
  }

  const material = createMaterial(materialSpec);
  const mesh = new THREE.Mesh(bufferGeometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;

  return mesh;
}

/**
 * Load an external mesh file (GLB/GLTF or OBJ) referenced by a URDF mesh geometry.
 * Returns a promise that resolves to a THREE.Group containing the loaded model.
 */
export async function loadGeometryObject(
  geometry: UrdfVisualGeometry,
  baseUrl: string,
  options: LoadGeometryOptions = {},
): Promise<THREE.Group> {
  if (geometry.type !== 'mesh' || !geometry.filename) {
    throw new Error(`loadGeometryObject requires a mesh geometry with a filename`);
  }

  const filename = geometry.filename;
  const url = baseUrl.endsWith('/')
    ? `${baseUrl}${filename}`
    : `${baseUrl}/${filename}`;

  const extension = filename.split('.').pop()?.toLowerCase();
  const scale = geometry.scale ?? [1, 1, 1];

  if (extension === 'glb' || extension === 'gltf') {
    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync(url);
    const group = new THREE.Group();
    group.add(gltf.scene);
    group.scale.set(scale[0], scale[1], scale[2]);
    applyLoadedMeshPresentation(group, options);
    return group;
  }

  if (extension === 'obj') {
    const loader = new OBJLoader();
    const obj = await loader.loadAsync(url);
    const group = new THREE.Group();
    group.add(obj);
    group.scale.set(scale[0], scale[1], scale[2]);
    applyLoadedMeshPresentation(group, options);
    return group;
  }

  throw new Error(`Unsupported mesh file format: .${extension} (${filename})`);
}

function applyLoadedMeshPresentation(root: THREE.Object3D, options: LoadGeometryOptions): void {
  const kind = options.kind ?? 'visual';
  const materialSpec = options.materialSpec;
  const side = options.doubleSided ? THREE.DoubleSide : THREE.FrontSide;

  root.traverse((child) => {
    if (!(child instanceof THREE.Mesh)) {
      return;
    }

    if (!child.geometry.attributes.normal) {
      child.geometry.computeVertexNormals();
    }

    if (materialSpec) {
      child.material = createMaterial(materialSpec, {
        side,
        transparent: kind === 'collision',
      });
    }

    child.castShadow = kind === 'visual';
    child.receiveShadow = kind === 'visual';
    child.userData.articraftVisual = kind === 'visual';
    child.userData.articraftCollision = kind === 'collision';
    child.visible = kind === 'visual';

    if (kind === 'collision') {
      child.renderOrder = 10;
      child.add(createEdgeLines(child.geometry, materialSpec?.color.getHex() ?? 0x000000));
    }
  });
}

/**
 * Add wireframe edge lines as a child overlay on a mesh.
 * Iterates through the object and its children, adding edge lines to any Mesh found.
 */
export function addEdgeLines(mesh: THREE.Object3D): void {
  const meshes: THREE.Mesh[] = [];
  mesh.traverse((child) => {
    if (child instanceof THREE.Mesh && child.geometry) {
      meshes.push(child);
    }
  });

  for (const m of meshes) {
    const edges = createEdgeLines(m.geometry);
    m.add(edges);
  }
}
