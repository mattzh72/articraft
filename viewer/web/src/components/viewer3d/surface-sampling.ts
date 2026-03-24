import * as THREE from 'three';
import { MeshSurfaceSampler } from 'three/addons/math/MeshSurfaceSampler.js';

const MIN_SAMPLE_COUNT = 1800;
const MAX_SAMPLE_COUNT = 24000;
const SAMPLE_DENSITY = 2600;
const MIN_POINT_SIZE = 0.0016;
const MAX_POINT_SIZE = 0.007;
const POINT_SIZE_RATIO = 0.012;
const NORMAL_OFFSET_RATIO = 0.0025;

export function createSurfaceSamplePoints(
  mesh: THREE.Mesh,
  color: THREE.Color,
  linkName: string,
): THREE.Points | null {
  const geometry = mesh.geometry;
  const positionsAttribute = geometry.getAttribute('position');

  if (!positionsAttribute || positionsAttribute.count === 0) {
    return null;
  }

  const sampleCount = estimateSampleCount(mesh);
  const pointSize = estimatePointSize(mesh);
  const normalOffset = estimateNormalOffset(mesh, pointSize);
  const sampler = new MeshSurfaceSampler(mesh).build();
  const positions = new Float32Array(sampleCount * 3);
  const colors = new Float32Array(sampleCount * 3);
  const position = new THREE.Vector3();
  const normal = new THREE.Vector3();

  for (let index = 0; index < sampleCount; index += 1) {
    sampler.sample(position, normal);

    if (normal.lengthSq() > 1e-8) {
      position.addScaledVector(normal.normalize(), normalOffset);
    }

    const offset = index * 3;
    positions[offset] = position.x;
    positions[offset + 1] = position.y;
    positions[offset + 2] = position.z;
    colors[offset] = color.r;
    colors[offset + 1] = color.g;
    colors[offset + 2] = color.b;
  }

  const pointGeometry = new THREE.BufferGeometry();
  pointGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  pointGeometry.computeBoundingSphere();

  const pointMaterial = new THREE.PointsMaterial({
    size: pointSize,
    sizeAttenuation: true,
    vertexColors: true,
    transparent: true,
    opacity: 0.96,
    depthWrite: false,
  });
  pointMaterial.toneMapped = false;

  const points = new THREE.Points(pointGeometry, pointMaterial);
  points.renderOrder = 25;
  points.userData.articraftSurfaceSample = true;
  points.userData.articraftLinkName = linkName;

  return points;
}

export function disposeSurfaceSamplePoints(pointsClouds: THREE.Points[]): void {
  for (const points of pointsClouds) {
    points.parent?.remove(points);
    points.geometry.dispose();

    if (Array.isArray(points.material)) {
      for (const material of points.material) {
        material.dispose();
      }
    } else {
      points.material.dispose();
    }
  }
}

function estimateSampleCount(mesh: THREE.Mesh): number {
  const surfaceArea = estimateSurfaceArea(mesh);
  return Math.round(
    THREE.MathUtils.clamp(surfaceArea * SAMPLE_DENSITY, MIN_SAMPLE_COUNT, MAX_SAMPLE_COUNT),
  );
}

function estimateSurfaceArea(mesh: THREE.Mesh): number {
  const geometry = mesh.geometry;

  if (!geometry.boundingBox) {
    geometry.computeBoundingBox();
  }

  const bounds = geometry.boundingBox;
  if (!bounds) {
    return MIN_SAMPLE_COUNT / SAMPLE_DENSITY;
  }

  mesh.updateWorldMatrix(true, false);
  const scale = mesh.getWorldScale(new THREE.Vector3());
  const size = bounds.getSize(new THREE.Vector3());
  const x = Math.abs(size.x * scale.x);
  const y = Math.abs(size.y * scale.y);
  const z = Math.abs(size.z * scale.z);

  return Math.max(2 * ((x * y) + (y * z) + (z * x)), MIN_SAMPLE_COUNT / SAMPLE_DENSITY);
}

function estimatePointSize(mesh: THREE.Mesh): number {
  const geometry = mesh.geometry;

  if (!geometry.boundingSphere) {
    geometry.computeBoundingSphere();
  }

  const radius = geometry.boundingSphere?.radius ?? 0.1;
  mesh.updateWorldMatrix(true, false);
  const scale = mesh.getWorldScale(new THREE.Vector3());
  const scaleFactor = Math.max(Math.abs(scale.x), Math.abs(scale.y), Math.abs(scale.z), 1e-3);

  return THREE.MathUtils.clamp(radius * scaleFactor * POINT_SIZE_RATIO, MIN_POINT_SIZE, MAX_POINT_SIZE);
}

function estimateNormalOffset(mesh: THREE.Mesh, pointSize: number): number {
  const geometry = mesh.geometry;

  if (!geometry.boundingSphere) {
    geometry.computeBoundingSphere();
  }

  const radius = geometry.boundingSphere?.radius ?? 0.1;
  return THREE.MathUtils.clamp(radius * NORMAL_OFFSET_RATIO, pointSize * 0.3, 0.012);
}
