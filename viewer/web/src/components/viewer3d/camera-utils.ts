import * as THREE from 'three';

export interface FitResult {
  position: THREE.Vector3;
  target: THREE.Vector3;
  near: number;
  far: number;
}

const MIN_NEAR = 0.001;
const MIN_FAR_GAP = 1;
const DEFAULT_CLIP_FAR = 1000;

function getBoundingBox(
  object: THREE.Object3D | readonly THREE.Object3D[],
): THREE.Box3 | null {
  const objects = Array.isArray(object) ? object : [object];
  const box = new THREE.Box3();

  for (const entry of objects) {
    if (!entry.visible) {
      continue;
    }
    box.expandByObject(entry);
  }

  return box.isEmpty() ? null : box;
}

function computeClipPlanesForBox(
  camera: THREE.PerspectiveCamera,
  box: THREE.Box3,
): { near: number; far: number } {
  camera.updateMatrixWorld(true);

  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const centerDepth = -sphere.center.clone().applyMatrix4(camera.matrixWorldInverse).z;
  const nearPadding = Math.max(sphere.radius * 0.1, 0.02);
  const farPadding = Math.max(sphere.radius * 0.5, 0.5);

  // A bounding sphere produces stable clip planes even when the model bounds
  // straddle the camera plane during pans or close orbit moves.
  const near = Math.max(MIN_NEAR, centerDepth - sphere.radius - nearPadding);
  const far = Math.max(near + MIN_FAR_GAP, centerDepth + sphere.radius + farPadding);
  return { near, far };
}

/**
 * Compute a camera position that fits the given object within the camera's field of view.
 * Returns the ideal position, target (center of bounding box), and clipping planes.
 */
export function computeFit(
  camera: THREE.PerspectiveCamera,
  object: THREE.Object3D,
): FitResult {
  const box = getBoundingBox(object);
  if (!box) {
    return {
      position: camera.position.clone(),
      target: new THREE.Vector3(),
      near: camera.near,
      far: camera.far || DEFAULT_CLIP_FAR,
    };
  }

  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());

  const maxDim = Math.max(size.x, size.y, size.z);
  const fov = camera.fov * (Math.PI / 180);
  // Distance needed so the object fits in view, with a small padding factor
  const distance = (maxDim / 2) / Math.tan(fov / 2) * 1.5;

  // Position camera along its current direction from center, or default diagonal
  const direction = new THREE.Vector3();
  if (camera.position.lengthSq() > 0) {
    direction.copy(camera.position).sub(center).normalize();
  }
  if (direction.lengthSq() < 0.001) {
    direction.set(1, 0.8, 1).normalize();
  }

  const position = center.clone().add(direction.multiplyScalar(distance));
  const fitCamera = camera.clone();
  fitCamera.position.copy(position);
  fitCamera.lookAt(center);
  const { near, far } = computeClipPlanesForBox(fitCamera, box);

  return { position, target: center, near, far };
}

/**
 * Preserve the camera's current viewing angle and relative distance when switching
 * to a new model. Adjusts position so the new object is centered and fits in view.
 */
export function preserveViewAcrossModelSwitch(
  camera: THREE.PerspectiveCamera,
  controls: { target: THREE.Vector3; update: () => void },
  newObject: THREE.Object3D,
): void {
  const oldTarget = controls.target.clone();
  const oldOffset = camera.position.clone().sub(oldTarget);
  const oldDistance = oldOffset.length();
  const direction = oldDistance > 0.001
    ? oldOffset.normalize()
    : new THREE.Vector3(1, 0.8, 1).normalize();

  const box = new THREE.Box3().setFromObject(newObject);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());

  const maxDim = Math.max(size.x, size.y, size.z);
  const fov = camera.fov * (Math.PI / 180);
  const fitDistance = (maxDim / 2) / Math.tan(fov / 2) * 1.5;

  // Use the old viewing direction but scale to fit the new object
  const newPosition = center.clone().add(direction.multiplyScalar(fitDistance));

  camera.position.copy(newPosition);
  controls.target.copy(center);
  controls.update();

  updateCameraClipping(camera, newObject);
}

/**
 * Update the camera's near and far clipping planes based on the object's bounding box size.
 * Ensures the full object is visible without z-fighting artifacts.
 */
export function updateCameraClipping(
  camera: THREE.PerspectiveCamera,
  object: THREE.Object3D | readonly THREE.Object3D[],
): void {
  const box = getBoundingBox(object);
  if (!box) {
    return;
  }

  const { near, far } = computeClipPlanesForBox(camera, box);
  camera.near = near;
  camera.far = far;
  camera.updateProjectionMatrix();
}
