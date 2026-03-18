import * as THREE from "three";

import type { UrdfJoint, UrdfSpec } from "./urdf-parser";

/* ── Palette ── */

const JOINT_COLORS: Record<string, number> = {
  revolute: 0x5b9cf5,
  continuous: 0x3ec9a8,
  prismatic: 0xf0a030,
  fixed: 0x8090a0,
  planar: 0xa87fd4,
  floating: 0xc06020,
};

const TYPE_ABBREV: Record<string, string> = {
  revolute: "REV",
  continuous: "CONT",
  prismatic: "PRI",
};

/* ── Helpers ── */

function isOverlayJoint(joint: UrdfJoint): boolean {
  return joint.type === "revolute" || joint.type === "continuous" || joint.type === "prismatic";
}

function compactAxis(raw?: [number, number, number]): string {
  const [x, y, z] = raw ?? [0, 0, 1];
  const e = 0.01;
  if (Math.abs(x - 1) < e && Math.abs(y) < e && Math.abs(z) < e) return "X";
  if (Math.abs(x + 1) < e && Math.abs(y) < e && Math.abs(z) < e) return "\u2212X";
  if (Math.abs(x) < e && Math.abs(y - 1) < e && Math.abs(z) < e) return "Y";
  if (Math.abs(x) < e && Math.abs(y + 1) < e && Math.abs(z) < e) return "\u2212Y";
  if (Math.abs(x) < e && Math.abs(y) < e && Math.abs(z - 1) < e) return "Z";
  if (Math.abs(x) < e && Math.abs(y) < e && Math.abs(z + 1) < e) return "\u2212Z";
  return [x, y, z].map((v) => v.toFixed(1)).join(" ");
}

/* ── Screen-fixed dark-glass pill label ── */

const LABEL_SCREEN_PX = 22;

function createLabel(type: string, axisText: string, tint: number): THREE.Sprite {
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("Failed to create canvas 2D context for joint overlay label.");

  const dpr = 2;
  const fs = 11 * dpr;
  const px = 8 * dpr;
  const py = 5 * dpr;
  const dotR = 3 * dpr;
  const dotGap = 5 * dpr;
  const wordGap = 5 * dpr;

  ctx.font = `600 ${fs}px "DM Sans", system-ui, sans-serif`;
  const tw = ctx.measureText(type).width;
  ctx.font = `400 ${fs}px "DM Sans", system-ui, sans-serif`;
  const aw = ctx.measureText(axisText).width;

  const w = Math.ceil(px + dotR * 2 + dotGap + tw + wordGap + aw + px);
  const h = Math.ceil(py + fs + py);
  canvas.width = w;
  canvas.height = h;

  ctx.fillStyle = "rgba(8, 8, 8, 0.74)";
  pillPath(ctx, 0, 0, w, h);
  ctx.fill();

  ctx.strokeStyle = "rgba(255, 255, 255, 0.07)";
  ctx.lineWidth = dpr;
  pillPath(ctx, dpr * 0.5, dpr * 0.5, w - dpr, h - dpr);
  ctx.stroke();

  let cx = px;

  ctx.beginPath();
  ctx.arc(cx + dotR, h / 2, dotR, 0, Math.PI * 2);
  ctx.fillStyle = `#${tint.toString(16).padStart(6, "0")}`;
  ctx.fill();
  cx += dotR * 2 + dotGap;

  ctx.font = `600 ${fs}px "DM Sans", system-ui, sans-serif`;
  ctx.textBaseline = "middle";
  ctx.fillStyle = "rgba(255, 255, 255, 0.94)";
  ctx.fillText(type, cx, h / 2 + dpr * 0.5);
  cx += tw + wordGap;

  ctx.font = `400 ${fs}px "DM Sans", system-ui, sans-serif`;
  ctx.fillStyle = "rgba(255, 255, 255, 0.44)";
  ctx.fillText(axisText, cx, h / 2 + dpr * 0.5);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.needsUpdate = true;

  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
  });

  const sprite = new THREE.Sprite(material);
  sprite.renderOrder = 40;
  sprite.userData.articraftOverlay = true;
  sprite.userData.articraftOverlayTexture = texture;

  const aspect = w / h;
  const wp = new THREE.Vector3();
  const cp = new THREE.Vector3();

  sprite.onBeforeRender = (renderer, _scene, camera) => {
    if (!(camera instanceof THREE.PerspectiveCamera)) return;
    const screenH = renderer.domElement.clientHeight;
    if (screenH === 0) return;

    sprite.getWorldPosition(wp);
    camera.getWorldPosition(cp);
    const dist = wp.distanceTo(cp);

    const worldPerPx = (2 * dist * Math.tan(camera.fov * THREE.MathUtils.DEG2RAD * 0.5)) / screenH;
    const sh = LABEL_SCREEN_PX * worldPerPx;
    sprite.scale.set(sh * aspect, sh, 1);
    sprite.updateMatrixWorld(true);
  };

  return sprite;
}

function pillPath(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number): void {
  const r = h / 2;
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.lineTo(x + w - r, y);
  ctx.arc(x + w - r, y + r, r, -Math.PI / 2, Math.PI / 2);
  ctx.lineTo(x + r, y + h);
  ctx.arc(x + r, y + r, r, Math.PI / 2, -Math.PI / 2);
  ctx.closePath();
}

/* ── Overlay material ── */

function overlayMat(color: number, opacity: number): THREE.MeshBasicMaterial {
  return new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    depthTest: false,
    depthWrite: false,
    toneMapped: false,
  });
}

/* ── 3-D geometry builders (real meshes, not hairline Lines) ── */

const _zUp = new THREE.Vector3(0, 0, 1);
const _yUp = new THREE.Vector3(0, 1, 0);

/** Axis line as a cylinder so it has visible thickness. */
function buildAxisLine(
  negLen: number,
  posLen: number,
  thickness: number,
  color: number,
  axis: THREE.Vector3,
): THREE.Mesh {
  const totalLen = negLen + posLen;
  const geo = new THREE.CylinderGeometry(thickness, thickness, totalLen, 8);
  const mesh = new THREE.Mesh(geo, overlayMat(color, 0.5));
  // CylinderGeometry is along Y; orient to joint axis then shift to centre
  mesh.quaternion.setFromUnitVectors(_yUp, axis);
  mesh.position.copy(axis.clone().multiplyScalar((posLen - negLen) / 2));
  mesh.renderOrder = 30;
  mesh.userData.articraftOverlay = true;
  return mesh;
}

function buildDot(radius: number, color: number): THREE.Mesh {
  const geo = new THREE.SphereGeometry(radius, 12, 12);
  const mesh = new THREE.Mesh(geo, overlayMat(color, 1));
  mesh.renderOrder = 32;
  mesh.userData.articraftOverlay = true;
  return mesh;
}

/**
 * ~270-degree arc tube with a cone arrowhead at its tip.
 * Built in the XY plane (rotation around Z); caller orients to the joint axis.
 */
function buildArcArrow(
  arcRadius: number,
  tubeR: number,
  headLen: number,
  headWidth: number,
  color: number,
): THREE.Group {
  const group = new THREE.Group();
  group.userData.articraftOverlay = true;

  const sweep = Math.PI * 1.5;
  const start = Math.PI * 0.25;
  const segments = 48;

  // Generate arc points in XY plane
  const pts: THREE.Vector3[] = [];
  for (let i = 0; i <= segments; i++) {
    const a = start + (i / segments) * sweep;
    pts.push(new THREE.Vector3(arcRadius * Math.cos(a), arcRadius * Math.sin(a), 0));
  }

  // Tube geometry along the arc curve
  const curve = new THREE.CatmullRomCurve3(pts, false, "catmullrom", 0);
  const tubeGeo = new THREE.TubeGeometry(curve, segments, tubeR, 8, false);
  const tubeMesh = new THREE.Mesh(tubeGeo, overlayMat(color, 0.6));
  tubeMesh.renderOrder = 31;
  tubeMesh.userData.articraftOverlay = true;
  group.add(tubeMesh);

  // Cone arrowhead at the end of the arc, pointing along the tangent
  const endAngle = start + sweep;
  const endPos = new THREE.Vector3(arcRadius * Math.cos(endAngle), arcRadius * Math.sin(endAngle), 0);
  const tangent = new THREE.Vector3(-Math.sin(endAngle), Math.cos(endAngle), 0).normalize();

  const coneGeo = new THREE.ConeGeometry(headWidth, headLen, 8);
  const cone = new THREE.Mesh(coneGeo, overlayMat(color, 0.7));
  cone.position.copy(endPos);
  cone.quaternion.setFromUnitVectors(_yUp, tangent);
  cone.renderOrder = 31;
  cone.userData.articraftOverlay = true;
  group.add(cone);

  return group;
}

/** Cone arrowhead pointing along a given direction at a given position. */
function buildConeHead(
  headLen: number,
  headWidth: number,
  color: number,
  direction: THREE.Vector3,
  position: THREE.Vector3,
): THREE.Mesh {
  const geo = new THREE.ConeGeometry(headWidth, headLen, 8);
  const cone = new THREE.Mesh(geo, overlayMat(color, 0.7));
  cone.position.copy(position);
  cone.quaternion.setFromUnitVectors(_yUp, direction);
  cone.renderOrder = 31;
  cone.userData.articraftOverlay = true;
  return cone;
}

/* ── Public API ── */

export function attachJointOverlay(
  urdfSpec: UrdfSpec,
  jointFrames: Map<string, THREE.Group>,
  modelExtent: number,
): THREE.Object3D[] {
  const attached: THREE.Object3D[] = [];
  const unit = THREE.MathUtils.clamp(modelExtent * 0.18, 0.14, 0.6);

  const lineHalf = unit * 0.95;
  const lineThick = unit * 0.016;
  const dotR = unit * 0.04;
  const arcR = unit * 0.16;
  const arcTube = unit * 0.013;
  const arcHeadLen = arcR * 0.55;
  const arcHeadW = arcR * 0.35;
  const coneH = unit * 0.15;
  const coneW = unit * 0.065;
  const labelOffset = unit * 0.24;

  for (const joint of urdfSpec.joints) {
    if (!isOverlayJoint(joint)) continue;

    const frame = jointFrames.get(joint.name);
    if (!frame) continue;

    const axis = new THREE.Vector3(...(joint.axis ?? [0, 0, 1]));
    if (axis.lengthSq() === 0) axis.set(0, 0, 1);
    axis.normalize();

    const color = JOINT_COLORS[joint.type] ?? 0x4b5563;

    const group = new THREE.Group();
    group.name = `joint-overlay:${joint.name}`;
    group.renderOrder = 30;
    group.userData.articraftOverlay = true;

    // Thick axis line (symmetric about joint origin)
    group.add(buildAxisLine(lineHalf, lineHalf, lineThick, color, axis));

    // Origin dot
    group.add(buildDot(dotR, color));

    if (joint.type === "revolute" || joint.type === "continuous") {
      // Attach arc arrows to the motion group so they spin with the joint
      const motionGroup = frame.getObjectByName(`joint-motion:${joint.name}`);
      const arcParent = motionGroup ?? group;

      const arcTop = buildArcArrow(arcR, arcTube, arcHeadLen, arcHeadW, color);
      arcTop.quaternion.setFromUnitVectors(_zUp, axis);
      arcTop.position.copy(axis.clone().multiplyScalar(lineHalf * 0.82));
      arcParent.add(arcTop);

      const arcBot = buildArcArrow(arcR, arcTube, arcHeadLen, arcHeadW, color);
      arcBot.quaternion.setFromUnitVectors(_zUp, axis);
      arcBot.position.copy(axis.clone().multiplyScalar(-lineHalf * 0.82));
      arcParent.add(arcBot);

      // Track separately for disposal since they live on a different parent
      attached.push(arcTop);
      attached.push(arcBot);
    } else if (joint.type === "prismatic") {
      // Double-ended arrow cones
      const posEnd = axis.clone().multiplyScalar(lineHalf);
      const negEnd = axis.clone().multiplyScalar(-lineHalf);
      group.add(buildConeHead(coneH, coneW, color, axis, posEnd));
      group.add(buildConeHead(coneH, coneW, color, axis.clone().negate(), negEnd));
    }

    // Screen-fixed label, offset perpendicular to the axis
    const abbrev = TYPE_ABBREV[joint.type] ?? joint.type.toUpperCase();
    const label = createLabel(abbrev, compactAxis(joint.axis), color);
    const perp = new THREE.Vector3(1, 0, 0);
    if (Math.abs(axis.dot(perp)) > 0.9) perp.set(0, 1, 0);
    perp.crossVectors(perp, axis).normalize();
    label.position.copy(perp.multiplyScalar(labelOffset));
    group.add(label);

    frame.add(group);
    attached.push(group);
  }

  return attached;
}

export function disposeOverlayObjects(objects: THREE.Object3D[]): void {
  for (const obj of objects) {
    obj.parent?.remove(obj);
    obj.traverse((child) => {
      if (child instanceof THREE.Sprite) {
        child.onBeforeRender = () => {};
      }
      if ("geometry" in child) {
        const geometry = (child as { geometry?: THREE.BufferGeometry }).geometry;
        geometry?.dispose();
      }
      if ("material" in child) {
        const material = (child as { material?: THREE.Material | THREE.Material[] }).material;
        if (Array.isArray(material)) {
          material.forEach((entry) => disposeMaterial(entry));
        } else if (material) {
          disposeMaterial(material);
        }
      }
    });
  }
}

function disposeMaterial(material: THREE.Material): void {
  if (material instanceof THREE.SpriteMaterial) {
    material.map?.dispose();
  }
  material.dispose();
}
