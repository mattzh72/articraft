import * as THREE from 'three';
import { USDLoader } from 'three/addons/loaders/USDLoader.js';

import { buildUrdfVisualKey, computeLinkWorldMatrices, type UrdfSpec } from './urdf-parser';

const usdzTemplateCache = new Map<string, Promise<THREE.Group>>();

function appendRevisionParam(url: string, assetRevisionKey: string | null | undefined): string {
  if (!assetRevisionKey) {
    return url;
  }

  const separator = url.includes('?') ? '&' : '?';
  return `${url}${separator}rev=${encodeURIComponent(assetRevisionKey)}`;
}

function cloneUsdTree(root: THREE.Object3D): THREE.Object3D {
  const clone = root.clone(true);

  clone.traverse((child) => {
    if (!(child instanceof THREE.Mesh)) {
      return;
    }

    const material = child.material;
    if (Array.isArray(material)) {
      child.material = material.map((entry) => entry.clone());
    } else if (material) {
      child.material = material.clone();
    }
  });

  return clone;
}

function hasRenderableMesh(root: THREE.Object3D): boolean {
  let found = false;
  root.traverse((child) => {
    if (child instanceof THREE.Mesh) {
      found = true;
    }
  });
  return found;
}

function matchUsdNodeToLinkName(name: string, linkNames: readonly string[]): string | null {
  const normalized = name.trim().toLowerCase();
  let bestMatch: string | null = null;

  for (const linkName of linkNames) {
    const normalizedLink = linkName.toLowerCase();
    if (normalized === normalizedLink || normalized.startsWith(`${normalizedLink}_`)) {
      if (bestMatch == null || linkName.length > bestMatch.length) {
        bestMatch = linkName;
      }
    }
  }

  return bestMatch;
}

function markUsdVisualTree(
  root: THREE.Object3D,
  {
    linkName,
    visualKey,
  }: {
    linkName: string;
    visualKey: string;
  },
): void {
  root.userData.articraftLinkName = linkName;
  root.userData.articraftVisualKey = visualKey;
  root.userData.articraftVisualLabel = root.name || linkName;

  root.traverse((child) => {
    if (!(child instanceof THREE.Mesh)) {
      return;
    }

    child.castShadow = true;
    child.receiveShadow = true;
    child.visible = true;
    child.userData.articraftVisual = true;
    child.userData.articraftLinkName = linkName;
    child.userData.articraftVisualKey = visualKey;
    child.userData.articraftVisualLabel = root.name || linkName;

    if (!child.geometry.attributes.normal) {
      child.geometry.computeVertexNormals();
    }
  });
}

async function loadUsdTemplate(resolvedUrl: string): Promise<THREE.Group> {
  const cached = usdzTemplateCache.get(resolvedUrl);
  if (cached) {
    return cached;
  }

  const pending = (async () => {
    const loader = new USDLoader();
    return loader.loadAsync(resolvedUrl);
  })();

  usdzTemplateCache.set(resolvedUrl, pending);
  void pending.catch(() => {
    if (usdzTemplateCache.get(resolvedUrl) === pending) {
      usdzTemplateCache.delete(resolvedUrl);
    }
  });

  return pending;
}

function findUsdContentRoot(group: THREE.Group): THREE.Object3D {
  if (group.children.length === 1 && group.children[0] && group.children[0].children.length > 0) {
    return group.children[0];
  }
  return group;
}

export async function attachUsdLinkVisuals(
  spec: UrdfSpec,
  linkNodes: Map<string, THREE.Group>,
  {
    usdzUrl,
    assetRevisionKey,
  }: {
    usdzUrl: string;
    assetRevisionKey: string | null;
  },
): Promise<Set<string>> {
  const resolvedUrl = appendRevisionParam(usdzUrl, assetRevisionKey);
  const template = await loadUsdTemplate(resolvedUrl);
  const clonedStage = cloneUsdTree(template) as THREE.Group;
  clonedStage.updateMatrixWorld(true);

  const contentRoot = findUsdContentRoot(clonedStage);
  const linkWorldMatrices = computeLinkWorldMatrices(spec);
  const linkNames = spec.links.map((link) => link.name);
  const linkVisualCounts = new Map<string, number>();
  const matchedLinks = new Set<string>();

  for (const child of contentRoot.children) {
    if (!hasRenderableMesh(child)) {
      continue;
    }

    const linkName = matchUsdNodeToLinkName(child.name, linkNames);
    if (!linkName) {
      continue;
    }

    const linkGroup = linkNodes.get(linkName);
    const linkWorld = linkWorldMatrices.get(linkName);
    if (!linkGroup || !linkWorld) {
      continue;
    }

    if (!matchedLinks.has(linkName)) {
      for (const existingChild of [...linkGroup.children]) {
        if (existingChild.userData.articraftVisual === true) {
          linkGroup.remove(existingChild);
        }
      }
      matchedLinks.add(linkName);
    }

    const clone = cloneUsdTree(child);
    const sourceWorld = child.matrixWorld.clone();
    // USDLoader already converts Z-up USD stages into Three.js Y-up coordinates
    // on the loaded root group, so we attach directly in URDF link space here.
    const localMatrix = linkWorld.clone().invert().multiply(sourceWorld);

    clone.matrixAutoUpdate = false;
    clone.matrix.copy(localMatrix);
    clone.matrix.decompose(clone.position, clone.quaternion, clone.scale);

    const visualIndex = linkVisualCounts.get(linkName) ?? 0;
    linkVisualCounts.set(linkName, visualIndex + 1);
    markUsdVisualTree(clone, {
      linkName,
      visualKey: buildUrdfVisualKey(linkName, visualIndex),
    });

    if (!clone.name) {
      clone.name = `usd:${linkName}:${visualIndex}`;
    }
    linkGroup.add(clone);
  }

  return matchedLinks;
}
