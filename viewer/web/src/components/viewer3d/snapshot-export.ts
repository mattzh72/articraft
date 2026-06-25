import * as THREE from 'three';

export function makeSnapshotFileName(selectionKey: string | null): string {
  const baseName = selectionKey?.replace(/[^a-z0-9_-]+/gi, "-").replace(/^-+|-+$/g, "") || "object";
  const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
  return `articraft-${baseName}-${timestamp}.png`;
}

export function canvasToBlob(canvas: HTMLCanvasElement): Promise<Blob> {
  return new Promise((resolve, reject) => {
    canvas.toBlob((blob) => {
      if (!blob) {
        reject(new Error("Snapshot export produced an empty image."));
        return;
      }
      resolve(blob);
    }, "image/png");
  });
}

export function downloadBlob(blob: Blob, fileName: string): void {
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = fileName;
  document.body.appendChild(link);
  link.click();
  link.remove();
  window.setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function setVisibleTemporarily(
  objects: Array<THREE.Object3D | null | undefined>,
  visible: boolean,
): () => void {
  const previousVisibility = new Map<THREE.Object3D, boolean>();

  for (const object of objects) {
    if (!object || previousVisibility.has(object)) {
      continue;
    }
    previousVisibility.set(object, object.visible);
    object.visible = visible;
  }

  return () => {
    for (const [object, wasVisible] of previousVisibility) {
      object.visible = wasVisible;
    }
  };
}
