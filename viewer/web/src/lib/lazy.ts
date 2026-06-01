import { lazy, type ComponentType, type LazyExoticComponent } from "react";

const CHUNK_RELOAD_KEY_PREFIX = "articraft:chunk-reload";

function isChunkLoadError(error: unknown): error is Error {
  if (!(error instanceof Error)) {
    return false;
  }

  const message = error.message;
  return (
    message.includes("Failed to fetch dynamically imported module")
    || message.includes("error loading dynamically imported module")
    || message.includes("Importing a module script failed")
    || message.includes("ChunkLoadError")
  );
}

function reloadKeyForError(error: Error): string {
  const failedUrl = error.message.match(/https?:\/\/\S+/)?.[0] ?? error.message;
  return `${CHUNK_RELOAD_KEY_PREFIX}:${failedUrl}`;
}

// React.lazy's public type uses `ComponentType<any>` for generic component props.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export function lazyWithChunkReload<T extends ComponentType<any>>(
  loader: () => Promise<{ default: T }>,
): LazyExoticComponent<T> {
  return lazy(async () => {
    try {
      return await loader();
    } catch (error) {
      if (isChunkLoadError(error) && typeof window !== "undefined") {
        const storageKey = reloadKeyForError(error);
        if (!window.sessionStorage.getItem(storageKey)) {
          window.sessionStorage.setItem(storageKey, String(Date.now()));
          window.location.reload();
          return new Promise<{ default: T }>(() => {});
        }
      }

      throw error;
    }
  });
}
