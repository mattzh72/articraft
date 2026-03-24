import { useState, useCallback, useEffect, useRef } from 'react';

import { updateUrlSearchParams } from '@/lib/url';

export interface RenderOptions {
  showEdges: boolean;
  showGrid: boolean;
  showCollisions: boolean;
  showSegmentColors: boolean;
  showSurfaceSamples: boolean;
  doubleSided: boolean;
  envLighting: boolean;
  autoAnimate: boolean;
  showJointOverlay: boolean;
}

const STORAGE_KEY = 'articraft-render-options';
const STORAGE_SCHEMA_VERSION = 4;
const RENDER_QUERY_PARAM = 'render';
const RENDER_OPTION_KEYS: Array<keyof RenderOptions> = [
  'showEdges',
  'showGrid',
  'showCollisions',
  'showSegmentColors',
  'doubleSided',
  'envLighting',
  'autoAnimate',
  'showJointOverlay',
  'showSurfaceSamples',
];

const DEFAULT_OPTIONS: RenderOptions = {
  showEdges: true,
  showGrid: true,
  showCollisions: false,
  showSegmentColors: false,
  showSurfaceSamples: false,
  doubleSided: true,
  envLighting: false,
  autoAnimate: false,
  showJointOverlay: false,
};

const DEFAULT_RENDER_QUERY_VALUE = serializeOptions(DEFAULT_OPTIONS);

export interface RenderOptionsState {
  options: RenderOptions;
  setOption: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
  resetOptions: () => void;
}

/**
 * Read persisted render options from localStorage, falling back to defaults.
 */
function loadOptions(): RenderOptions {
  let options = { ...DEFAULT_OPTIONS };

  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw) as Partial<RenderOptions> & { schemaVersion?: number };
      if (parsed.schemaVersion === STORAGE_SCHEMA_VERSION) {
        options = { ...DEFAULT_OPTIONS, ...parsed };
      } else {
        options = {
          ...DEFAULT_OPTIONS,
          ...parsed,
          showSegmentColors: false,
          doubleSided: true,
        };
      }
    }
  } catch {
    // Ignore corrupt or inaccessible storage.
  }

  if (typeof window === 'undefined') {
    return options;
  }

  try {
    const params = new URLSearchParams(window.location.search);
    const fromUrl = parseOptions(params.get(RENDER_QUERY_PARAM));
    if (fromUrl) {
      return fromUrl;
    }
  } catch {
    // Ignore malformed URLs.
  }

  return options;
}

function serializeOptions(options: RenderOptions): string {
  return RENDER_OPTION_KEYS.map((key) => (options[key] ? '1' : '0')).join('');
}

function parseOptions(value: string | null): RenderOptions | null {
  if (!value) {
    return null;
  }

  const nextOptions = { ...DEFAULT_OPTIONS };
  const keys =
    value.length === RENDER_OPTION_KEYS.length
      ? RENDER_OPTION_KEYS
      : value.length === RENDER_OPTION_KEYS.length - 1
        ? RENDER_OPTION_KEYS.slice(0, -1)
        : null;
  if (!keys) {
    return null;
  }

  for (const [index, key] of keys.entries()) {
    const bit = value[index];
    if (bit !== '0' && bit !== '1') {
      return null;
    }
    nextOptions[key] = bit === '1';
  }

  return nextOptions;
}

/**
 * Hook for managing render option toggles.
 *
 * State is persisted to localStorage under the key 'articraft-render-options'.
 */
export function useRenderOptions(): RenderOptionsState {
  const [options, setOptions] = useState<RenderOptions>(loadOptions);
  const isInitialMount = useRef(true);

  // Persist to localStorage whenever options change (skip the initial mount).
  useEffect(() => {
    if (isInitialMount.current) {
      isInitialMount.current = false;
      return;
    }
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({
        ...options,
        schemaVersion: STORAGE_SCHEMA_VERSION,
      }));
    } catch {
      // Storage full or unavailable -- silently ignore.
    }
  }, [options]);

  useEffect(() => {
    const serialized = serializeOptions(options);
    updateUrlSearchParams((params) => {
      if (serialized === DEFAULT_RENDER_QUERY_VALUE) {
        params.delete(RENDER_QUERY_PARAM);
      } else {
        params.set(RENDER_QUERY_PARAM, serialized);
      }
    });
  }, [options]);

  const setOption = useCallback(
    <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => {
      setOptions((prev) => ({ ...prev, [key]: value }));
    },
    [],
  );

  const resetOptions = useCallback(() => {
    setOptions({ ...DEFAULT_OPTIONS });
  }, []);

  return { options, setOption, resetOptions };
}
