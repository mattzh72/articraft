import { useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createLightingRig, createGridHelper, createAxisHelper } from './lighting';

export interface ThreeSceneState {
  scene: THREE.Scene | null;
  camera: THREE.PerspectiveCamera | null;
  renderer: THREE.WebGLRenderer | null;
  controls: OrbitControls | null;
  gridGroup: THREE.Group | null;
  axisGroup: THREE.Group | null;
  invalidate: () => void;
  sceneReady: boolean;
}

export interface ThreeSceneOptions {
  maxPixelRatio?: number;
  continuousRender?: boolean;
}

function resolveDepthBufferOptions(): Pick<
  THREE.WebGLRendererParameters,
  'logarithmicDepthBuffer' | 'reversedDepthBuffer'
> {
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('webgl2');
  const supportsReversedDepth = context?.getExtension('EXT_clip_control') != null;
  context?.getExtension('WEBGL_lose_context')?.loseContext();

  return supportsReversedDepth
    ? { reversedDepthBuffer: true }
    : { logarithmicDepthBuffer: true };
}

/**
 * Hook that manages the Three.js renderer lifecycle.
 *
 * Creates and manages: WebGLRenderer, PerspectiveCamera, Scene, OrbitControls.
 * Sets up: lighting rig, grid helper, axis helper, render loop via requestAnimationFrame.
 * Handles: ResizeObserver for container size changes, cleanup on unmount.
 */
export function useThreeScene(
  containerRef: React.RefObject<HTMLDivElement | null>,
  options: ThreeSceneOptions = {},
): ThreeSceneState {
  const [sceneReady, setSceneReady] = useState(false);
  const invalidate = useCallback(() => {
    invalidateRef.current();
  }, []);

  // Mutable refs for scene objects so they persist without triggering re-renders.
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const gridGroupRef = useRef<THREE.Group | null>(null);
  const axisGroupRef = useRef<THREE.Group | null>(null);
  const frameIdRef = useRef<number>(0);
  const lastFrameTimeRef = useRef<number | null>(null);
  const continuousRenderRef = useRef(Boolean(options.continuousRender));
  const invalidateRef = useRef<() => void>(() => {});

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    // --- Renderer ---
    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: false,
      ...resolveDepthBufferOptions(),
    });
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.3;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, options.maxPixelRatio ?? 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.domElement.style.display = 'block';
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // --- Scene ---
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xe8edf5);
    sceneRef.current = scene;

    // --- Camera ---
    const aspect = container.clientWidth / container.clientHeight;
    const camera = new THREE.PerspectiveCamera(45, aspect, 0.01, 1000);
    camera.position.set(3, 3, 3);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    // --- OrbitControls ---
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.target.set(0, 0, 0);
    controlsRef.current = controls;

    // --- Lighting ---
    const lightingRig = createLightingRig();
    scene.add(lightingRig);

    // --- Grid ---
    const gridGroup = createGridHelper();
    scene.add(gridGroup);
    gridGroupRef.current = gridGroup;

    // --- Axis Helper ---
    const axisGroup = createAxisHelper();
    scene.add(axisGroup);
    axisGroupRef.current = axisGroup;

    const renderScene = () => {
      renderer.render(scene, camera);
    };

    const requestFrame = () => {
      if (frameIdRef.current === 0) {
        frameIdRef.current = requestAnimationFrame(renderFrame);
      }
    };

    const renderFrame = (now: number) => {
      frameIdRef.current = 0;

      const deltaSeconds =
        lastFrameTimeRef.current == null ? null : (now - lastFrameTimeRef.current) / 1000;
      lastFrameTimeRef.current = now;

      const controlsChanged = controls.update(deltaSeconds ?? undefined);
      if (continuousRenderRef.current || controlsChanged) {
        renderScene();
      }

      if (continuousRenderRef.current || controlsChanged) {
        requestFrame();
        return;
      }

      lastFrameTimeRef.current = null;
    };

    const invalidate = () => {
      renderScene();
      if (continuousRenderRef.current || controls.enableDamping) {
        requestFrame();
      }
    };

    invalidateRef.current = invalidate;
    controls.addEventListener('change', invalidate);
    invalidate();

    // --- ResizeObserver ---
    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width === 0 || height === 0) continue;
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
        invalidate();
      }
    });
    resizeObserver.observe(container);

    setSceneReady(true);

    // --- Cleanup ---
    return () => {
      setSceneReady(false);
      cancelAnimationFrame(frameIdRef.current);
      resizeObserver.disconnect();
      controls.removeEventListener('change', invalidate);
      controls.dispose();
      renderer.domElement.remove();
      renderer.dispose();

      // Dispose scene contents.
      scene.traverse((object) => {
        if (object instanceof THREE.Mesh) {
          object.geometry?.dispose();
          const mat = object.material;
          if (Array.isArray(mat)) {
            mat.forEach((m) => m.dispose());
          } else {
            mat?.dispose();
          }
        }
      });

      sceneRef.current = null;
      cameraRef.current = null;
      rendererRef.current = null;
      controlsRef.current = null;
      gridGroupRef.current = null;
      axisGroupRef.current = null;
      lastFrameTimeRef.current = null;
      invalidateRef.current = () => {};
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    const renderer = rendererRef.current;
    const container = containerRef.current;
    if (!renderer || !container) {
      return;
    }

    renderer.setPixelRatio(Math.min(window.devicePixelRatio, options.maxPixelRatio ?? 2));
    renderer.setSize(container.clientWidth, container.clientHeight, false);
    invalidateRef.current();
  }, [containerRef, options.maxPixelRatio, sceneReady]);

  useEffect(() => {
    continuousRenderRef.current = Boolean(options.continuousRender);
    invalidateRef.current();
  }, [options.continuousRender]);

  return {
    scene: sceneRef.current,
    camera: cameraRef.current,
    renderer: rendererRef.current,
    controls: controlsRef.current,
    gridGroup: gridGroupRef.current,
    axisGroup: axisGroupRef.current,
    invalidate,
    sceneReady,
  };
}
