import { useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createLightingRig, createGridHelper, createAxisHelper, createEnvironmentMap } from './lighting';

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
  fancyGraphics?: boolean;
}

type NamedLight = THREE.Light & {
  color: THREE.Color;
  groundColor?: THREE.Color;
};

const VIEWPORT_BACKGROUND = 0xe8edf5;

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

function getNamedLight(group: THREE.Group, name: string): NamedLight | null {
  const object = group.getObjectByName(name);
  return object instanceof THREE.Light ? object as NamedLight : null;
}

function getNamedDirectionalLight(group: THREE.Group, name: string): THREE.DirectionalLight | null {
  const object = group.getObjectByName(name);
  return object instanceof THREE.DirectionalLight ? object : null;
}

function applyLightingQuality(lightingRig: THREE.Group, fancyGraphics: boolean): void {
  const hemiLight = getNamedLight(lightingRig, 'hemisphere-light');
  const keyLight = getNamedDirectionalLight(lightingRig, 'key-light');
  const fillLight = getNamedDirectionalLight(lightingRig, 'fill-light');
  const rimLight = getNamedDirectionalLight(lightingRig, 'rim-light');
  const frontFillLight = getNamedDirectionalLight(lightingRig, 'front-fill-light');

  if (fancyGraphics) {
    if (hemiLight) {
      hemiLight.color.set(0xffffff);
      hemiLight.groundColor?.set(0xdfe3e8);
      hemiLight.intensity = 0.22;
    }
    if (keyLight) {
      keyLight.color.set(0xffffff);
      keyLight.intensity = 1.05;
      keyLight.position.set(5, 9, 6);
      keyLight.shadow.bias = -0.00008;
      keyLight.shadow.normalBias = 0.035;
      keyLight.shadow.radius = 3;
    }
    if (fillLight) {
      fillLight.color.set(0xf4f7ff);
      fillLight.intensity = 0.12;
      fillLight.position.set(-7, 4, -5);
    }
    if (rimLight) {
      rimLight.color.set(0xffffff);
      rimLight.intensity = 0.48;
      rimLight.position.set(-5, 5, -8);
    }
    if (frontFillLight) {
      frontFillLight.color.set(0xffffff);
      frontFillLight.intensity = 0.04;
      frontFillLight.position.set(0, 4, 7);
    }
    return;
  }

  if (hemiLight) {
    hemiLight.color.set(0xffffff);
    hemiLight.groundColor?.set(0xd4d4d4);
    hemiLight.intensity = 0.95;
  }
  if (keyLight) {
    keyLight.color.set(0xfff5e6);
    keyLight.intensity = 0.75;
    keyLight.position.set(6, 12, 8);
    keyLight.shadow.bias = 0;
    keyLight.shadow.normalBias = 0;
    keyLight.shadow.radius = 1;
  }
  if (fillLight) {
    fillLight.color.set(0xf0f4ff);
    fillLight.intensity = 0.65;
    fillLight.position.set(-6, 5, -6);
  }
  if (rimLight) {
    rimLight.color.set(0xffffff);
    rimLight.intensity = 0.35;
    rimLight.position.set(0, -6, 10);
  }
  if (frontFillLight) {
    frontFillLight.color.set(0xfff9f0);
    frontFillLight.intensity = 0.5;
    frontFillLight.position.set(0, 6, 10);
  }
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
  const lightingRigRef = useRef<THREE.Group | null>(null);
  const environmentMapRef = useRef<THREE.Texture | null>(null);
  const frameIdRef = useRef<number>(0);
  const lastFrameTimeRef = useRef<number | null>(null);
  const continuousRenderRef = useRef(Boolean(options.continuousRender));
  const fancyGraphicsRef = useRef(Boolean(options.fancyGraphics));
  const needsRenderRef = useRef(false);
  const invalidateRef = useRef<() => void>(() => {});

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    // --- Renderer ---
    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: false,
      preserveDrawingBuffer: true,
      ...resolveDepthBufferOptions(),
    });
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, options.maxPixelRatio ?? 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.domElement.style.display = 'block';
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // --- Scene ---
    const scene = new THREE.Scene();
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
    lightingRigRef.current = lightingRig;

    // --- Grid ---
    const gridGroup = createGridHelper();
    scene.add(gridGroup);
    gridGroupRef.current = gridGroup;

    // --- Axis Helper ---
    const axisGroup = createAxisHelper();
    scene.add(axisGroup);
    axisGroupRef.current = axisGroup;

    const applyGraphicsQuality = (fancyGraphics: boolean) => {
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.toneMapping = fancyGraphics ? THREE.NoToneMapping : THREE.LinearToneMapping;
      renderer.toneMappingExposure = 1.0;
      renderer.shadowMap.enabled = fancyGraphics;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      scene.background = new THREE.Color(VIEWPORT_BACKGROUND);
      applyLightingQuality(lightingRig, fancyGraphics);

      if (fancyGraphics) {
        if (!environmentMapRef.current) {
          environmentMapRef.current = createEnvironmentMap(renderer);
        }
        scene.environment = environmentMapRef.current;
        return;
      }

      scene.environment = null;
      environmentMapRef.current?.dispose();
      environmentMapRef.current = null;
    };

    fancyGraphicsRef.current = Boolean(options.fancyGraphics);
    applyGraphicsQuality(fancyGraphicsRef.current);

    const renderScene = (deltaSeconds?: number) => {
      void deltaSeconds;
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

      const hadInvalidation = needsRenderRef.current;
      needsRenderRef.current = false;
      const controlsChanged = controls.update(deltaSeconds ?? undefined);
      if (continuousRenderRef.current || hadInvalidation || controlsChanged) {
        renderScene(deltaSeconds ?? undefined);
      }

      if (continuousRenderRef.current || controlsChanged) {
        requestFrame();
        return;
      }

      lastFrameTimeRef.current = null;
    };

    const invalidate = () => {
      needsRenderRef.current = true;
      requestFrame();
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
      environmentMapRef.current?.dispose();
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
      lightingRigRef.current = null;
      environmentMapRef.current = null;
      lastFrameTimeRef.current = null;
      needsRenderRef.current = false;
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
    const renderer = rendererRef.current;
    const scene = sceneRef.current;
    const camera = cameraRef.current;
    const lightingRig = lightingRigRef.current;
    const container = containerRef.current;
    if (!renderer || !scene || !camera || !lightingRig || !container) {
      return;
    }

    const fancyGraphics = Boolean(options.fancyGraphics);
    fancyGraphicsRef.current = fancyGraphics;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = fancyGraphics ? THREE.NoToneMapping : THREE.LinearToneMapping;
    renderer.toneMappingExposure = 1.0;
    renderer.shadowMap.enabled = fancyGraphics;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    scene.background = new THREE.Color(VIEWPORT_BACKGROUND);
    applyLightingQuality(lightingRig, fancyGraphics);

    if (fancyGraphics) {
      if (!environmentMapRef.current) {
        environmentMapRef.current = createEnvironmentMap(renderer);
      }
      scene.environment = environmentMapRef.current;
      invalidateRef.current();
      return;
    }

    scene.environment = null;
    environmentMapRef.current?.dispose();
    environmentMapRef.current = null;
    invalidateRef.current();
  }, [containerRef, options.fancyGraphics, sceneReady]);

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
