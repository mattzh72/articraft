import { useRef, useEffect, useState } from 'react';
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
  sceneReady: boolean;
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
): ThreeSceneState {
  const [sceneReady, setSceneReady] = useState(false);

  // Mutable refs for scene objects so they persist without triggering re-renders.
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const gridGroupRef = useRef<THREE.Group | null>(null);
  const axisGroupRef = useRef<THREE.Group | null>(null);
  const frameIdRef = useRef<number>(0);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    // --- Renderer ---
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.3;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
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

    // --- Render loop ---
    const tick = () => {
      controls.update();
      renderer.render(scene, camera);
      frameIdRef.current = requestAnimationFrame(tick);
    };
    tick();

    // --- ResizeObserver ---
    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width === 0 || height === 0) continue;
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
      }
    });
    resizeObserver.observe(container);

    setSceneReady(true);

    // --- Cleanup ---
    return () => {
      setSceneReady(false);
      cancelAnimationFrame(frameIdRef.current);
      resizeObserver.disconnect();
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
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  return {
    scene: sceneRef.current,
    camera: cameraRef.current,
    renderer: rendererRef.current,
    controls: controlsRef.current,
    gridGroup: gridGroupRef.current,
    axisGroup: axisGroupRef.current,
    sceneReady,
  };
}
