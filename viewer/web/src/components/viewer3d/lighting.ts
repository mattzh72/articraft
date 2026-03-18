import * as THREE from 'three';

/**
 * Create the standard lighting rig for the URDF viewer scene.
 * Includes hemisphere ambient light, key directional light with shadows,
 * a fill light, and a rim light.
 */
export function createLightingRig(): THREE.Group {
  const group = new THREE.Group();
  group.name = 'lighting-rig';

  // Hemisphere light for soft ambient illumination
  const hemiLight = new THREE.HemisphereLight(0xf4f5ff, 0x62676f, 0.75);
  hemiLight.name = 'hemisphere-light';
  group.add(hemiLight);

  // Key directional light with shadows
  const keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
  keyLight.name = 'key-light';
  keyLight.position.set(5, 10, 7);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.width = 2048;
  keyLight.shadow.mapSize.height = 2048;
  keyLight.shadow.camera.near = 0.1;
  keyLight.shadow.camera.far = 50;
  keyLight.shadow.camera.left = -10;
  keyLight.shadow.camera.right = 10;
  keyLight.shadow.camera.top = 10;
  keyLight.shadow.camera.bottom = -10;
  group.add(keyLight);

  // Fill light (slightly blue tint, softer)
  const fillLight = new THREE.DirectionalLight(0xaab8ff, 0.45);
  fillLight.name = 'fill-light';
  fillLight.position.set(-5, 3, -5);
  group.add(fillLight);

  // Rim light (behind, for edge definition)
  const rimLight = new THREE.DirectionalLight(0xffffff, 0.28);
  rimLight.name = 'rim-light';
  rimLight.position.set(0, -5, 10);
  group.add(rimLight);

  // Low frontal fill to keep dark front-facing panels readable.
  const frontFillLight = new THREE.DirectionalLight(0xfff7ef, 0.22);
  frontFillLight.name = 'front-fill-light';
  frontFillLight.position.set(0, 4, 8);
  group.add(frontFillLight);

  return group;
}

/**
 * Create a fine-grained ground-plane grid helper.
 * The grid is centered at the origin and can be repositioned later.
 */
export function createGridHelper(): THREE.Group {
  const group = new THREE.Group();
  group.name = 'grid-group';

  // Fine grid (small subdivisions) - subtle like Figma's canvas grid
  const fineGrid = new THREE.GridHelper(10, 100, 0xc0c0c0, 0xd0d0d0);
  fineGrid.name = 'grid-fine';
  const fineMat = fineGrid.material;
  if (Array.isArray(fineMat)) {
    for (const m of fineMat) {
      m.transparent = true;
      m.opacity = 0.45;
    }
  } else {
    fineMat.transparent = true;
    fineMat.opacity = 0.45;
  }
  group.add(fineGrid);

  // Coarse grid (major lines every 1 unit) - slightly stronger for reference
  const coarseGrid = new THREE.GridHelper(10, 10, 0x999999, 0x999999);
  coarseGrid.name = 'grid-coarse';
  const coarseMat = coarseGrid.material;
  if (Array.isArray(coarseMat)) {
    for (const m of coarseMat) {
      m.transparent = true;
      m.opacity = 0.35;
    }
  } else {
    coarseMat.transparent = true;
    coarseMat.opacity = 0.35;
  }
  group.add(coarseGrid);

  return group;
}

/**
 * Create colored XYZ axis lines.
 * X = red, Y = green, Z = blue.
 */
export function createAxisHelper(size: number = 2): THREE.Group {
  const group = new THREE.Group();
  group.name = 'axis-helper';

  const axes: Array<{ dir: THREE.Vector3; color: number }> = [
    { dir: new THREE.Vector3(1, 0, 0), color: 0xe74c3c }, // X = red
    { dir: new THREE.Vector3(0, 1, 0), color: 0x2ecc71 }, // Y = green
    { dir: new THREE.Vector3(0, 0, 1), color: 0x3498db }, // Z = blue
  ];

  for (const { dir, color } of axes) {
    const points = [
      new THREE.Vector3(0, 0, 0),
      dir.clone().multiplyScalar(size),
    ];
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color, linewidth: 2 });
    const line = new THREE.Line(geometry, material);
    group.add(line);
  }

  return group;
}

/**
 * Position the grid and axis helper underneath the model's bounding box.
 */
export function positionGroundHelpers(
  gridGroup: THREE.Group,
  axisGroup: THREE.Group,
  robotGroup: THREE.Object3D,
): void {
  const box = new THREE.Box3().setFromObject(robotGroup);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const bottomY = box.min.y;

  // Position grid at the bottom of the model, centered on the model's XZ footprint
  gridGroup.position.set(center.x, bottomY, center.z);

  // Scale grid to fit the model - at least 3x the model's footprint
  const footprint = Math.max(size.x, size.z, 1) * 3;
  const gridScale = footprint / 10; // base grid is 10 units
  gridGroup.scale.set(gridScale, 1, gridScale);

  // Position axis at the bottom center of the model
  axisGroup.position.set(center.x, bottomY, center.z);

  // Scale axis to be proportional to the model size
  const axisScale = Math.max(size.x, size.y, size.z, 0.5) * 0.4;
  axisGroup.scale.setScalar(axisScale);
}

/**
 * Create a simple studio-style environment map using PMREMGenerator.
 * Provides subtle reflections on metallic/glossy materials.
 */
export function createEnvironmentMap(renderer: THREE.WebGLRenderer): THREE.Texture {
  const pmremGenerator = new THREE.PMREMGenerator(renderer);
  pmremGenerator.compileEquirectangularShader();

  // Build a simple gradient scene to use as an environment map
  const envScene = new THREE.Scene();

  // Top hemisphere color
  const topColor = new THREE.Color(0xdbe7ff);
  // Bottom hemisphere color
  const bottomColor = new THREE.Color(0x8a8f98);
  // Equator accent
  const midColor = new THREE.Color(0xfafcff);

  // Use a large sphere with a gradient shader as environment
  const envGeometry = new THREE.SphereGeometry(100, 32, 16);
  const envMaterial = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: {
      topColor: { value: topColor },
      midColor: { value: midColor },
      bottomColor: { value: bottomColor },
    },
    vertexShader: /* glsl */ `
      varying vec3 vWorldPosition;
      void main() {
        vec4 worldPos = modelMatrix * vec4(position, 1.0);
        vWorldPosition = worldPos.xyz;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform vec3 topColor;
      uniform vec3 midColor;
      uniform vec3 bottomColor;
      varying vec3 vWorldPosition;
      void main() {
        float h = normalize(vWorldPosition).y;
        vec3 color;
        if (h > 0.0) {
          color = mix(midColor, topColor, h);
        } else {
          color = mix(midColor, bottomColor, -h);
        }
        gl_FragColor = vec4(color, 1.0);
      }
    `,
  });

  const envMesh = new THREE.Mesh(envGeometry, envMaterial);
  envScene.add(envMesh);

  const envMap = pmremGenerator.fromScene(envScene, 0.04).texture;
  pmremGenerator.dispose();
  envGeometry.dispose();
  envMaterial.dispose();

  return envMap;
}
