import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DObject, CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';
import {
  ARM,
  BENCH,
  JOINTS,
  REACH,
  SHOULDER_Z,
  WORK_BAND,
  jacobian,
  onBench,
  overBench,
  reachOnBench,
  solveForward,
  solveInverse,
  wrapDegrees,
} from './crane-kinematics.js';
import { SHADOW_MAP_TYPE, createRenderer, showRendererFallback } from './webgl.js';

const DEG = Math.PI / 180;
const clamp = THREE.MathUtils.clamp;
const fmt = (value, digits = 3) => (Math.abs(value) < 0.0005 ? 0 : value).toFixed(digits);
const UP = new THREE.Vector3(0, 0, 1);

/**
 * Mount the bench-mounted crane viewer inside `root` and return its frame hooks.
 *
 * The model is a real joint hierarchy — turret on the bench, mast, shoulder
 * hinge, upper arm, elbow hinge, forearm, tool — so what the sliders do to the
 * geometry is exactly what they would do to a printed arm.
 */
export function createCraneView(root) {
  const $ = (selector) => root.querySelector(selector);
  const viewport = $('#spatial-viewport');

  /* ------------------------------------------------------------------ *
   * Renderer, camera, orbit
   * ------------------------------------------------------------------ */
  const renderer = createRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
  if (!renderer) showRendererFallback(viewport);
  if (renderer) {
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.08;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = SHADOW_MAP_TYPE;
    renderer.domElement.setAttribute('role', 'img');
    renderer.domElement.setAttribute('aria-label', '3D model of a three axis crane arm bolted to a bench: a vertical yaw axis, a shoulder hinge on a short mast and an elbow hinge that folds the forearm down to the bench top');
    renderer.domElement.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;outline:none;cursor:grab;';
    viewport.prepend(renderer.domElement);
  }
  const canvas = renderer?.domElement;

  const labelRenderer = new CSS2DRenderer();
  labelRenderer.domElement.classList.add('labels-layer');
  labelRenderer.domElement.style.cssText = 'position:absolute;inset:0;overflow:hidden;pointer-events:none;z-index:2;';
  viewport.appendChild(labelRenderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 40);
  camera.up.copy(UP);

  const benchCenter = new THREE.Vector3(BENCH.center.x, BENCH.center.y, BENCH.height);
  const views = {
    bench: { eye: new THREE.Vector3(1.95, -2.05, 1.85), target: new THREE.Vector3(0.26, 0.0, 0.86), sideDistance: 2.85, sideHeight: 0.62 },
    plan: { eye: new THREE.Vector3(0.26, -0.42, 3.3), target: new THREE.Vector3(0.26, 0.0, BENCH.height) },
  };
  const camState = { eye: views.bench.eye.clone(), target: views.bench.target.clone() };
  const camGoal = { eye: views.bench.eye.clone(), target: views.bench.target.clone() };
  let activeView = 'bench';

  const controls = renderer ? new OrbitControls(camera, renderer.domElement) : null;
  if (controls) {
    camera.position.copy(camState.eye);
    controls.target.copy(camState.target);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 1.1;
    controls.maxDistance = 7;
    controls.maxPolarAngle = Math.PI * 0.497;
    controls.addEventListener('start', () => {
      syncingView = false;
      if (activeView === 'side') {
        activeView = 'free';
        syncViewButtons();
      }
      stopDemo();
    });
    controls.update();
  }
  let syncingView = false;

  /* ------------------------------------------------------------------ *
   * Lights
   * ------------------------------------------------------------------ */
  scene.add(new THREE.HemisphereLight(0xe4f7f2, 0x14202a, 1.7));
  const keyLight = new THREE.DirectionalLight(0xfff2df, 2.7);
  keyLight.position.set(-2.2, -2.8, 4.4);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(1024, 1024);
  keyLight.shadow.camera.left = -2.4;
  keyLight.shadow.camera.right = 2.4;
  keyLight.shadow.camera.top = 2.4;
  keyLight.shadow.camera.bottom = -2.4;
  keyLight.shadow.camera.near = 0.1;
  keyLight.shadow.camera.far = 10;
  keyLight.shadow.bias = -0.0009;
  scene.add(keyLight);
  const rimLight = new THREE.DirectionalLight(0x58caba, 1.4);
  rimLight.position.set(2.6, 2.4, 2.4);
  scene.add(rimLight);

  /* ------------------------------------------------------------------ *
   * Materials & primitives
   * ------------------------------------------------------------------ */
  const materials = {
    housing: new THREE.MeshStandardMaterial({ color: '#29353d', metalness: 0.76, roughness: 0.34 }),
    shell: new THREE.MeshStandardMaterial({ color: '#bfccca', metalness: 0.7, roughness: 0.26 }),
    shellLight: new THREE.MeshStandardMaterial({ color: '#dde6df', metalness: 0.64, roughness: 0.22 }),
    dark: new THREE.MeshStandardMaterial({ color: '#101a20', metalness: 0.62, roughness: 0.36 }),
    accent: new THREE.MeshStandardMaterial({ color: '#f3925e', metalness: 0.5, roughness: 0.26, emissive: '#3d1807', emissiveIntensity: 0.24 }),
    accentSoft: new THREE.MeshStandardMaterial({ color: '#e4a077', metalness: 0.62, roughness: 0.3 }),
    teal: new THREE.MeshStandardMaterial({ color: '#5ee2bd', metalness: 0.36, roughness: 0.22, emissive: '#105943', emissiveIntensity: 0.7 }),
    fastener: new THREE.MeshStandardMaterial({ color: '#d6dedb', metalness: 0.82, roughness: 0.19 }),
    bench: new THREE.MeshStandardMaterial({ color: '#16242c', metalness: 0.42, roughness: 0.62 }),
    benchTop: new THREE.MeshStandardMaterial({ color: '#1d2f38', metalness: 0.3, roughness: 0.55 }),
    benchEdge: new THREE.MeshStandardMaterial({ color: '#2f4a52', metalness: 0.5, roughness: 0.45 }),
  };

  function box(width, height, depth, material, position) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), material);
    if (position) mesh.position.set(...position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  /** Cylinder whose axis is the vertical Z axis. */
  function disc(radius, height, material, segments = 34, topRadius = radius) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(topRadius, radius, height, segments, 1), material);
    mesh.rotation.x = Math.PI / 2;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  /** Cylinder whose axis is the local Y axis: a hinge pin or bearing boss. */
  function hinge(radius, length, material, segments = 30) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, segments, 1), material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  /* ------------------------------------------------------------------ *
   * Floor & bench
   * ------------------------------------------------------------------ */
  function makeFloorTexture() {
    const size = 1024;
    const span = 6;
    const textureCanvas = document.createElement('canvas');
    textureCanvas.width = size;
    textureCanvas.height = size;
    const ctx = textureCanvas.getContext('2d');
    ctx.fillStyle = '#0a141b';
    ctx.fillRect(0, 0, size, size);
    const glow = ctx.createRadialGradient(size / 2, size / 2, 60, size / 2, size / 2, 700);
    glow.addColorStop(0, 'rgba(32, 74, 78, 0.4)');
    glow.addColorStop(0.6, 'rgba(16, 36, 44, 0.2)');
    glow.addColorStop(1, 'rgba(8, 16, 22, 0)');
    ctx.fillStyle = glow;
    ctx.fillRect(0, 0, size, size);
    const half = span / 2;
    for (let step = 0; step <= span * 10; step += 1) {
      const metres = -half + step * 0.1;
      const position = (metres + half) * (size / span);
      const major = Math.abs(Math.abs(metres) % 0.5) < 1e-6;
      ctx.strokeStyle = major ? 'rgba(118, 165, 172, 0.24)' : 'rgba(92, 132, 141, 0.11)';
      ctx.lineWidth = major ? 1.7 : 1;
      ctx.beginPath(); ctx.moveTo(position, 0); ctx.lineTo(position, size); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, position); ctx.lineTo(size, position); ctx.stroke();
    }
    const texture = new THREE.CanvasTexture(textureCanvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = renderer ? renderer.capabilities.getMaxAnisotropy() : 1;
    return texture;
  }

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(6, 6),
    new THREE.MeshStandardMaterial({ map: makeFloorTexture(), roughness: 0.95, metalness: 0.05 }),
  );
  floor.receiveShadow = true;
  scene.add(floor);

  const benchGroup = new THREE.Group();
  scene.add(benchGroup);

  const topThickness = 0.04;
  const slab = box(BENCH.width, BENCH.depth, topThickness, materials.benchTop,
    [BENCH.center.x, BENCH.center.y, BENCH.height - topThickness / 2]);
  benchGroup.add(slab);
  const edge = box(BENCH.width + 0.02, BENCH.depth + 0.02, 0.008, materials.benchEdge,
    [BENCH.center.x, BENCH.center.y, BENCH.height - topThickness + 0.004]);
  benchGroup.add(edge);

  // Frame and legs, so the bench reads as furniture rather than a slab.
  const legTop = BENCH.height - topThickness;
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const leg = box(0.05, 0.05, legTop, materials.bench, [
        BENCH.center.x + sx * (BENCH.width / 2 - 0.07),
        BENCH.center.y + sy * (BENCH.depth / 2 - 0.07),
        legTop / 2,
      ]);
      benchGroup.add(leg);
      benchGroup.add(box(0.12, 0.12, 0.012, materials.benchEdge, [leg.position.x, leg.position.y, 0.006]));
    }
    const railZ = [0.12, legTop - 0.14];
    for (const z of railZ) {
      benchGroup.add(box(0.03, BENCH.depth - 0.14, 0.03, materials.bench, [
        BENCH.center.x + sx * (BENCH.width / 2 - 0.07),
        BENCH.center.y,
        z,
      ]));
    }
  }
  for (const sy of [-1, 1]) {
    benchGroup.add(box(BENCH.width - 0.14, 0.03, 0.03, materials.bench, [
      BENCH.center.x,
      BENCH.center.y + sy * (BENCH.depth / 2 - 0.07),
      0.12,
    ]));
  }

  // Bench label.
  const benchTag = document.createElement('div');
  benchTag.className = 'plane-tag';
  benchTag.innerHTML = `<b>BENCH</b> WORK SURFACE <span>z ${BENCH.height.toFixed(2)} m</span>`;
  const benchLabel = new CSS2DObject(benchTag);
  benchLabel.position.set(BENCH.center.x - BENCH.width / 2 - 0.12, BENCH.center.y + BENCH.depth / 2 + 0.1, BENCH.height - 0.05);
  benchGroup.add(benchLabel);

  /* ------------------------------------------------------------------ *
   * Workspace overlay: the band of the bench the tool can be driven over
   * ------------------------------------------------------------------ */
  const workspaceLayer = new THREE.Group();
  scene.add(workspaceLayer);

  function bandMesh(inner, outer) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(inner, outer, 128, 1),
      new THREE.MeshBasicMaterial({ color: 0x5ee2bd, transparent: true, opacity: 0.055, side: THREE.DoubleSide, depthWrite: false }),
    );
    return ring;
  }

  function ringLine(radius, opacity = 0.42) {
    const points = [];
    for (let i = 0; i <= 128; i += 1) {
      const a = (i / 128) * Math.PI * 2;
      points.push(new THREE.Vector3(Math.cos(a) * radius, Math.sin(a) * radius, 0));
    }
    return new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: 0x5ee2bd, transparent: true, opacity }),
    );
  }

  const bandFill = bandMesh(WORK_BAND.inner, WORK_BAND.outer);
  bandFill.position.z = BENCH.height + 0.0015;
  workspaceLayer.add(bandFill);
  const bandInner = ringLine(WORK_BAND.inner);
  const bandOuter = ringLine(WORK_BAND.outer);
  bandInner.position.z = BENCH.height + 0.002;
  bandOuter.position.z = BENCH.height + 0.002;
  workspaceLayer.add(bandInner, bandOuter);

  // Radial ticks every 15° make the yaw travel easy to read from above.
  const tickPoints = [];
  for (let deg = -150; deg <= 150; deg += 15) {
    const a = deg * DEG;
    const long = Math.abs(deg) % 45 === 0;
    const r0 = WORK_BAND.inner;
    const r1 = WORK_BAND.inner + (long ? 0.06 : 0.035);
    tickPoints.push(new THREE.Vector3(Math.cos(a) * r0, Math.sin(a) * r0, 0));
    tickPoints.push(new THREE.Vector3(Math.cos(a) * r1, Math.sin(a) * r1, 0));
  }
  const ticks = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(tickPoints),
    new THREE.LineBasicMaterial({ color: 0x5ee2bd, transparent: true, opacity: 0.32 }),
  );
  ticks.position.z = BENCH.height + 0.002;
  workspaceLayer.add(ticks);

  const bandTag = document.createElement('div');
  bandTag.className = 'plane-tag';
  bandTag.innerHTML = `<b>BAND</b> ${WORK_BAND.inner.toFixed(2)}–${WORK_BAND.outer.toFixed(2)} m`;
  const bandLabel = new CSS2DObject(bandTag);
  bandLabel.position.set(0, -(WORK_BAND.outer + 0.09), BENCH.height + 0.01);
  workspaceLayer.add(bandLabel);

  /* ------------------------------------------------------------------ *
   * The arm
   * ------------------------------------------------------------------ */
  const armRoot = new THREE.Group();
  armRoot.position.set(0, 0, BENCH.height);
  scene.add(armRoot);

  // Base plate bolted to the bench.
  armRoot.add(box(0.26, 0.26, 0.016, materials.housing, [0, 0, 0.008]));
  armRoot.add(box(0.2, 0.2, 0.004, materials.dark, [0, 0, 0.018]));
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const bolt = disc(0.011, 0.012, materials.fastener, 14);
      bolt.position.set(sx * 0.098, sy * 0.098, 0.024);
      armRoot.add(bolt);
    }
  }

  // J1 — yaw turret, vertical axis. Local z = 0 is the bench top, so the mast
  // height below is exactly the kinematic offset of the shoulder hinge.
  const yawGroup = new THREE.Group();
  armRoot.add(yawGroup);

  const turret = disc(0.082, 0.03, materials.shell, 40, 0.076);
  turret.position.z = 0.031;
  yawGroup.add(turret);
  const turretTrim = disc(0.085, 0.012, materials.accent, 40);
  turretTrim.position.z = 0.05;
  yawGroup.add(turretTrim);
  const turretCollar = disc(0.062, 0.05, materials.housing, 34);
  turretCollar.position.z = 0.075;
  yawGroup.add(turretCollar);

  // Mast: the two-plate tower that carries the shoulder hinge.
  const mastHeight = ARM.mast;
  const plateFrom = 0.062;
  const plateHeight = mastHeight - plateFrom;
  for (const sy of [-1, 1]) {
    const plate = box(0.078, 0.016, plateHeight, materials.shellLight, [0, sy * 0.043, plateFrom + plateHeight / 2]);
    yawGroup.add(plate);
    // Rounded top corner where the pin sits.
    const boss = hinge(0.026, 0.016, materials.shellLight, 22);
    boss.position.set(0, sy * 0.043, mastHeight);
    yawGroup.add(boss);
  }
  yawGroup.add(box(0.06, 0.086, 0.024, materials.housing, [0, 0, plateFrom + 0.05]));
  yawGroup.add(box(0.062, 0.086, 0.02, materials.housing, [0, 0, mastHeight - 0.052]));

  // J2 — shoulder hinge, horizontal axis along the local Y direction.
  const pitchGroup = new THREE.Group();
  pitchGroup.position.z = mastHeight;
  yawGroup.add(pitchGroup);

  const shoulderPin = hinge(0.019, 0.13, materials.fastener, 24);
  pitchGroup.add(shoulderPin);
  const shoulderDrum = hinge(0.042, 0.07, materials.housing, 30);
  pitchGroup.add(shoulderDrum);
  for (const sy of [-1, 1]) {
    const cap = hinge(0.036, 0.01, materials.accent, 26);
    cap.position.y = sy * 0.037;
    pitchGroup.add(cap);
  }

  // Upper arm: a milled link from the shoulder hinge to the elbow hinge.
  const upperLength = ARM.upperArm;
  const upperBody = box(upperLength * 0.82, 0.05, 0.062, materials.shell, [upperLength * 0.47, 0, 0]);
  pitchGroup.add(upperBody);
  pitchGroup.add(box(upperLength * 0.6, 0.062, 0.03, materials.housing, [upperLength * 0.44, 0, 0]));
  // Lightening slot, drawn as a slightly inset dark panel on both side faces.
  for (const sy of [-1, 1]) {
    pitchGroup.add(box(upperLength * 0.46, 0.004, 0.028, materials.dark, [upperLength * 0.46, sy * 0.0255, 0]));
  }
  const upperBand = box(0.026, 0.058, 0.07, materials.accent, [upperLength * 0.8, 0, 0]);
  pitchGroup.add(upperBand);

  // J3 — elbow hinge.
  const elbowGroup = new THREE.Group();
  elbowGroup.position.x = upperLength;
  pitchGroup.add(elbowGroup);

  const elbowPin = hinge(0.017, 0.115, materials.fastener, 24);
  elbowGroup.add(elbowPin);
  elbowGroup.add(hinge(0.04, 0.066, materials.housing, 30));
  for (const sy of [-1, 1]) {
    const cap = hinge(0.032, 0.009, materials.accentSoft, 24);
    cap.position.y = sy * 0.033;
    elbowGroup.add(cap);
  }

  // Forearm: elbow hinge to the tool point, tool fixed to the end of it.
  const forearmLength = ARM.forearm;
  elbowGroup.add(box(forearmLength * 0.84, 0.042, 0.05, materials.shellLight, [forearmLength * 0.48, 0, 0]));
  elbowGroup.add(box(forearmLength * 0.6, 0.052, 0.026, materials.housing, [forearmLength * 0.44, 0, 0]));
  elbowGroup.add(box(0.022, 0.05, 0.058, materials.accentSoft, [forearmLength * 0.74, 0, 0]));

  // Tool: a stubby gripper whose tip is exactly the modelled tool point.
  const toolGroup = new THREE.Group();
  toolGroup.position.x = forearmLength;
  elbowGroup.add(toolGroup);
  toolGroup.add(box(0.05, 0.044, 0.05, materials.housing, [-0.026, 0, 0]));
  toolGroup.add(box(0.016, 0.052, 0.058, materials.accentSoft, [-0.004, 0, 0]));
  for (const sy of [-1, 1]) {
    toolGroup.add(box(0.05, 0.01, 0.012, materials.shellLight, [-0.028, sy * 0.019, 0.019]));
    toolGroup.add(box(0.05, 0.01, 0.012, materials.shellLight, [-0.028, sy * 0.019, -0.019]));
  }
  const tipDot = new THREE.Mesh(new THREE.SphereGeometry(0.011, 16, 12), materials.teal);
  toolGroup.add(tipDot);

  /* ------------------------------------------------------------------ *
   * Frames, labels, goal
   * ------------------------------------------------------------------ */
  const frameLayer = new THREE.Group();
  scene.add(frameLayer);
  const frameAxes = {
    yaw: new THREE.AxesHelper(0.13),
    shoulder: new THREE.AxesHelper(0.16),
    elbow: new THREE.AxesHelper(0.13),
    tip: new THREE.AxesHelper(0.1),
  };
  Object.values(frameAxes).forEach((helper) => {
    helper.material.transparent = true;
    helper.material.opacity = 0.85;
    frameLayer.add(helper);
  });

  const jointElements = JOINTS.map((_, index) => {
    const element = document.createElement('div');
    element.className = 'joint-label';
    element.innerHTML = `<b>J${index + 1}</b> &nbsp; 0°`;
    const label = new CSS2DObject(element);
    scene.add(label);
    return { label, element };
  });
  const tipElement = document.createElement('div');
  tipElement.className = 'joint-label tip-label';
  tipElement.innerHTML = '<b>TCP</b> &nbsp; z 0.00';
  const tipLabel = new CSS2DObject(tipElement);
  scene.add(tipLabel);

  const goalMarker = new THREE.Group();
  scene.add(goalMarker);
  const goalRing = new THREE.Mesh(
    new THREE.RingGeometry(0.03, 0.042, 44),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.95, side: THREE.DoubleSide }),
  );
  goalMarker.add(goalRing);
  const goalDot = new THREE.Mesh(
    new THREE.CircleGeometry(0.01, 20),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.95 }),
  );
  goalMarker.add(goalDot);
  const goalCross = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-0.075, 0, 0), new THREE.Vector3(0.075, 0, 0),
      new THREE.Vector3(0, -0.075, 0), new THREE.Vector3(0, 0.075, 0),
    ]),
    new THREE.LineBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.55 }),
  );
  goalMarker.add(goalCross);
  const goalStem = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0.18),
      new THREE.Vector3(0.05, 0, 0.18), new THREE.Vector3(0.05, 0, 0.2),
    ]),
    new THREE.LineBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.5 }),
  );
  goalMarker.add(goalStem);
  goalMarker.position.z = BENCH.height + 0.003;

  /* ------------------------------------------------------------------ *
   * State
   * ------------------------------------------------------------------ */
  let angles = JOINTS.map(({ home }) => home);
  let mode = 'fk';
  let goal = { x: 0.45, y: 0.12 };
  let goalClamped = false;
  let goalNote = '';
  let animationEnabled = false;
  let animationStart = 0;
  let lastAnimationUpdate = 0;
  const controlRefs = { input: [], angleInput: [] };

  /* ------------------------------------------------------------------ *
   * Panel: joint controls
   * ------------------------------------------------------------------ */
  function buildJointControls() {
    $('#s-sliders').innerHTML = JOINTS.map((joint, index) => `
      <div class="joint-control">
        <div class="joint-topline">
          <span class="joint-index">${String(index + 1).padStart(2, '0')}</span>
          <div class="joint-copy"><span class="joint-name">${joint.name}</span><span class="joint-axis">${joint.axis}</span></div>
          <label class="joint-angle-wrap">
            <input class="joint-angle-input" id="s-angle-${index}" type="number" min="${joint.min}" max="${joint.max}" step="0.5" value="${joint.home.toFixed(1)}" aria-label="Joint ${index + 1} angle in degrees" />
            <span>°</span>
          </label>
        </div>
        <input class="joint-range" id="s-slider-${index}" type="range" min="${joint.min}" max="${joint.max}" step="0.5" value="${joint.home}" aria-label="Joint ${index + 1}, ${joint.name}, angle in degrees" />
        <div class="range-limits"><span>${joint.min}°</span><span>${joint.max}°</span></div>
        <p class="joint-hint">${joint.hint}</p>
      </div>
    `).join('');

    JOINTS.forEach((_, index) => {
      const input = $(`#s-slider-${index}`);
      const angleInput = $(`#s-angle-${index}`);
      controlRefs.input[index] = input;
      controlRefs.angleInput[index] = angleInput;

      input.addEventListener('input', () => {
        stopDemo();
        setMode('fk', { silent: true });
        angles[index] = Number(input.value);
        angleInput.value = angles[index].toFixed(1);
        refresh();
      });
      angleInput.addEventListener('change', () => {
        stopDemo();
        setMode('fk', { silent: true });
        const typed = angleInput.value.trim() === '' ? angles[index] : Number(angleInput.value);
        angles[index] = Number.isFinite(typed) ? clamp(typed, JOINTS[index].min, JOINTS[index].max) : angles[index];
        input.value = angles[index].toFixed(1);
        angleInput.value = angles[index].toFixed(1);
        refresh();
      });
      angleInput.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') angleInput.blur();
      });
    });
  }

  function syncJointInputs() {
    angles.forEach((angle, index) => {
      const formatted = angle.toFixed(1);
      controlRefs.input[index].value = formatted;
      controlRefs.angleInput[index].value = formatted;
    });
  }

  function syncGoalInputs() {
    $('#s-goal-x').value = goal.x.toFixed(3);
    $('#s-goal-y').value = goal.y.toFixed(3);
  }

  /* ------------------------------------------------------------------ *
   * Panel: static tables
   * ------------------------------------------------------------------ */
  function buildStaticTables() {
    const linkRows = [
      ['1', ARM.mast, 'Bench top → shoulder hinge', 'mast'],
      ['2', ARM.upperArm, 'Shoulder hinge → elbow hinge', 'upper arm'],
      ['3', ARM.forearm, 'Elbow hinge → tool point', 'forearm'],
    ];
    $('#s-link-table tbody').innerHTML = linkRows.map(([index, length, use, note]) => `
      <tr><td>${index}</td><td>${length.toFixed(2)}</td><td>${use}</td><td>${note}</td></tr>
    `).join('');

    $('#s-frames').innerHTML = [
      ['BASE', 'bench top, on the yaw axis'],
      ['J1', 'yaw axis'],
      ['J2', 'shoulder hinge'],
      ['J3', 'elbow hinge'],
      ['TCP', 'tool point'],
    ].map(([name, note], index) => `
      <div class="frame-row"><span>${name}</span><code id="s-frame-${index}">0.000, 0.000, 0.000</code></div>
    `).join('');

    $('#s-jacobian tbody').innerHTML = Array.from({ length: 3 }, (_, row) =>
      '<tr>' + Array.from({ length: 3 }, (_, column) => `<td id="s-jac-${row}-${column}">0.000</td>`).join('') + '</tr>'
    ).join('');
  }

  /* ------------------------------------------------------------------ *
   * Mode & solving
   * ------------------------------------------------------------------ */
  function setMode(next, { silent = false } = {}) {
    const changing = mode !== next;
    mode = next;
    if (changing && mode === 'ik') {
      // Start from where the tool already is so the arm does not jump.
      const pose = solveForward(angles);
      goal = { x: pose.tip.x, y: pose.tip.y };
      goalClamped = false;
      goalNote = '';
      syncGoalInputs();
    }
    updateModeUI();
    if (!silent) refresh();
  }

  function updateModeUI() {
    root.querySelectorAll('[data-spatial-mode]').forEach((button) => {
      const selected = button.dataset.spatialMode === mode;
      button.setAttribute('aria-pressed', String(selected));
      button.classList.toggle('is-active', selected);
    });
    $('#s-goal-fields').classList.toggle('is-disabled', mode !== 'ik');
    $('#s-mode-note').textContent = mode === 'ik'
      ? 'Type an X and Y on the bench, or click the bench surface. Yaw aims at the point, then the two hinges fold to reach it.'
      : 'Move a joint slider, or click the bench to switch to goal control.';
    if (canvas) canvas.style.cursor = mode === 'ik' ? 'crosshair' : 'grab';
  }

  /** Keep a solved joint on the same revolution as the current value, then limit it. */
  function unwrapNear(value, reference, limit) {
    let unwrapped = value;
    while (unwrapped - reference > 180) unwrapped -= 360;
    while (reference - unwrapped > 180) unwrapped += 360;
    const limited = clamp(unwrapped, limit.min, limit.max);
    return { value: limited, clamped: limited !== unwrapped };
  }

  function solveGoal() {
    const solution = solveInverse({ x: goal.x, y: goal.y, z: BENCH.height }, { clamp: true });
    if (!solution.ok) {
      goalClamped = true;
      goalNote = solution.reason ?? 'Not reachable';
      return;
    }
    let clamped = solution.clamped;
    angles = solution.angles.map((angle, index) => {
      const result = unwrapNear(angle, angles[index], JOINTS[index]);
      if (result.clamped) clamped = true;
      return result.value;
    });
    goalClamped = clamped;
    goalNote = clamped ? 'Moved to the nearest pose the joints can hold.' : '';
    syncJointInputs();
  }

  /* ------------------------------------------------------------------ *
   * Scene refresh
   * ------------------------------------------------------------------ */
  function updateScene() {
    const pose = solveForward(angles);
    const [yaw, pitch, elbow] = angles;
    const yawRad = yaw * DEG;

    yawGroup.rotation.z = yawRad;
    pitchGroup.rotation.y = -pitch * DEG;
    elbowGroup.rotation.y = elbow * DEG;

    // Frame triads, placed in world space from the kinematics.
    const yawQuaternion = new THREE.Quaternion().setFromAxisAngle(UP, yawRad);
    const hingeQuaternion = (deg) => new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 1, 0), -deg * DEG,
    );
    frameAxes.yaw.position.set(0, 0, BENCH.height + 0.02);
    frameAxes.yaw.quaternion.copy(yawQuaternion);
    frameAxes.shoulder.position.set(pose.shoulder.x, pose.shoulder.y, pose.shoulder.z);
    frameAxes.shoulder.quaternion.copy(yawQuaternion).multiply(hingeQuaternion(pitch));
    frameAxes.elbow.position.set(pose.elbow.x, pose.elbow.y, pose.elbow.z);
    frameAxes.elbow.quaternion.copy(yawQuaternion).multiply(hingeQuaternion(pitch - elbow));
    frameAxes.tip.position.set(pose.tip.x, pose.tip.y, pose.tip.z);
    frameAxes.tip.quaternion.copy(yawQuaternion).multiply(hingeQuaternion(pitch - elbow));

    const labelPositions = [
      new THREE.Vector3(0, 0, BENCH.height + 0.11),
      new THREE.Vector3(pose.shoulder.x, pose.shoulder.y, pose.shoulder.z + 0.08),
      new THREE.Vector3(pose.elbow.x, pose.elbow.y, pose.elbow.z + 0.075),
    ];
    jointElements.forEach(({ label, element }, index) => {
      label.position.copy(labelPositions[index]);
      element.innerHTML = `<b>J${index + 1}</b> &nbsp; ${angles[index].toFixed(0)}°`;
    });
    tipLabel.position.set(pose.tip.x, pose.tip.y, pose.tip.z + 0.03);
    tipElement.innerHTML = `<b>TCP</b> &nbsp; z ${fmt(pose.tip.z, 2)}`;

    goalMarker.visible = mode === 'ik';
    goalMarker.position.set(goal.x, goal.y, BENCH.height + 0.003);
    const goalColor = goalClamped ? 0xff9d67 : 0x7ff0cf;
    goalRing.material.color.setHex(goalColor);
    goalDot.material.color.setHex(goalColor);
    goalCross.material.color.setHex(goalColor);
    goalStem.material.color.setHex(goalColor);

    return pose;
  }

  function updateReadouts(pose) {
    const { matrix, determinant, dexterity } = jacobian(angles);
    const benchState = onBench(pose.tip.z)
      ? { text: 'ON THE BENCH', cls: 'chip-ok' }
      : pose.tip.z > BENCH.height
        ? { text: 'ABOVE THE BENCH', cls: 'chip-neutral' }
        : { text: 'BELOW THE BENCH', cls: 'chip-warn' };

    $('#s-pos-x').textContent = fmt(pose.tip.x);
    $('#s-pos-y').textContent = fmt(pose.tip.y);
    $('#s-pos-z').textContent = fmt(pose.tip.z);
    $('#s-viewport-position').innerHTML = `X ${fmt(pose.tip.x)} <em>Y ${fmt(pose.tip.y)}</em> Z ${fmt(pose.tip.z)}`;

    $('#s-radial').textContent = `${pose.radial.toFixed(3)} m`;
    $('#s-elbow-height').textContent = `${pose.elbowAboveBench >= 0 ? '+' : '−'}${Math.abs(pose.elbowAboveBench).toFixed(3)} m`;
    $('#s-forearm-angle').textContent = `${pose.forearmElevationDeg.toFixed(1)}°`;
    $('#s-height').textContent = `${pose.heightAboveBench >= 0 ? '+' : '−'}${Math.abs(pose.heightAboveBench).toFixed(3)} m`;
    $('#s-yaw').textContent = `${pose.yawDeg.toFixed(1)}°`;
    $('#s-pitch').textContent = `${angles[1].toFixed(1)}°`;
    $('#s-elbow-angle').textContent = `${angles[2].toFixed(1)}°`;

    const benchChip = $('#s-bench-state');
    benchChip.textContent = benchState.text;
    benchChip.className = `status-chip ${benchState.cls}`;

    const singular = dexterity < 0.01;
    const chip = $('#s-dexterity-status');
    chip.textContent = singular ? 'SINGULAR' : 'DEXTEROUS';
    chip.className = `status-chip ${singular ? 'chip-warn' : 'chip-ok'}`;
    $('#s-dexterity').textContent = dexterity.toFixed(4);
    $('#s-determinant').textContent = determinant.toFixed(4);

    const status = $('#s-reach-status');
    if (mode === 'ik') {
      status.textContent = goalClamped ? 'NEAREST POSE' : 'GOAL IN REACH';
      status.className = `status-chip ${goalClamped ? 'chip-warn' : 'chip-ok'}`;
    } else {
      status.textContent = 'JOINT-SPACE CONTROL';
      status.className = 'status-chip chip-neutral';
    }
    $('#s-goal-note').textContent = mode === 'ik'
      ? (goalNote || `Goal at ${Math.hypot(goal.x, goal.y).toFixed(2)} m from the yaw axis, bearing ${(Math.atan2(goal.y, goal.x) * 180 / Math.PI).toFixed(0)}°.`)
      : 'Switch to goal control to drive the tool point to a spot on the bench.';

    for (let row = 0; row < 3; row += 1) {
      for (let column = 0; column < 3; column += 1) {
        $(`#s-jac-${row}-${column}`).textContent = matrix[row][column].toFixed(3);
      }
    }

    const frames = $('#s-frames').children;
    const framePoints = [
      new THREE.Vector3(0, 0, BENCH.height),
      new THREE.Vector3(0, 0, BENCH.height + ARM.mast),
      new THREE.Vector3(pose.elbow.x, pose.elbow.y, pose.elbow.z),
      new THREE.Vector3(pose.tip.x, pose.tip.y, pose.tip.z),
    ];
    framePoints.forEach((point, index) => {
      if (frames[index]) frames[index].lastElementChild.textContent = `${fmt(point.x)}, ${fmt(point.y)}, ${fmt(point.z)}`;
    });

    const band = reachOnBench(BENCH.height);
    $('#s-bench-height').textContent = `${BENCH.height.toFixed(2)} m`;
    $('#s-band-inner').textContent = `${WORK_BAND.inner.toFixed(2)} m`;
    $('#s-band-outer').textContent = `${Math.min(WORK_BAND.outer, band.outer).toFixed(2)} m`;
    $('#s-shoulder-height').textContent = `${SHOULDER_Z.toFixed(2)} m`;
    $('#s-reach-limit').textContent = `${REACH.max.toFixed(2)} m`;
  }

  function refresh() {
    updateReadouts(updateScene());
  }

  /* ------------------------------------------------------------------ *
   * Actions
   * ------------------------------------------------------------------ */
  function stopDemo() {
    if (!animationEnabled) return;
    animationEnabled = false;
    $('#s-animate').checked = false;
  }

  function goHome() {
    stopDemo();
    angles = JOINTS.map(({ home }) => home);
    goalClamped = false;
    goalNote = '';
    if (mode === 'ik') {
      const pose = solveForward(angles);
      goal = { x: pose.tip.x, y: pose.tip.y };
      solveGoal();
      syncGoalInputs();
    }
    syncJointInputs();
    refresh();
  }

  function goRandom() {
    stopDemo();
    let best = null;
    for (let attempt = 0; attempt < 18; attempt += 1) {
      const candidate = [
        THREE.MathUtils.randFloat(JOINTS[0].min * 0.8, JOINTS[0].max * 0.8),
        THREE.MathUtils.randFloat(20, 70),
        THREE.MathUtils.randFloat(60, 150),
      ];
      const pose = solveForward(candidate);
      // Prefer poses with the elbow up and the tool clear of the bench.
      const score = pose.elbowAboveBench * 2 - Math.abs(pose.heightAboveBench - 0.1);
      if (!best || score > best.score) best = { candidate, score };
    }
    angles = best.candidate;
    goalClamped = false;
    goalNote = '';
    if (mode === 'ik') {
      const pose = solveForward(angles);
      goal = { x: pose.tip.x, y: pose.tip.y };
      syncGoalInputs();
    }
    syncJointInputs();
    refresh();
  }

  /** Demo tour: sweep the goal around the bench, then in and out. */
  function stepDemo(time) {
    const t = (time - animationStart) / 1000;
    if (mode !== 'ik') setMode('ik', { silent: true });
    const bearing = (-55 + 55 * Math.sin(t * 0.42)) * DEG;
    const radius = 0.52 + 0.22 * Math.sin(t * 0.29 + 0.7);
    goal = { x: Math.cos(bearing) * radius, y: Math.sin(bearing) * radius };
    goalClamped = false;
    goalNote = '';
    solveGoal();
    syncGoalInputs();
    refresh();
    lastAnimationUpdate = time;
  }

  /* ------------------------------------------------------------------ *
   * Pointer: drop the goal on the bench
   * ------------------------------------------------------------------ */
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();

  function benchPointAt(event) {
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    pointer.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.setFromCamera(pointer, camera);
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -BENCH.height);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(plane, hit)) return null;
    if (!overBench(hit.x, hit.y, 0.02)) return null;
    return hit;
  }

  // A click drops the goal on the bench; a drag orbits. Distinguish by travel.
  let pressPoint = null;
  canvas?.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) return;
    pressPoint = { x: event.clientX, y: event.clientY };
  });
  canvas?.addEventListener('pointerup', (event) => {
    if (!pressPoint || event.button !== 0) return;
    const travel = Math.hypot(event.clientX - pressPoint.x, event.clientY - pressPoint.y);
    pressPoint = null;
    if (travel > 5) return; // that was an orbit, not a click
    const hit = benchPointAt(event);
    if (!hit) return;
    stopDemo();
    mode = 'ik';
    updateModeUI();
    goal = { x: hit.x, y: hit.y };
    solveGoal();
    syncGoalInputs();
    refresh();
  });
  canvas?.addEventListener('contextmenu', (event) => event.preventDefault());

  /* ------------------------------------------------------------------ *
   * Camera views
   * ------------------------------------------------------------------ */
  function syncViewButtons() {
    root.querySelectorAll('[data-spatial-view]').forEach((button) => {
      const selected = button.dataset.spatialView === activeView;
      button.setAttribute('aria-pressed', String(selected));
      button.classList.toggle('is-active', selected);
    });
  }

  /** The camera pose for the side view: square on to the arm's current heading. */
  function sideViewGoal() {
    const perpendicular = angles[0] * DEG + Math.PI / 2;
    const distance = views.bench.sideDistance;
    camGoal.target.copy(benchCenter).add(new THREE.Vector3(0, 0, 0.16));
    camGoal.eye.set(
      camGoal.target.x + Math.cos(perpendicular) * distance,
      camGoal.target.y + Math.sin(perpendicular) * distance,
      camGoal.target.z + views.bench.sideHeight,
    );
  }

  function selectView(name) {
    activeView = name;
    if (name === 'side') {
      sideViewGoal();
    } else {
      const view = views[name] ?? views.bench;
      camGoal.eye.copy(view.eye);
      camGoal.target.copy(view.target);
    }
    syncingView = true;
    syncViewButtons();
  }

  root.querySelectorAll('[data-spatial-view]').forEach((button) => {
    button.addEventListener('click', () => {
      stopDemo();
      selectView(button.dataset.spatialView);
    });
  });

  /* ------------------------------------------------------------------ *
   * Panel wiring
   * ------------------------------------------------------------------ */
  root.querySelectorAll('[data-spatial-mode]').forEach((button) => {
    button.addEventListener('click', () => {
      stopDemo();
      setMode(button.dataset.spatialMode);
    });
  });

  ['x', 'y'].forEach((key) => {
    const input = $(`#s-goal-${key}`);
    input.addEventListener('change', () => {
      const value = Number(input.value);
      if (!Number.isFinite(value)) { syncGoalInputs(); return; }
      stopDemo();
      if (mode !== 'ik') setMode('ik', { silent: true });
      goal = { ...goal, [key]: value };
      solveGoal();
      syncGoalInputs();
      refresh();
    });
    input.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') input.blur();
    });
  });

  $('#s-copy-pose').addEventListener('click', () => {
    const pose = solveForward(angles);
    if (mode !== 'ik') setMode('ik', { silent: true });
    goal = { x: pose.tip.x, y: pose.tip.y };
    goalClamped = false;
    goalNote = '';
    syncGoalInputs();
    updateModeUI();
    refresh();
  });

  $('#s-centre-goal').addEventListener('click', () => {
    stopDemo();
    if (mode !== 'ik') setMode('ik', { silent: true });
    goal = { x: 0.42, y: 0 };
    solveGoal();
    syncGoalInputs();
    refresh();
  });

  $('#s-reset').addEventListener('click', goHome);
  $('#s-random').addEventListener('click', goRandom);

  $('#s-show-rings').addEventListener('change', (event) => { workspaceLayer.visible = event.target.checked; });
  $('#s-show-frames').addEventListener('change', (event) => {
    frameLayer.visible = event.target.checked;
    jointElements.forEach(({ label }) => { label.visible = event.target.checked; });
  });
  $('#s-animate').addEventListener('change', (event) => {
    animationEnabled = event.target.checked;
    if (animationEnabled) animationStart = performance.now();
  });

  /* ------------------------------------------------------------------ *
   * Sizing, camera, loop
   * ------------------------------------------------------------------ */
  function resize() {
    const width = Math.max(1, viewport.clientWidth);
    const height = Math.max(1, viewport.clientHeight);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer?.setSize(width, height, false);
    labelRenderer.setSize(width, height);
  }

  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(viewport);
  window.addEventListener('resize', resize, { passive: true });

  function updateCamera(delta) {
    if (!controls) return;
    if (activeView === 'side') {
      // Track the yaw so the A-frame stays in profile as the base turns.
      sideViewGoal();
      syncingView = true;
    }
    if (syncingView) {
      const lerp = Math.min(1, delta * 4.5);
      camState.eye.lerp(camGoal.eye, lerp);
      camState.target.lerp(camGoal.target, lerp);
      camera.position.copy(camState.eye);
      controls.target.copy(camState.target);
      if (camState.eye.distanceTo(camGoal.eye) < 0.02) syncingView = false;
    } else {
      camState.eye.copy(camera.position);
      camState.target.copy(controls.target);
    }
    controls.update();
  }

  /* ------------------------------------------------------------------ *
   * Boot
   * ------------------------------------------------------------------ */
  buildJointControls();
  buildStaticTables();
  const startPose = solveForward(angles);
  goal = { x: startPose.tip.x, y: startPose.tip.y };
  syncGoalInputs();
  updateModeUI();
  syncViewButtons();
  resize();
  refresh();

  let previousTime = performance.now();
  function tick(time) {
    if (!renderer) return;
    const delta = Math.min(0.05, (time - previousTime) / 1000);
    previousTime = time;
    if (animationEnabled && time - lastAnimationUpdate > 30) stepDemo(time);
    updateCamera(delta);
    renderer.render(scene, camera);
    labelRenderer.render(scene, camera);
  }

  return {
    resize,
    tick,
    onActivate() {
      resize();
      refresh();
    },
    /**
     * Scene handles for tests and debugging. The model is a real hinge chain, so
     * a test can read the world-space tool point straight out of the scene graph
     * and compare it with the kinematics.
     */
    scene: {
      root: scene,
      benchGroup,
      yawGroup,
      pitchGroup,
      elbowGroup,
      toolGroup,
      get angles() { return [...angles]; },
      get goal() { return { ...goal }; },
      get mode() { return mode; },
    },
  };
}
