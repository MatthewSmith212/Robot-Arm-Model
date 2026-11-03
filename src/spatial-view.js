import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DObject, CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';
import {
  ARM,
  ELBOW_TO_TIP,
  JOINTS,
  PLANES,
  REACH,
  jacobian,
  reachOnPlane,
  solveForward,
  solveInverse,
  touchingPlane,
  wrapDegrees,
} from './spatial-kinematics.js';
import { SHADOW_MAP_TYPE, createRenderer, showRendererFallback } from './webgl.js';

const DEG = Math.PI / 180;
const clamp = THREE.MathUtils.clamp;
const fmt = (value, digits = 3) => (Math.abs(value) < 0.0005 ? 0 : value).toFixed(digits);
const UP = new THREE.Vector3(0, 0, 1);

const JOINT_KEYS = ['yaw', 'pitch', 'elbow'];
const BASE_POSITION = new THREE.Vector3(0, 0, 0);

/**
 * Mount the spatial shoulder–elbow arm viewer inside `root` and return its
 * frame hooks. The arm is bolted to the upper bench (plane 1) and reaches down
 * onto the lower bench (plane 2), matching the two-tier setup.
 */
export function createSpatialView(root) {
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
    renderer.domElement.setAttribute('aria-label', '3D model of a three degree of freedom arm with a two axis shoulder and a raised elbow, mounted on an upper bench and reaching down to a lower bench');
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

  const views = {
    iso: { eye: new THREE.Vector3(2.15, -2.35, 1.85), target: new THREE.Vector3(0.22, -0.04, 0.72), zoom: 1 },
    side: { eye: new THREE.Vector3(3.3, -0.28, 0.92), target: new THREE.Vector3(0.1, -0.05, 0.8), zoom: 1 },
    top: { eye: new THREE.Vector3(0.22, -0.02, 3.6), target: new THREE.Vector3(0.22, -0.04, 0.4), zoom: 1 },
  };
  const camState = { eye: views.iso.eye.clone(), target: views.iso.target.clone(), zoom: 1 };
  const camGoal = { eye: views.iso.eye.clone(), target: views.iso.target.clone(), zoom: 1 };
  let activeView = 'iso';

  const controls = renderer ? new OrbitControls(camera, renderer.domElement) : null;
  if (controls) {
    camera.position.copy(camState.eye);
    controls.target.copy(camState.target);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 1.4;
    controls.maxDistance = 6.5;
    controls.maxPolarAngle = Math.PI * 0.495;
    controls.addEventListener('start', () => { syncingView = false; stopDemo(); });
    controls.update();
  }
  let syncingView = false;

  /* ------------------------------------------------------------------ *
   * Lights & floor
   * ------------------------------------------------------------------ */
  scene.add(new THREE.HemisphereLight(0xe4f7f2, 0x14202a, 1.7));
  const keyLight = new THREE.DirectionalLight(0xfff2df, 2.6);
  keyLight.position.set(-2.4, -2.8, 4.6);
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
  const rimLight = new THREE.DirectionalLight(0x58caba, 1.5);
  rimLight.position.set(2.6, 2.4, 2.4);
  scene.add(rimLight);

  function makeFloorTexture() {
    const size = 1024;
    const span = 5;
    const canvasTexture = document.createElement('canvas');
    canvasTexture.width = size;
    canvasTexture.height = size;
    const ctx = canvasTexture.getContext('2d');
    const px = size / span;
    ctx.fillStyle = '#0a141b';
    ctx.fillRect(0, 0, size, size);
    const glow = ctx.createRadialGradient(size / 2, size / 2, 60, size / 2, size / 2, 700);
    glow.addColorStop(0, 'rgba(32, 74, 78, 0.45)');
    glow.addColorStop(0.6, 'rgba(16, 36, 44, 0.22)');
    glow.addColorStop(1, 'rgba(8, 16, 22, 0)');
    ctx.fillStyle = glow;
    ctx.fillRect(0, 0, size, size);
    const half = span / 2;
    for (let step = 0; step <= span * 10; step += 1) {
      const metres = -half + step * 0.1;
      const position = (metres + half) * px;
      const major = Math.abs(Math.abs(metres) % 0.5) < 1e-6;
      ctx.strokeStyle = major ? 'rgba(118, 165, 172, 0.26)' : 'rgba(92, 132, 141, 0.12)';
      ctx.lineWidth = major ? 1.7 : 1;
      ctx.beginPath(); ctx.moveTo(position, 0); ctx.lineTo(position, size); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, position); ctx.lineTo(size, position); ctx.stroke();
    }
    const texture = new THREE.CanvasTexture(canvasTexture);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = renderer ? renderer.capabilities.getMaxAnisotropy() : 1;
    return texture;
  }

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(5, 5),
    new THREE.MeshStandardMaterial({ map: makeFloorTexture(), roughness: 0.95, metalness: 0.05 }),
  );
  floor.receiveShadow = true;
  scene.add(floor);

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

  function beamBetween(start, end, radius, material, segments = 22) {
    const direction = new THREE.Vector3().subVectors(end, start);
    const length = direction.length();
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius * 1.08, 1, segments, 1), material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.position.copy(start).add(end).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
    mesh.scale.set(1, length, 1);
    return mesh;
  }

  function box(width, height, depth, material, position) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), material);
    if (position) mesh.position.set(...position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  /** Cylinder whose axis is the vertical Z axis (shared axis of the two shoulder joints). */
  function disc(radius, height, material, segments = 34) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, height, segments, 1), material);
    mesh.rotation.x = Math.PI / 2;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  /* ------------------------------------------------------------------ *
   * Two-tier bench: plane 1 (mounting) and plane 2 (lower target)
   * ------------------------------------------------------------------ */
  const deck = { width: 1.7, depth: 1.15, lowerWidth: 1.15, lowerDepth: 0.85 };
  const mountY = -0.02;
  const lowerY = -1.03;

  function buildPlane({ height, x, width, depth, name, highlight }) {
    const group = new THREE.Group();
    group.position.set(x, 0, 0);
    const top = new THREE.Mesh(new THREE.BoxGeometry(width, depth, 0.035), materials.benchTop);
    top.position.z = height - 0.0175;
    top.receiveShadow = true;
    top.castShadow = true;
    group.add(top);

    const lip = new THREE.Mesh(new THREE.BoxGeometry(width + 0.03, depth + 0.03, 0.012), materials.benchEdge);
    lip.position.z = height - 0.04;
    group.add(lip);

    // Simple welded legs with a foot plate, so the decks read as real benches.
    const legHeight = height - 0.04;
    for (const sx of [-1, 1]) {
      for (const sy of [-1, 1]) {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.045, legHeight), materials.bench);
        leg.position.set(sx * (width / 2 - 0.09), sy * (depth / 2 - 0.09), legHeight / 2);
        leg.castShadow = true;
        leg.receiveShadow = true;
        group.add(leg);
        const foot = box(0.11, 0.11, 0.012, materials.benchEdge, [sx * (width / 2 - 0.09), sy * (depth / 2 - 0.09), 0.006]);
        group.add(foot);
      }
      const rail = new THREE.Mesh(new THREE.BoxGeometry(0.03, depth - 0.12, 0.03), materials.bench);
      rail.position.set(sx * (width / 2 - 0.09), 0, legHeight * 0.28);
      group.add(rail);
    }

    if (highlight) {
      const ring = new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(-width / 2, -depth / 2, 0.001),
          new THREE.Vector3(width / 2, -depth / 2, 0.001),
          new THREE.Vector3(width / 2, depth / 2, 0.001),
          new THREE.Vector3(-width / 2, depth / 2, 0.001),
        ]),
        new THREE.LineBasicMaterial({ color: 0x5ee2bd, transparent: true, opacity: 0.35 }),
      );
      ring.position.z = height;
      group.add(ring);
    }

    // Plane label floats at the front corner of the deck.
    const element = document.createElement('div');
    element.className = 'plane-tag';
    element.innerHTML = `<b>${name.badge}</b> ${name.label} <span>z ${height.toFixed(2)} m</span>`;
    const label = new CSS2DObject(element);
    label.position.set(-width / 2 - 0.16, depth / 2 + 0.1, height - 0.1);
    group.add(label);

    scene.add(group);
    return group;
  }

  buildPlane({ height: PLANES.mount.height, x: mountY, width: deck.width, depth: deck.depth, name: { badge: 'P1', label: 'MOUNTING PLANE' }, highlight: true });
  buildPlane({ height: PLANES.lower.height, x: lowerY, width: deck.lowerWidth, depth: deck.lowerDepth, name: { badge: 'P2', label: 'LOWER PLANE' }, highlight: true });

  // Reachability annuli drawn flat on each deck.
  const workspaceLayer = new THREE.Group();
  scene.add(workspaceLayer);

  function annulus(plane) {
    const { inner, outer } = reachOnPlane(plane.height);
    const points = [];
    if (inner > 0.001) {
      for (let i = 0; i < 96; i += 1) {
        const a = (i / 96) * Math.PI * 2;
        points.push(new THREE.Vector3(Math.cos(a) * inner, Math.sin(a) * inner, 0.002));
        points.push(new THREE.Vector3(Math.cos(a) * outer, Math.sin(a) * outer, 0.002));
      }
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const line = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: 0x66ddc0, transparent: true, opacity: 0.22 }));
      line.position.z = plane.height;
      return line;
    }
    const ring = [];
    for (let i = 0; i <= 128; i += 1) {
      const a = (i / 128) * Math.PI * 2;
      ring.push(new THREE.Vector3(Math.cos(a) * outer, Math.sin(a) * outer, 0.002));
    }
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(ring), new THREE.LineBasicMaterial({ color: 0x66ddc0, transparent: true, opacity: 0.3 }));
    line.position.z = plane.height;
    return line;
  }
  workspaceLayer.add(annulus(PLANES.mount), annulus(PLANES.lower));

  /* ------------------------------------------------------------------ *
   * The arm: pedestal, two-axis shoulder, upper arm, elbow, forearm, tool
   * ------------------------------------------------------------------ */
  const armRoot = new THREE.Group();
  scene.add(armRoot);

  // Pedestal bolted to the mounting plane.
  const pedestalBase = box(0.3, 0.3, 0.022, materials.housing, [0, 0, PLANES.mount.height + 0.011]);
  pedestalBase.position.x = mountY;
  armRoot.add(pedestalBase);
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const bolt = disc(0.011, 0.012, materials.fastener, 12);
      bolt.position.set(mountY + sx * 0.115, sy * 0.115, PLANES.mount.height + 0.026);
      armRoot.add(bolt);
    }
  }

  // Turret: rotates about Z (shoulder yaw).
  const yawGroup = new THREE.Group();
  yawGroup.position.set(mountY, 0, PLANES.mount.height);
  armRoot.add(yawGroup);

  const turret = disc(0.095, 0.085, materials.shell, 36);
  turret.position.z = 0.045;
  yawGroup.add(turret);
  const turretTrim = disc(0.099, 0.014, materials.accent, 36);
  turretTrim.position.z = 0.014;
  yawGroup.add(turretTrim);
  const turretCap = disc(0.06, 0.012, materials.dark, 30);
  turretCap.position.z = 0.094;
  yawGroup.add(turretCap);
  const yawMarker = box(0.11, 0.014, 0.01, materials.teal, [0.06, 0, 0.1]);
  yawGroup.add(yawMarker);

  // Yoke uprights carry the horizontal shoulder-pitch axis.
  const yokeTop = ARM.pedestal;
  for (const sx of [-1, 1]) {
    const plate = box(0.022, 0.11, yokeTop - 0.05, materials.shellLight, [sx * 0.072, 0, 0.05 + (yokeTop - 0.05) / 2]);
    yawGroup.add(plate);
  }
  const pitchAxis = disc(0.052, 0.17, materials.housing, 30);
  pitchAxis.rotation.z = Math.PI / 2;
  pitchAxis.position.z = yokeTop;
  yawGroup.add(pitchAxis);
  const pitchCaps = [-1, 1].map((sign) => {
    const cap = disc(0.044, 0.014, materials.accent, 26);
    cap.rotation.z = Math.PI / 2;
    cap.position.set(sign * 0.088, 0, yokeTop);
    yawGroup.add(cap);
    return cap;
  });

  // Shoulder pitch rotates the upper arm in the vertical plane the yaw points at.
  const pitchGroup = new THREE.Group();
  pitchGroup.position.set(mountY, 0, ARM.shoulderZ);
  armRoot.add(pitchGroup);

  const upperArm = beamBetween(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, -1),
    0.041,
    materials.shell,
  );
  upperArm.scale.set(1, ARM.upperArm, 1);
  upperArm.position.set(0, 0, -ARM.upperArm / 2);
  upperArm.quaternion.identity();
  pitchGroup.add(upperArm);
  const upperSpine = box(0.028, 0.05, ARM.upperArm * 0.72, materials.housing, [0, 0.03, -ARM.upperArm / 2]);
  pitchGroup.add(upperSpine);
  const upperBand = disc(0.05, 0.03, materials.accent, 24);
  upperBand.rotation.z = Math.PI / 2;
  upperBand.position.set(0, 0, -ARM.upperArm * 0.82);
  pitchGroup.add(upperBand);

  // Elbow housing at the end of the upper arm, then the forearm about the same axis.
  const elbowGroup = new THREE.Group();
  elbowGroup.position.set(mountY, 0, ARM.shoulderZ);
  armRoot.add(elbowGroup);

  const elbowBody = disc(0.058, 0.115, materials.housing, 30);
  elbowBody.rotation.y = Math.PI / 2;
  elbowGroup.add(elbowBody);
  const elbowRim = disc(0.062, 0.016, materials.accent, 30);
  elbowRim.rotation.y = Math.PI / 2;
  elbowRim.position.x = 0.058;
  elbowGroup.add(elbowRim);
  const elbowCap = disc(0.032, 0.012, materials.fastener, 22);
  elbowCap.rotation.y = Math.PI / 2;
  elbowCap.position.x = 0.068;
  elbowGroup.add(elbowCap);

  const forearmGroup = new THREE.Group();
  forearmGroup.position.set(mountY, 0, ARM.shoulderZ);
  armRoot.add(forearmGroup);

  const forearm = beamBetween(
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, -1),
    0.031,
    materials.shellLight,
  );
  forearm.scale.set(1, ELBOW_TO_TIP, 1);
  forearm.position.set(0, 0, -ELBOW_TO_TIP / 2);
  forearm.quaternion.identity();
  forearmGroup.add(forearm);
  const forearmSpine = box(0.022, 0.042, ELBOW_TO_TIP * 0.66, materials.housing, [0, 0.026, -ELBOW_TO_TIP * 0.46]);
  forearmGroup.add(forearmSpine);
  const wristBand = disc(0.038, 0.026, materials.accentSoft, 24);
  wristBand.rotation.z = Math.PI / 2;
  wristBand.position.set(0, 0, -ARM.forearm);

  forearmGroup.add(wristBand);

  // Gripper: two fingers, the tips of which are exactly the tool center point.
  const tool = new THREE.Group();
  tool.position.set(0, 0, -ARM.forearm);
  forearmGroup.add(tool);
  const toolPalm = box(0.062, 0.05, 0.03, materials.housing, [0, 0, -0.045]);
  tool.add(toolPalm);
  const toolPlate = box(0.07, 0.056, 0.012, materials.accentSoft, [0, 0, -0.016]);
  tool.add(toolPlate);
  for (const side of [-1, 1]) {
    tool.add(box(0.014, 0.02, 0.1, materials.shellLight, [side * 0.022, 0, -0.05]));
  }
  const toolTip = new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 12), materials.teal);
  toolTip.position.set(0, 0, -ARM.tool);
  tool.add(toolTip);

  // Joint frames and labels.
  const frameLayer = new THREE.Group();
  scene.add(frameLayer);
  const frameHelpers = {
    shoulder: new THREE.AxesHelper(0.16),
    elbow: new THREE.AxesHelper(0.13),
    tip: new THREE.AxesHelper(0.12),
  };
  Object.values(frameHelpers).forEach((helper) => {
    helper.material.transparent = true;
    helper.material.opacity = 0.9;
    frameLayer.add(helper);
  });

  const labelElements = JOINTS.map((_, index) => {
    const element = document.createElement('div');
    element.className = 'joint-label';
    element.innerHTML = `<b>J${index + 1}</b> &nbsp; 0°`;
    const label = new CSS2DObject(element);
    label.position.z = 0.12;
    scene.add(label);
    return { label, element };
  });
  const tipElement = document.createElement('div');
  tipElement.className = 'joint-label tip-label';
  tipElement.innerHTML = '<b>TCP</b> &nbsp; z 0.00';
  const tipLabel = new CSS2DObject(tipElement);
  scene.add(tipLabel);

  /* ------------------------------------------------------------------ *
   * Goal marker
   * ------------------------------------------------------------------ */
  const goalMarker = new THREE.Group();
  goalMarker.visible = false;
  scene.add(goalMarker);
  const goalRing = new THREE.Mesh(
    new THREE.RingGeometry(0.034, 0.046, 40),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
  );
  goalMarker.add(goalRing);
  const goalDot = new THREE.Mesh(
    new THREE.CircleGeometry(0.012, 20),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.95 }),
  );
  goalDot.position.z = 0.001;
  goalMarker.add(goalDot);
  goalMarker.add(new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-0.08, 0, 0), new THREE.Vector3(0.08, 0, 0),
      new THREE.Vector3(0, -0.08, 0), new THREE.Vector3(0, 0.08, 0),
    ]),
    new THREE.LineBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.6 }),
  ));
  // A thin post shows which plane the goal belongs to.
  const goalPost = new THREE.Mesh(
    new THREE.CylinderGeometry(0.004, 0.004, 0.2, 8),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.35 }),
  );
  goalPost.rotation.x = Math.PI / 2;
  goalPost.position.z = 0.1;
  goalMarker.add(goalPost);

  /* ------------------------------------------------------------------ *
   * State
   * ------------------------------------------------------------------ */
  let angles = JOINTS.map(({ home }) => home);
  let mode = 'fk';
  let elbowPreference = 'up';
  let goal = { x: 0, y: 0, z: PLANES.lower.height, plane: 'lower' };
  let goalClamped = false;
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
        <div class="range-limits"><span>${joint.min}°</span><span>${joint.max}°</span>${index > 0 ? '<em>negative = below horizontal</em>' : ''}</div>
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
    const rows = [
      ['1', ARM.upperArm, 'Shoulder pitch → elbow', 'rises with θ₂'],
      ['2', ARM.forearm, 'Elbow pitch → wrist', 'hangs down'],
      ['3', ARM.tool, 'Wrist → tool tip', 'collinear'],
    ];
    $('#s-link-table tbody').innerHTML = rows.map(([index, length, use, note]) => `
      <tr><td>${index}</td><td>${length.toFixed(2)}</td><td>${use}</td><td>${note}</td></tr>
    `).join('');

    $('#s-frames').innerHTML = [
      ['BASE', 'floor, under the mount'],
      ['S', 'shoulder (yaw + pitch)'],
      ['E', 'elbow'],
      ['TCP', 'tool center point'],
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
      // Start from the current tool position so the arm does not jump.
      const pose = solveForward(angles);
      const plane = touchingPlane(pose.tip.z)?.key
        ?? (pose.tip.z > (PLANES.mount.height + PLANES.lower.height) / 2 ? 'mount' : 'lower');
      elbowPreference = pose.forearmElevationDeg < pose.angles[1] ? 'up' : 'down';
      goal = { x: pose.tip.x, y: pose.tip.y, z: PLANES[plane].height, plane };
      goalClamped = false;
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
    root.querySelectorAll('[data-spatial-plane]').forEach((button) => {
      const selected = button.dataset.spatialPlane === goal.plane;
      button.setAttribute('aria-pressed', String(selected));
      button.classList.toggle('is-active', selected);
    });
    root.querySelectorAll('[data-spatial-elbow]').forEach((button) => {
      const selected = button.dataset.spatialElbow === elbowPreference;
      button.setAttribute('aria-pressed', String(selected));
      button.classList.toggle('is-active', selected);
    });
    $('#s-goal-fields').classList.toggle('is-disabled', mode !== 'ik');
    $('#s-mode-note').textContent = mode === 'ik'
      ? 'Pick a plane, then drag on it (or type X and Y). The arm solves yaw, shoulder pitch and elbow to touch that point.'
      : 'Move a joint slider, or click a bench to switch to goal control.';
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
    const solution = solveInverse({ x: goal.x, y: goal.y, z: goal.z }, { elbow: elbowPreference, clamp: true });
    if (!solution.ok) {
      goalClamped = true;
      return;
    }
    let clamped = solution.clamped;
    angles = solution.angles.map((angle, index) => {
      const result = unwrapNear(angle, angles[index], JOINTS[index]);
      if (result.clamped) clamped = true;
      return result.value;
    });
    goalClamped = clamped;
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
    pitchGroup.rotation.set(0, -yawRad, 0);
    elbowGroup.position.set(pose.elbow.x, pose.elbow.y, pose.elbow.z);
    elbowGroup.rotation.set(0, -yawRad, 0);
    forearmGroup.position.set(pose.elbow.x, pose.elbow.y, pose.elbow.z);
    forearmGroup.rotation.set(0, -yawRad, -elbow * DEG);

    frameHelpers.shoulder.position.set(BASE_POSITION.x, BASE_POSITION.y, ARM.shoulderZ);
    const shoulderEuler = new THREE.Euler(0, -yawRad, 0);
    frameHelpers.shoulder.rotation.copy(shoulderEuler);
    frameHelpers.elbow.position.copy(new THREE.Vector3(pose.elbow.x, pose.elbow.y, pose.elbow.z));
    frameHelpers.elbow.rotation.set(0, -yawRad, -elbow * DEG);
    frameHelpers.tip.position.set(pose.tip.x, pose.tip.y, pose.tip.z);
    frameHelpers.tip.rotation.set(0, -yawRad, -elbow * DEG);

    const labelPositions = [
      new THREE.Vector3(BASE_POSITION.x, BASE_POSITION.y, ARM.shoulderZ + 0.14),
      new THREE.Vector3(pose.elbow.x, pose.elbow.y, pose.elbow.z + 0.1),
      new THREE.Vector3(pose.wrist.x, pose.wrist.y, pose.wrist.z),
    ];
    labelElements.forEach(({ label, element }, index) => {
      label.position.copy(labelPositions[index]);
      element.innerHTML = `<b>J${index + 1}</b> &nbsp; ${angles[index].toFixed(0)}°`;
    });
    tipLabel.position.set(pose.tip.x, pose.tip.y, pose.tip.z + 0.02);
    tipElement.innerHTML = `<b>TCP</b> &nbsp; z ${fmt(pose.tip.z, 2)}`;

    goalMarker.position.set(goal.x, goal.y, goal.z);
    goalMarker.visible = mode === 'ik';
    const color = goalClamped ? 0xff9d67 : 0x7ff0cf;
    goalRing.material.color.setHex(color);
    goalDot.material.color.setHex(color);
    goalPost.material.color.setHex(color);
    goalPost.position.z = 0.1;

    // The elbow is the headline: is it up in the air?
    const elbowState = pose.elbowZ > PLANES.mount.height + 0.03 ? 'HIGH' : pose.elbowZ > ARM.shoulderZ ? 'RAISED' : 'LOW';
    const elbowChip = $('#s-elbow-state');
    elbowChip.textContent = elbowState;
    elbowChip.className = `status-chip ${elbowState === 'HIGH' ? 'chip-ok' : elbowState === 'RAISED' ? 'chip-neutral' : 'chip-warn'}`;

    const contact = touchingPlane(pose.tip.z);
    const planeChip = $('#s-contact-state');
    planeChip.textContent = contact ? contact.name.split('·')[0].trim().toUpperCase() : 'IN THE AIR';
    planeChip.className = `status-chip ${contact ? 'chip-ok' : 'chip-neutral'}`;

    return pose;
  }

  function updateReadouts(pose) {
    const { matrix, determinant, dexterity } = jacobian(angles);

    $('#s-pos-x').textContent = fmt(pose.tip.x);
    $('#s-pos-y').textContent = fmt(pose.tip.y);
    $('#s-pos-z').textContent = fmt(pose.tip.z);
    $('#s-viewport-position').innerHTML = `X ${fmt(pose.tip.x)} <em>Y ${fmt(pose.tip.y)}</em> Z ${fmt(pose.tip.z)}`;
    $('#s-elbow-height').textContent = `${fmt(pose.elbowZ)} m`;
    $('#s-elbow-rise').textContent = `${pose.elbowRise >= 0 ? '+' : '−'}${Math.abs(pose.elbowRise).toFixed(3)} m`;
    $('#s-reach').textContent = `${pose.reach.toFixed(3)} m`;
    $('#s-forearm-angle').textContent = `${pose.forearmElevationDeg.toFixed(1)}°`;
    $('#s-yaw').textContent = `${wrapDegrees(angles[0]).toFixed(1)}°`;
    $('#s-pitch').textContent = `${angles[1].toFixed(1)}°`;
    $('#s-elbow-angle').textContent = `${angles[2].toFixed(1)}°`;

    const singular = dexterity < 0.01;
    const chip = $('#s-dexterity-status');
    chip.textContent = singular ? 'SINGULAR' : 'DEXTEROUS';
    chip.className = `status-chip ${singular ? 'chip-warn' : 'chip-ok'}`;
    $('#s-dexterity').textContent = dexterity.toFixed(4);
    if (mode === 'ik') {
      const status = $('#s-reach-status');
      status.textContent = goalClamped ? 'CLAMPED TO WORKSPACE' : 'GOAL IN REACH';
      status.className = `status-chip ${goalClamped ? 'chip-warn' : 'chip-ok'}`;
    }

    for (let row = 0; row < 3; row += 1) {
      for (let column = 0; column < 3; column += 1) {
        $(`#s-jac-${row}-${column}`).textContent = matrix[row][column].toFixed(3);
      }
    }

    const frames = $('#s-frames').children;
    const positions = [BASE_POSITION, pose.shoulder, pose.elbow, pose.tip];
    positions.forEach((point, index) => {
      if (frames[index]) frames[index].lastElementChild.textContent = `${fmt(point.x)}, ${fmt(point.y)}, ${fmt(point.z)}`;
    });

    // Plane table shows, per plane, the height and the reachable annulus.
    ['mount', 'lower'].forEach((key) => {
      const reach = reachOnPlane(PLANES[key].height);
      $(`#s-plane-${key}-height`).textContent = `${PLANES[key].height.toFixed(2)} m`;
      $(`#s-plane-${key}-reach`).textContent = reach.inner > 0.001
        ? `${reach.inner.toFixed(2)}–${reach.outer.toFixed(2)} m`
        : `≤ ${reach.outer.toFixed(2)} m`;
    });
    $('#s-goal-plane-label').textContent = PLANES[goal.plane].name;
    $('#s-goal-z').textContent = `${goal.z.toFixed(2)} m`;
    $('#s-determinant').textContent = determinant.toFixed(4);
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

  function moveGoalToPlane(key) {
    goal = { ...goal, plane: key, z: PLANES[key].height };
    goalClamped = false;
    updateModeUI();
  }

  function goHome() {
    stopDemo();
    angles = JOINTS.map(({ home }) => home);
    if (mode === 'ik') {
      const pose = solveForward(angles);
      goal = { x: pose.tip.x, y: pose.tip.y, z: PLANES.lower.height, plane: 'lower' };
      elbowPreference = 'up';
      goalClamped = false;
      solveGoal();
      syncGoalInputs();
      updateModeUI();
    }
    syncJointInputs();
    refresh();
  }

  function goRandom() {
    stopDemo();
    angles = JOINTS.map(({ min, max }) => THREE.MathUtils.randFloat(min, max));
    if (mode === 'ik') {
      const pose = solveForward(angles);
      goal = { ...goal, x: pose.tip.x, y: pose.tip.y };
      goalClamped = false;
      syncGoalInputs();
    }
    syncJointInputs();
    refresh();
  }

  /** The demo tour: touch both planes, then sweep out and back. */
  const tour = [
    { x: 0.34, y: -0.30, plane: 'lower' },
    { x: 0.02, y: 0.26, plane: 'mount' },
    { x: 0.52, y: 0.02, plane: 'lower' },
    { x: 0.30, y: 0.26, plane: 'mount' },
  ];

  function stepDemo(time) {
    const t = (time - animationStart) / 1000;
    if (mode === 'ik') {
      const segment = (t * 0.32) % tour.length;
      const index = Math.floor(segment);
      const blend = segment - index;
      const from = tour[index];
      const to = tour[(index + 1) % tour.length];
      const ease = blend * blend * (3 - 2 * blend);
      const plane = ease < 0.5 ? from.plane : to.plane;
      goal = {
        x: THREE.MathUtils.lerp(from.x, to.x, ease),
        y: THREE.MathUtils.lerp(from.y, to.y, ease),
        z: PLANES[plane].height,
        plane,
      };
      goalClamped = false;
      solveGoal();
      syncGoalInputs();
      updateModeUI();
    } else {
      angles = [
        18 * Math.sin(t * 0.42) - 12,
        62 + 26 * Math.sin(t * 0.55),
        118 + 34 * Math.sin(t * 0.61 + 1.2),
      ];
      syncJointInputs();
    }
    refresh();
    lastAnimationUpdate = time;
  }

  /* ------------------------------------------------------------------ *
   * Pointer: drop the goal on whichever bench is clicked
   * ------------------------------------------------------------------ */
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const decks = [
    { plane: PLANES.mount, x: mountY, width: deck.width, depth: deck.depth },
    { plane: PLANES.lower, x: lowerY, width: deck.lowerWidth, depth: deck.depth === 0 ? 0 : deck.lowerDepth },
  ];

  function deckAt(event) {
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    raycaster.setFromCamera(pointer, camera);
    let best = null;
    for (const entry of decks) {
      const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -(entry.plane.height));
      const hit = new THREE.Vector3();
      if (!raycaster.ray.intersectPlane(plane, hit)) continue;
      const withinX = Math.abs(hit.x - entry.x) <= entry.width / 2;
      const withinY = Math.abs(hit.y) <= entry.depth / 2;
      if (withinX && withinY && (!best || hit.distanceTo(camera.position) < best.distance)) {
        best = { ...entry, point: hit, distance: hit.distanceTo(camera.position) };
      }
    }
    return best;
  }

  // A click drops the goal on a bench; a drag orbits. Distinguish by movement so
  // the two never fight over the same gesture.
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
    const hit = deckAt(event);
    if (!hit) return;
    stopDemo();
    mode = 'ik';
    updateModeUI();
    moveGoalToPlane(hit.plane.key);
    goal = { ...goal, x: hit.point.x, y: hit.point.y };
    solveGoal();
    syncGoalInputs();
    refresh();
  });
  canvas?.addEventListener('contextmenu', (event) => event.preventDefault());
  canvas?.addEventListener('wheel', (event) => event.preventDefault(), { passive: false });

  /* ------------------------------------------------------------------ *
   * Panel wiring
   * ------------------------------------------------------------------ */
  root.querySelectorAll('[data-spatial-view]').forEach((button) => {
    button.addEventListener('click', () => {
      activeView = button.dataset.spatialView;
      root.querySelectorAll('[data-spatial-view]').forEach((other) => {
        const selected = other.dataset.spatialView === activeView;
        other.setAttribute('aria-pressed', String(selected));
        other.classList.toggle('is-active', selected);
      });
      camGoal.eye.copy(views[activeView].eye);
      camGoal.target.copy(views[activeView].target);
      camGoal.zoom = views[activeView].zoom;
      syncingView = true;
    });
  });

  root.querySelectorAll('[data-spatial-mode]').forEach((button) => {
    button.addEventListener('click', () => {
      stopDemo();
      setMode(button.dataset.spatialMode);
    });
  });

  root.querySelectorAll('[data-spatial-plane]').forEach((button) => {
    button.addEventListener('click', () => {
      stopDemo();
      if (mode !== 'ik') setMode('ik', { silent: true });
      moveGoalToPlane(button.dataset.spatialPlane);
      solveGoal();
      syncGoalInputs();
      refresh();
    });
  });

  root.querySelectorAll('[data-spatial-elbow]').forEach((button) => {
    button.addEventListener('click', () => {
      elbowPreference = button.dataset.spatialElbow;
      stopDemo();
      updateModeUI();
      if (mode === 'ik') {
        solveGoal();
        refresh();
      }
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
      refresh();
    });
    input.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') input.blur();
    });
  });

  $('#s-copy-pose').addEventListener('click', () => {
    const pose = solveForward(angles);
    if (mode !== 'ik') setMode('ik', { silent: true });
    const plane = touchingPlane(pose.tip.z)?.key ?? goal.plane;
    goal = { x: pose.tip.x, y: pose.tip.y, z: PLANES[plane].height, plane };
    goalClamped = false;
    syncGoalInputs();
    updateModeUI();
    refresh();
  });

  $('#s-reset').addEventListener('click', goHome);
  $('#s-random').addEventListener('click', goRandom);

  $('#s-show-rings').addEventListener('change', (event) => { workspaceLayer.visible = event.target.checked; });
  $('#s-show-frames').addEventListener('change', (event) => {
    frameLayer.visible = event.target.checked;
    labelElements.forEach(({ label }) => { label.visible = event.target.checked; });
  });
  $('#s-animate').addEventListener('change', (event) => {
    animationEnabled = event.target.checked;
    if (animationEnabled) animationStart = performance.now();
  });

  /* ------------------------------------------------------------------ *
   * Camera + sizing
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
    if (syncingView) {
      const lerp = Math.min(1, delta * 4.5);
      camState.eye.lerp(camGoal.eye, lerp);
      camState.target.lerp(camGoal.target, lerp);
      camState.zoom += (camGoal.zoom - camState.zoom) * lerp;
      camera.position.copy(camState.eye);
      controls.target.copy(camState.target);
      camera.zoom = camState.zoom;
      camera.updateProjectionMatrix();
      if (camState.eye.distanceTo(camGoal.eye) < 0.01) syncingView = false;
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
  goal = { x: solveForward(angles).tip.x, y: solveForward(angles).tip.y, z: PLANES.lower.height, plane: 'lower' };
  syncGoalInputs();
  updateModeUI();
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
  };
}
