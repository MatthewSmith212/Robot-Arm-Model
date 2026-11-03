import * as THREE from 'three';
import { CSS2DObject, CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';
import {
  PLANAR_LINKS,
  PLANAR_LIMITS,
  TIP_WORKSPACE,
  WRIST_WORKSPACE,
  planarJacobian,
  solvePlanarForward,
  solvePlanarInverse,
  wrapDegrees,
} from './planar-kinematics.js';
import { SHADOW_MAP_TYPE, createRenderer, showRendererFallback } from './webgl.js';

const DEG = Math.PI / 180;
const FLOOR_SPAN = 3.4;        // metres of floor drawn by the grid texture
const FRUSTUM_HEIGHT = 3.05;   // metres visible vertically at zoom 1
const LINK_Z = 0.078;          // centre height of the link bars above the floor
const JOINT_Z = 0.062;
const LINK_RADIUS = [0.036, 0.030, 0.024];

const JOINT_META = [
  { name: 'Base swivel', axis: 'REVOLUTE ABOUT Z · FLOOR NORMAL', hint: 'Aims the whole arm around the mount.' },
  { name: 'Shoulder', axis: 'REVOLUTE ABOUT Z · FLOOR NORMAL', hint: 'Bends the upper arm within the plane.' },
  { name: 'Wrist', axis: 'REVOLUTE ABOUT Z · FLOOR NORMAL', hint: 'Swings the tool about the wrist point.' },
];

const JOINTS = JOINT_META.map((meta, index) => ({
  ...meta,
  ...PLANAR_LIMITS[index],
  label: `J${index + 1}`,
}));

const clamp = THREE.MathUtils.clamp;
const fmt = (value, digits = 3) => (Math.abs(value) < 0.0005 ? 0 : value).toFixed(digits);

/**
 * Mount the 3-DOF planar arm viewer (a floor-mounted arm whose links sweep the
 * mounting plane) inside `root` and return its frame hooks.
 */
export function createPlanarView(root) {
  const $ = (selector) => root.querySelector(selector);
  const viewport = $('#planar-viewport');

  /* ------------------------------------------------------------------ *
   * Renderer, camera, views
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
    renderer.domElement.setAttribute('aria-label', 'Top-down 3D model of a three degree of freedom planar robotic arm mounted on a floor');
    renderer.domElement.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;outline:none;touch-action:none;cursor:crosshair;';
    viewport.prepend(renderer.domElement);
  }
  const canvas = renderer?.domElement;

  const labelRenderer = new CSS2DRenderer();
  labelRenderer.domElement.classList.add('labels-layer');
  labelRenderer.domElement.style.cssText = 'position:absolute;inset:0;overflow:hidden;pointer-events:none;z-index:2;';
  viewport.appendChild(labelRenderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.05, 30);
  camera.up.set(0, 1, 0);

  const VIEWS = {
    iso: {
      eye: new THREE.Vector3(0.15, -2.15, 2.05),
      target: new THREE.Vector3(0.1, 0.26, 0.02),
    },
    top: {
      eye: new THREE.Vector3(0.1, 0.26, 3.4),
      target: new THREE.Vector3(0.1, 0.26, 0),
    },
  };
  const camState = { eye: VIEWS.iso.eye.clone(), target: VIEWS.iso.target.clone(), zoom: 1 };
  const camGoal = { eye: VIEWS.iso.eye.clone(), target: VIEWS.iso.target.clone(), zoom: 1 };
  let activeView = 'iso';
  const pan = new THREE.Vector2();

  /* ------------------------------------------------------------------ *
   * Lights & floor
   * ------------------------------------------------------------------ */
  scene.add(new THREE.HemisphereLight(0xe4f7f2, 0x121e26, 2.0));
  const keyLight = new THREE.DirectionalLight(0xfff2df, 2.5);
  keyLight.position.set(-1.4, -2.1, 3.2);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(1024, 1024);
  keyLight.shadow.camera.left = -1.8;
  keyLight.shadow.camera.right = 1.8;
  keyLight.shadow.camera.top = 1.8;
  keyLight.shadow.camera.bottom = -1.8;
  keyLight.shadow.camera.near = 0.1;
  keyLight.shadow.camera.far = 9;
  keyLight.shadow.bias = -0.0008;
  scene.add(keyLight);
  const rimLight = new THREE.DirectionalLight(0x59cdb9, 1.6);
  rimLight.position.set(2.2, 1.6, 2.0);
  scene.add(rimLight);

  function makeFloorTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 1024;
    canvas.height = 1024;
    const ctx = canvas.getContext('2d');
    const px = canvas.width / FLOOR_SPAN;

    ctx.fillStyle = '#0c1a22';
    ctx.fillRect(0, 0, 1024, 1024);

    const glow = ctx.createRadialGradient(512, 512, 40, 512, 512, 720);
    glow.addColorStop(0, 'rgba(35, 82, 84, 0.5)');
    glow.addColorStop(0.55, 'rgba(18, 40, 47, 0.24)');
    glow.addColorStop(1, 'rgba(8, 16, 22, 0)');
    ctx.fillStyle = glow;
    ctx.fillRect(0, 0, 1024, 1024);

    const half = FLOOR_SPAN / 2;
    const steps = Math.round(FLOOR_SPAN / 0.1);
    for (let step = 0; step <= steps; step += 1) {
      const metres = -half + step * 0.1;
      const position = (metres + half) * px;
      const major = Math.abs(Math.abs(metres) % 0.5) < 1e-6;
      ctx.strokeStyle = major ? 'rgba(118, 165, 172, 0.3)' : 'rgba(92, 132, 141, 0.13)';
      ctx.lineWidth = major ? 1.7 : 1;
      ctx.beginPath();
      ctx.moveTo(position, 0);
      ctx.lineTo(position, 1024);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, position);
      ctx.lineTo(1024, position);
      ctx.stroke();
    }

    // Floor axes: X through the base, Y through the base.
    const centre = (FLOOR_SPAN / 2) * px;
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(239, 114, 122, 0.42)';
    ctx.beginPath();
    ctx.moveTo(0, centre);
    ctx.lineTo(1024, centre);
    ctx.stroke();
    ctx.strokeStyle = 'rgba(120, 198, 140, 0.4)';
    ctx.beginPath();
    ctx.moveTo(centre, 0);
    ctx.lineTo(centre, 1024);
    ctx.stroke();

    ctx.font = '600 17px "DM Mono", monospace';
    ctx.fillStyle = 'rgba(146, 176, 184, 0.5)';
    ctx.textAlign = 'center';
    for (let metres = 0.5; metres <= FLOOR_SPAN / 2; metres += 0.5) {
      const offset = metres * px;
      if (centre + offset > 1000) continue;
      ctx.fillText(`${metres.toFixed(1)} m`, centre + offset, centre - 12);
      ctx.fillText(`${metres.toFixed(1)} m`, centre + 12, centre - offset - 6);
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = renderer ? renderer.capabilities.getMaxAnisotropy() : 1;
    return texture;
  }

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(FLOOR_SPAN, FLOOR_SPAN),
    new THREE.MeshStandardMaterial({ map: makeFloorTexture(), roughness: 0.94, metalness: 0.06 }),
  );
  floor.receiveShadow = true;
  scene.add(floor);

  /* ------------------------------------------------------------------ *
   * Materials
   * ------------------------------------------------------------------ */
  const materials = {
    housing: new THREE.MeshStandardMaterial({ color: '#29353d', metalness: 0.78, roughness: 0.32 }),
    shell: new THREE.MeshStandardMaterial({ color: '#bfccca', metalness: 0.7, roughness: 0.26 }),
    shellLight: new THREE.MeshStandardMaterial({ color: '#dde6df', metalness: 0.64, roughness: 0.22 }),
    dark: new THREE.MeshStandardMaterial({ color: '#101a20', metalness: 0.62, roughness: 0.36 }),
    accent: new THREE.MeshStandardMaterial({ color: '#f3925e', metalness: 0.5, roughness: 0.26, emissive: '#3d1807', emissiveIntensity: 0.24 }),
    accentSoft: new THREE.MeshStandardMaterial({ color: '#e4a077', metalness: 0.62, roughness: 0.3 }),
    teal: new THREE.MeshStandardMaterial({ color: '#5ee2bd', metalness: 0.36, roughness: 0.22, emissive: '#105943', emissiveIntensity: 0.7 }),
    fastener: new THREE.MeshStandardMaterial({ color: '#d6dedb', metalness: 0.82, roughness: 0.19 }),
  };

  function makeCylinder(radiusTop, radiusBottom, height, material, segments = 30) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radiusTop, radiusBottom, height, segments, 1), material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  function makeBox(width, height, depth, material, position) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), material);
    if (position) mesh.position.set(...position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  /** A cylinder whose axis lies along +Z, i.e. the normal of the mounting plane. */
  function makeJointBody(radius, height, material, segments = 30) {
    const mesh = makeCylinder(radius, radius, height, material, segments);
    mesh.rotation.x = Math.PI / 2;
    return mesh;
  }

  /* ------------------------------------------------------------------ *
   * Workspace rings drawn on the floor
   * ------------------------------------------------------------------ */
  const workspaceLayer = new THREE.Group();
  scene.add(workspaceLayer);

  function makeRing(radius, color, opacity, dashed = true) {
    const points = [];
    for (let i = 0; i <= 160; i += 1) {
      const angle = (i / 160) * Math.PI * 2;
      points.push(new THREE.Vector3(Math.cos(angle) * radius, Math.sin(angle) * radius, 0.0025));
    }
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = dashed
      ? new THREE.LineDashedMaterial({ color, transparent: true, opacity, dashSize: 0.045, gapSize: 0.035 })
      : new THREE.LineBasicMaterial({ color, transparent: true, opacity });
    const line = new THREE.Line(geometry, material);
    line.computeLineDistances();
    return line;
  }

  // For these link lengths the tool tip can reach anywhere inside the outer
  // ring (the wrist annulus plus a link that can point any way), so the tip
  // workspace is a filled disk, while the wrist point is an annulus.
  const tipDisk = new THREE.Mesh(
    new THREE.RingGeometry(0.0005, TIP_WORKSPACE.outerRadius, 96, 1),
    new THREE.MeshBasicMaterial({ color: 0x2f8f7f, transparent: true, opacity: 0.055, side: THREE.DoubleSide, depthWrite: false }),
  );
  tipDisk.position.z = 0.0012;
  workspaceLayer.add(tipDisk);
  workspaceLayer.add(makeRing(TIP_WORKSPACE.outerRadius, 0x66ddc0, 0.52));
  workspaceLayer.add(makeRing(WRIST_WORKSPACE.maxReach, 0x6f97a4, 0.42));
  workspaceLayer.add(makeRing(WRIST_WORKSPACE.minReach, 0x6f97a4, 0.3, false));

  /* ------------------------------------------------------------------ *
   * Base mount (bolted to the floor)
   * ------------------------------------------------------------------ */
  const baseGroup = new THREE.Group();
  scene.add(baseGroup);

  const plate = makeBox(0.3, 0.3, 0.022, materials.housing, [0, 0, 0.011]);
  baseGroup.add(plate);
  const plateInlay = makeBox(0.24, 0.24, 0.006, materials.dark, [0, 0, 0.024]);
  baseGroup.add(plateInlay);
  for (const xSign of [-1, 1]) {
    for (const ySign of [-1, 1]) {
      const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.012, 12), materials.fastener);
      bolt.rotation.x = Math.PI / 2;
      bolt.position.set(xSign * 0.115, ySign * 0.115, 0.026);
      baseGroup.add(bolt);
    }
  }

  const turret = makeJointBody(0.083, 0.072, materials.shell, 36);
  turret.position.z = 0.06;
  baseGroup.add(turret);
  const turretTrim = makeJointBody(0.086, 0.013, materials.accent, 36);
  turretTrim.position.z = 0.03;
  baseGroup.add(turretTrim);
  const turretCap = makeJointBody(0.055, 0.014, materials.dark, 30);
  turretCap.position.z = 0.102;
  baseGroup.add(turretCap);

  // A fixed "home" reference notch on the mounting plate (points along +X).
  const homeNotch = makeBox(0.06, 0.012, 0.008, materials.teal, [0.13, 0, 0.03]);
  baseGroup.add(homeNotch);

  /* ------------------------------------------------------------------ *
   * Links, joint housings, tool
   * ------------------------------------------------------------------ */
  const linkGroups = PLANAR_LINKS.map((link, index) => {
    const group = new THREE.Group();
    const radius = LINK_RADIUS[index];
    const material = index % 2 === 0 ? materials.shell : materials.shellLight;

    const beam = makeCylinder(radius * 0.9, radius * 1.06, 1, material, 24);
    beam.rotation.z = -Math.PI / 2;
    beam.position.set(link.length / 2, 0, 0);
    group.add(beam);

    const spine = makeBox(link.length * 0.72, radius * 0.9, radius * 1.35, materials.housing,
      [link.length * 0.5, 0, radius * 0.42]);
    group.add(spine);

    const band = makeCylinder(radius * 1.16, radius * 1.16, 0.026, index === 2 ? materials.accentSoft : materials.accent, 24);
    band.rotation.z = -Math.PI / 2;
    band.position.set(link.length * 0.84, 0, 0);
    group.add(band);

    const collar = makeCylinder(radius * 1.24, radius * 1.24, 0.018, materials.dark, 24);
    collar.rotation.z = -Math.PI / 2;
    collar.position.set(link.length * 0.06, 0, 0);
    group.add(collar);

    group.position.z = LINK_Z;
    scene.add(group);
    return group;
  });

  const housings = JOINTS.map((_, index) => {
    const group = new THREE.Group();
    const outerRadius = [0.075, 0.062, 0.05][index];
    const height = [0.055, 0.05, 0.044][index];

    const body = makeJointBody(outerRadius, height, materials.housing, 32);
    group.add(body);

    const collar = makeJointBody(outerRadius * 0.94, 0.012, materials.accent, 32);
    collar.position.z = height / 2 + 0.005;
    group.add(collar);

    const face = makeJointBody(outerRadius * 0.7, 0.014, materials.shell, 30);
    face.position.z = height / 2 + 0.014;
    group.add(face);

    const hub = makeJointBody(outerRadius * 0.3, 0.02, materials.fastener, 22);
    hub.position.z = height / 2 + 0.028;
    group.add(hub);

    const hubCap = makeJointBody(outerRadius * 0.16, 0.008, materials.dark, 18);
    hubCap.position.z = height / 2 + 0.041;
    group.add(hubCap);

    // Encoder dot offsets from the axis so rotation is visible on the housing.
    const encoder = new THREE.Mesh(new THREE.SphereGeometry(0.0095, 12, 10), materials.teal);
    encoder.position.set(outerRadius * 0.72, 0, height / 2 + 0.014);
    group.add(encoder);

    for (const xSign of [-1, 1]) {
      for (const ySign of [-1, 1]) {
        const screw = new THREE.Mesh(new THREE.SphereGeometry(0.0055, 8, 6), materials.fastener);
        screw.position.set(xSign * outerRadius * 0.5, ySign * outerRadius * 0.5, height / 2 + 0.012);
        screw.scale.set(1, 1, 0.55);
        group.add(screw);
      }
    }

    group.position.z = JOINT_Z;
    group.userData.height = height;
    scene.add(group);
    return group;
  });

  // Tool: a small two-finger gripper whose jaw line ends exactly at the tip.
  const tool = new THREE.Group();
  scene.add(tool);
  const toolPalm = makeBox(0.07, 0.052, 0.03, materials.housing, [-0.045, 0, 0]);
  tool.add(toolPalm);
  const toolPlate = makeBox(0.016, 0.056, 0.034, materials.accentSoft, [-0.012, 0, 0]);
  tool.add(toolPlate);
  for (const side of [-1, 1]) {
    const finger = makeBox(0.036, 0.012, 0.024, materials.shellLight, [-0.018, side * 0.019, 0]);
    tool.add(finger);
  }
  const toolTipDot = new THREE.Mesh(new THREE.SphereGeometry(0.013, 16, 12), materials.teal);
  tool.add(toolTipDot);
  tool.position.z = LINK_Z;

  // Joint frames (small axis tripods) and angle labels.
  const frameLayer = new THREE.Group();
  scene.add(frameLayer);
  const frameTriads = JOINTS.map(() => {
    const triad = new THREE.AxesHelper(0.085);
    triad.material.transparent = true;
    triad.material.opacity = 0.85;
    frameLayer.add(triad);
    return triad;
  });

  const tipTriad = new THREE.AxesHelper(0.11);
  tipTriad.material.transparent = true;
  tipTriad.material.opacity = 0.95;
  frameLayer.add(tipTriad);

  const jointLabels = JOINTS.map((_, index) => {
    const element = document.createElement('div');
    element.className = 'joint-label';
    element.innerHTML = `<b>J${index + 1}</b> &nbsp; 0°`;
    const label = new CSS2DObject(element);
    label.position.z = 0.1;
    scene.add(label);
    return { object: label, element };
  });

  const tipLabelElement = document.createElement('div');
  tipLabelElement.className = 'joint-label tip-label';
  tipLabelElement.innerHTML = '<b>TCP</b> &nbsp; φ 0°';
  const tipLabel = new CSS2DObject(tipLabelElement);
  tipLabel.position.z = 0.12;
  scene.add(tipLabel);

  /* ------------------------------------------------------------------ *
   * Target marker (inverse-kinematics goal)
   * ------------------------------------------------------------------ */
  const targetMarker = new THREE.Group();
  targetMarker.visible = false;
  scene.add(targetMarker);

  const targetHalo = new THREE.Mesh(
    new THREE.RingGeometry(0.03, 0.042, 40),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.9, side: THREE.DoubleSide }),
  );
  targetMarker.add(targetHalo);

  const targetDot = new THREE.Mesh(
    new THREE.CircleGeometry(0.011, 20),
    new THREE.MeshBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.95 }),
  );
  targetDot.position.z = 0.001;
  targetMarker.add(targetDot);

  const crossGeometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-0.075, 0, 0), new THREE.Vector3(0.075, 0, 0),
    new THREE.Vector3(0, -0.075, 0), new THREE.Vector3(0, 0.075, 0),
  ]);
  const cross = new THREE.LineSegments(crossGeometry, new THREE.LineBasicMaterial({ color: 0x7ff0cf, transparent: true, opacity: 0.55 }));
  targetMarker.add(cross);
  targetMarker.position.z = 0.004;

  /* ------------------------------------------------------------------ *
   * State
   * ------------------------------------------------------------------ */
  let angles = JOINTS.map(({ home }) => home);
  let mode = 'fk';
  let elbow = 'up';
  let target = { x: 0, y: 0, phiDeg: 0 };
  let targetClamped = false;
  let animationEnabled = false;
  let animationStart = 0;
  let lastAnimationUpdate = 0;

  const controls = { input: [], angleInput: [] };

  /* ------------------------------------------------------------------ *
   * Panel: joint controls
   * ------------------------------------------------------------------ */
  function buildJointControls() {
    $('#p-sliders').innerHTML = JOINTS.map((joint, index) => `
      <div class="joint-control">
        <div class="joint-topline">
          <span class="joint-index">${String(index + 1).padStart(2, '0')}</span>
          <div class="joint-copy"><span class="joint-name">${joint.name}</span><span class="joint-axis">${joint.axis}</span></div>
          <label class="joint-angle-wrap">
            <input class="joint-angle-input" id="p-angle-${index}" type="number" min="${joint.min}" max="${joint.max}" step="0.5" value="${joint.home.toFixed(1)}" aria-label="Planar joint ${index + 1} angle in degrees" />
            <span>°</span>
          </label>
        </div>
        <input class="joint-range" id="p-slider-${index}" type="range" min="${joint.min}" max="${joint.max}" step="0.5" value="${joint.home}" aria-label="Planar joint ${index + 1}, ${joint.name}, angle in degrees" />
        <div class="range-limits"><span>${joint.min}°</span><span>${joint.max}°</span></div>
      </div>
    `).join('');

    JOINTS.forEach((_, index) => {
      const input = $(`#p-slider-${index}`);
      const angleInput = $(`#p-angle-${index}`);
      controls.input[index] = input;
      controls.angleInput[index] = angleInput;

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
      controls.input[index].value = formatted;
      controls.angleInput[index].value = formatted;
    });
  }

  function syncTargetInputs() {
    $('#p-target-x').value = target.x.toFixed(3);
    $('#p-target-y').value = target.y.toFixed(3);
    $('#p-target-phi').value = wrapDegrees(target.phiDeg).toFixed(1);
  }

  /* ------------------------------------------------------------------ *
   * Panel: static tables
   * ------------------------------------------------------------------ */
  function buildStaticTables() {
    $('#p-link-table tbody').innerHTML = PLANAR_LINKS.map((link, index) => `
      <tr>
        <td>${index + 1}</td>
        <td>${link.length.toFixed(2)}</td>
        <td id="p-link-angle-${index}">0.0</td>
        <td>${index === 0 ? 'ground' : 'J' + index}</td>
      </tr>
    `).join('');

    $('#p-frames').innerHTML = Array.from({ length: 4 }, (_, index) => `
      <div class="frame-row"><span>${index === 3 ? 'TCP' : 'F' + index}</span><code id="p-frame-${index}">0.000, 0.000</code></div>
    `).join('');

    $('#p-jacobian tbody').innerHTML = Array.from({ length: 3 }, (_, row) =>
      '<tr>' + Array.from({ length: 3 }, (_, column) => `<td id="p-jac-${row}-${column}">0.000</td>`).join('') + '</tr>'
    ).join('');
  }

  /* ------------------------------------------------------------------ *
   * Mode & pose handling
   * ------------------------------------------------------------------ */
  function setMode(next, { silent = false } = {}) {
    const changing = mode !== next;
    mode = next;
    if (changing && mode === 'ik') {
      // Start from wherever the arm currently points so nothing jumps: the goal
      // is the current tool pose, keeping the current elbow configuration.
      const forward = solvePlanarForward(angles);
      elbow = angles[1] >= 0 ? 'up' : 'down';
      target = { x: forward.tip.x, y: forward.tip.y, phiDeg: forward.orientationDeg };
      targetClamped = false;
      syncTargetInputs();
    }
    updateModeUI();
    if (!silent) refresh();
  }

  function updateModeUI() {
    root.querySelectorAll('[data-planar-mode]').forEach((button) => {
      const selected = button.dataset.planarMode === mode;
      button.setAttribute('aria-pressed', String(selected));
      button.classList.toggle('is-active', selected);
    });
    root.querySelectorAll('[data-planar-elbow]').forEach((button) => {
      const selected = button.dataset.planarElbow === elbow;
      button.setAttribute('aria-pressed', String(selected));
      button.classList.toggle('is-active', selected);
    });
    $('#p-target-fields').classList.toggle('is-disabled', mode !== 'ik');
    $('#p-mode-note').textContent = mode === 'ik'
      ? 'Drag on the floor to move the goal. The arm solves joint angles for that pose; goals outside the rings clamp to the nearest reachable point.'
      : 'Move a joint slider (or drag on the floor to switch to goal-pose control).';
    if (renderer) renderer.domElement.style.cursor = mode === 'ik' ? 'crosshair' : 'grab';
  }

  /** Keep a solved joint value on the same revolution as `reference`, then honour its limit. */
  function unwrapNear(value, reference, limit) {
    let unwrapped = value;
    while (unwrapped - reference > 180) unwrapped -= 360;
    while (reference - unwrapped > 180) unwrapped += 360;
    const limited = clamp(unwrapped, limit.min, limit.max);
    if (limited === unwrapped) return { value: limited, clamped: false };
    const fallback = clamp(value, limit.min, limit.max);
    return { value: fallback, clamped: true };
  }

  /**
   * Drive the arm from the goal pose. Out-of-reach goals and joint limits are
   * clamped; `targetClamped` reports that the marker is unreachable as drawn.
   */
  function solveTarget() {
    const solution = solvePlanarInverse(target, { elbow, clamp: true });
    if (!solution.ok) {
      targetClamped = true;
      return;
    }
    let clamped = solution.clamped;
    angles = solution.anglesDeg.map((angle, index) => {
      const result = unwrapNear(angle, angles[index], JOINTS[index]);
      if (result.clamped) clamped = true;
      return result.value;
    });
    targetClamped = clamped;
    syncJointInputs();
  }

  /* ------------------------------------------------------------------ *
   * Scene refresh
   * ------------------------------------------------------------------ */
  function updateScene() {
    const forward = solvePlanarForward(angles);
    const { points, absoluteDegrees, tip, orientationDeg } = forward;

    PLANAR_LINKS.forEach((link, index) => {
      const group = linkGroups[index];
      group.position.x = points[index].x;
      group.position.y = points[index].y;
      group.rotation.z = absoluteDegrees[index] * DEG;
      group.visible = true;
    });

    JOINTS.forEach((_, index) => {
      const housing = housings[index];
      housing.position.x = points[index].x;
      housing.position.y = points[index].y;
      housing.rotation.z = (index === 0 ? 0 : absoluteDegrees[index - 1]) * DEG;

      const triad = frameTriads[index];
      triad.position.set(points[index].x, points[index].y, 0.02);
      triad.rotation.z = (index === 0 ? 0 : absoluteDegrees[index - 1]) * DEG;

      const label = jointLabels[index];
      label.object.position.set(points[index].x, points[index].y, 0.13);
      label.element.innerHTML = `<b>${JOINTS[index].label}</b> &nbsp; ${angles[index].toFixed(0)}°`;
    });

    tool.position.x = tip.x;
    tool.position.y = tip.y;
    tool.rotation.z = orientationDeg * DEG;

    tipTriad.position.set(tip.x, tip.y, 0.02);
    tipTriad.rotation.z = orientationDeg * DEG;
    tipLabel.position.set(tip.x, tip.y, 0.12);
    tipLabelElement.innerHTML = `<b>TCP</b> &nbsp; φ ${orientationDeg.toFixed(0)}°`;

    targetMarker.position.x = target.x;
    targetMarker.position.y = target.y;
    targetMarker.visible = mode === 'ik';
    const markerColor = targetClamped ? 0xff9d67 : 0x7ff0cf;
    targetHalo.material.color.setHex(markerColor);
    targetDot.material.color.setHex(markerColor);
    cross.material.color.setHex(markerColor);

    return { forward, points, absoluteDegrees, tip, orientationDeg };
  }

  function updateReadouts({ forward, absoluteDegrees, tip, orientationDeg }) {
    const jacobian = planarJacobian(angles);
    const reach = forward.reach;

    $('#p-pos-x').textContent = fmt(tip.x);
    $('#p-pos-y').textContent = fmt(tip.y);
    $('#p-pos-phi').textContent = `${orientationDeg.toFixed(1)}°`;
    $('#p-reach').textContent = `${reach.toFixed(3)} m`;
    $('#p-height').textContent = '0.000 m';
    $('#p-dexterity').textContent = jacobian.dexterity.toFixed(3);
    $('#p-jacobian-det').textContent = jacobian.determinant.toFixed(3);

    $('#p-viewport-position').innerHTML = `X ${fmt(tip.x)} <em>Y ${fmt(tip.y)}</em> φ ${orientationDeg.toFixed(0)}°`;

    const singular = jacobian.dexterity < 0.012;
    const status = $('#p-dexterity-status');
    status.textContent = singular ? 'SINGULAR' : 'DEXTEROUS';
    status.className = `status-chip ${singular ? 'chip-warn' : 'chip-ok'}`;

    const reachStatus = $('#p-reach-status');
    if (mode === 'ik') {
      reachStatus.textContent = targetClamped ? 'CLAMPED TO WORKSPACE' : 'TARGET IN REACH';
      reachStatus.className = `status-chip ${targetClamped ? 'chip-warn' : 'chip-ok'}`;
    } else {
      reachStatus.textContent = 'JOINT-SPACE CONTROL';
      reachStatus.className = 'status-chip chip-neutral';
    }

    $('#p-joint-abs').innerHTML = absoluteDegrees
      .map((angle, index) => `<div><span>χ${index + 1}</span><b>${angle.toFixed(1)}°</b></div>`)
      .join('');

    PLANAR_LINKS.forEach((_, index) => {
      $(`#p-link-angle-${index}`).textContent = absoluteDegrees[index].toFixed(1);
    });

    const frameRows = $('#p-frames').children;
    if (frameRows[0]) frameRows[0].lastElementChild.textContent = '0.000, 0.000';
    forward.points.slice(1).forEach((point, index) => {
      const row = frameRows[index + 1];
      if (row) row.lastElementChild.textContent = `${fmt(point.x)}, ${fmt(point.y)}`;
    });

    for (let row = 0; row < 3; row += 1) {
      for (let column = 0; column < 3; column += 1) {
        $(`#p-jac-${row}-${column}`).textContent = jacobian.matrix[row][column].toFixed(3);
      }
    }

  }

  function refresh() {
    const snapshot = updateScene();
    updateReadouts(snapshot);
  }

  /* ------------------------------------------------------------------ *
   * Actions
   * ------------------------------------------------------------------ */
  function stopDemo() {
    if (!animationEnabled) return;
    animationEnabled = false;
    $('#p-animate').checked = false;
  }

  function goHome() {
    stopDemo();
    if (mode === 'ik') {
      const forward = solvePlanarForward(JOINTS.map(({ home }) => home));
      elbow = JOINTS[1].home >= 0 ? 'up' : 'down';
      target = { x: forward.tip.x, y: forward.tip.y, phiDeg: forward.orientationDeg };
      targetClamped = false;
      solveTarget();
      syncTargetInputs();
      refresh();
      return;
    }
    angles = JOINTS.map(({ home }) => home);
    syncJointInputs();
    refresh();
  }

  function goRandom() {
    stopDemo();
    angles = JOINTS.map(({ min, max }) => THREE.MathUtils.randFloat(min * 0.7, max * 0.7));
    if (mode === 'ik') {
      const forward = solvePlanarForward(angles);
      target = { x: forward.tip.x, y: forward.tip.y, phiDeg: forward.orientationDeg };
      targetClamped = false;
      syncTargetInputs();
    }
    syncJointInputs();
    refresh();
  }

  /* ------------------------------------------------------------------ *
   * Pointer: drag a target on the floor, pan the camera
   * ------------------------------------------------------------------ */
  const raycaster = new THREE.Raycaster();
  const floorPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  const pointerNdc = new THREE.Vector2();
  let dragMode = null;
  let lastPan = { x: 0, y: 0 };

  function pointerToFloor(event) {
    const rect = canvas.getBoundingClientRect();
    pointerNdc.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.setFromCamera(pointerNdc, camera);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(floorPlane, hit)) return null;
    return hit;
  }

  canvas?.addEventListener('pointerdown', (event) => {
    canvas.setPointerCapture(event.pointerId);
    if (event.button === 1 || event.button === 2 || event.shiftKey) {
      dragMode = 'pan';
      lastPan = { x: event.clientX, y: event.clientY };
      canvas.style.cursor = 'grabbing';
      return;
    }
    const hit = pointerToFloor(event);
    if (!hit) return;
    stopDemo();
    dragMode = 'target';
    // Dragging on the floor always means "put the tool here".
    if (mode !== 'ik') setMode('ik', { silent: true });
    target = { ...target, x: hit.x, y: hit.y };
    solveTarget();
    refresh();
  });

  canvas?.addEventListener('pointermove', (event) => {
    if (!dragMode) return;
    if (dragMode === 'pan') {
      const scale = FRUSTUM_HEIGHT / camera.zoom / Math.max(1, canvas.clientHeight);
      pan.x -= (event.clientX - lastPan.x) * scale;
      pan.y += (event.clientY - lastPan.y) * scale;
      lastPan = { x: event.clientX, y: event.clientY };
      return;
    }
    const hit = pointerToFloor(event);
    if (!hit) return;
    target = { ...target, x: hit.x, y: hit.y };
    solveTarget();
    refresh();
  });

  function endDrag() {
    dragMode = null;
    if (renderer) renderer.domElement.style.cursor = mode === 'ik' ? 'crosshair' : 'grab';
  }
  canvas?.addEventListener('pointerup', endDrag);
  canvas?.addEventListener('pointercancel', endDrag);
  canvas?.addEventListener('contextmenu', (event) => event.preventDefault());

  let zoom = 1;
  canvas?.addEventListener('wheel', (event) => {
    event.preventDefault();
    zoom = clamp(zoom * Math.exp(-event.deltaY * 0.0012), 0.65, 3.4);
    camGoal.zoom = zoom;
  }, { passive: false });

  /* ------------------------------------------------------------------ *
   * View buttons, toggles, panel actions
   * ------------------------------------------------------------------ */
  root.querySelectorAll('[data-planar-view]').forEach((button) => {
    button.addEventListener('click', () => {
      activeView = button.dataset.planarView;
      root.querySelectorAll('[data-planar-view]').forEach((other) => {
        const selected = other.dataset.planarView === activeView;
        other.setAttribute('aria-pressed', String(selected));
        other.classList.toggle('is-active', selected);
      });
      camGoal.eye.copy(VIEWS[activeView].eye);
      camGoal.target.copy(VIEWS[activeView].target);
    });
  });

  root.querySelectorAll('[data-planar-mode]').forEach((button) => {
    button.addEventListener('click', () => {
      stopDemo();
      setMode(button.dataset.planarMode);
    });
  });

  root.querySelectorAll('[data-planar-elbow]').forEach((button) => {
    button.addEventListener('click', () => {
      elbow = button.dataset.planarElbow;
      stopDemo();
      if (mode === 'ik') {
        solveTarget();
        refresh();
      }
      updateModeUI();
    });
  });

  ['x', 'y', 'phi'].forEach((key) => {
    const input = $(`#p-target-${key}`);
    input.addEventListener('change', () => {
      const value = Number(input.value);
      if (!Number.isFinite(value)) {
        syncTargetInputs();
        return;
      }
      stopDemo();
      if (mode !== 'ik') setMode('ik', { silent: true });
      if (key === 'phi') target = { ...target, phiDeg: value };
      else target = { ...target, [key]: value };
      solveTarget();
      refresh();
    });
    input.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') input.blur();
    });
  });

  $('#p-copy-pose').addEventListener('click', () => {
    const forward = solvePlanarForward(angles);
    target = { x: forward.tip.x, y: forward.tip.y, phiDeg: forward.orientationDeg };
    targetClamped = false;
    if (mode !== 'ik') setMode('ik', { silent: true });
    syncTargetInputs();
    refresh();
  });

  $('#p-reset').addEventListener('click', goHome);
  $('#p-random').addEventListener('click', goRandom);

  $('#p-show-rings').addEventListener('change', (event) => {
    workspaceLayer.visible = event.target.checked;
  });
  $('#p-show-frames').addEventListener('change', (event) => {
    frameLayer.visible = event.target.checked;
    jointLabels.forEach(({ object }) => { object.visible = event.target.checked; });
  });
  $('#p-animate').addEventListener('change', (event) => {
    animationEnabled = event.target.checked;
    if (animationEnabled) animationStart = performance.now();
  });

  /* ------------------------------------------------------------------ *
   * Camera + sizing
   * ------------------------------------------------------------------ */
  function resize() {
    const width = Math.max(1, viewport.clientWidth);
    const height = Math.max(1, viewport.clientHeight);
    const halfHeight = FRUSTUM_HEIGHT / 2;
    const halfWidth = halfHeight * (width / height);
    camera.left = -halfWidth;
    camera.right = halfWidth;
    camera.top = halfHeight;
    camera.bottom = -halfHeight;
    camera.updateProjectionMatrix();
    renderer?.setSize(width, height, false);
    labelRenderer.setSize(width, height);
  }

  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(viewport);
  window.addEventListener('resize', resize, { passive: true });

  function updateCamera(delta) {
    const lerp = Math.min(1, delta * 0.55);
    camState.eye.lerp(camGoal.eye, lerp);
    camState.target.lerp(camGoal.target, lerp);
    camState.zoom += (camGoal.zoom - camState.zoom) * lerp;
    camera.zoom = camState.zoom;
    camera.position.set(camState.eye.x + pan.x, camState.eye.y + pan.y, camState.eye.z);
    camera.lookAt(camState.target.x + pan.x, camState.target.y + pan.y, camState.target.z);
    camera.updateProjectionMatrix();
  }

  /* ------------------------------------------------------------------ *
   * Demo motion
   * ------------------------------------------------------------------ */
  const amplitudes = [52, 58, 74];
  const frequencies = [0.42, 0.63, 0.88];
  const phases = [0.4, 1.9, 3.1];

  function stepDemo(time) {
    const t = (time - animationStart) / 1000;
    if (mode === 'fk') {
      angles = JOINTS.map(({ home, min, max }, index) =>
        clamp(home + amplitudes[index] * Math.sin(t * frequencies[index] + phases[index]), min, max));
      syncJointInputs();
    } else {
      const radius = 0.5;
      target = {
        x: radius * Math.cos(t * 0.52) + 0.1,
        y: 0.3 + radius * 0.85 * Math.sin(t * 0.68),
        phiDeg: 38 * Math.sin(t * 0.44) + 12,
      };
      targetClamped = false;
      solveTarget();
      syncTargetInputs();
    }
    refresh();
    lastAnimationUpdate = time;
  }

  /* ------------------------------------------------------------------ *
   * Boot
   * ------------------------------------------------------------------ */
  buildJointControls();
  buildStaticTables();
  target = {
    ...solvePlanarForward(angles).tip,
    phiDeg: solvePlanarForward(angles).orientationDeg,
  };
  syncTargetInputs();
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
