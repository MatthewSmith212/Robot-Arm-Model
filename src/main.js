import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DObject, CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';
import { DH, matrixToRPYDegrees, solveForwardKinematics } from './kinematics.js';
import './style.css';

const JOINTS = [
  { name: 'Base swivel', axis: 'REVOLUTE ABOUT Z₀', min: -170, max: 170, home: 0 },
  { name: 'Shoulder pitch', axis: 'REVOLUTE ABOUT Z₁', min: -95, max: 105, home: 38 },
  { name: 'Elbow pitch', axis: 'REVOLUTE ABOUT Z₂', min: -135, max: 135, home: -55 },
  { name: 'Wrist roll', axis: 'REVOLUTE ABOUT Z₃', min: -180, max: 180, home: 0 },
  { name: 'Wrist pitch', axis: 'REVOLUTE ABOUT Z₄', min: -110, max: 110, home: 38 },
  { name: 'Tool roll', axis: 'REVOLUTE ABOUT Z₅', min: -180, max: 180, home: 0 },
];

const BASE_HEIGHT = 0.16;
const UP = new THREE.Vector3(0, 1, 0);
const WORLD_UP = new THREE.Vector3(0, 0, 1);
const viewport = document.querySelector('#viewport');
const panel = document.querySelector('#panel');
const sliderHost = document.querySelector('#sliders');
const dhBody = document.querySelector('#dh-table tbody');
const matrixBody = document.querySelector('#transform-matrix tbody');
const framesHost = document.querySelector('#frames');

let angles = JOINTS.map(({ home }) => home);
let animationEnabled = false;
let animationStart = 0;
let lastAnimationUpdate = 0;

const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b151e');
scene.fog = new THREE.Fog('#0b151e', 5.4, 12.5);

const camera = new THREE.PerspectiveCamera(38, 1, 0.015, 40);
camera.up.copy(WORLD_UP);
camera.position.set(1.65, -2.45, 1.62);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.15;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.domElement.setAttribute('aria-label', '3D model of a six degree of freedom servo-driven robotic arm');
renderer.domElement.setAttribute('role', 'img');
renderer.domElement.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;outline:none;';
viewport.prepend(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.classList.add('labels-layer');
labelRenderer.domElement.style.cssText = 'position:absolute;inset:0;overflow:hidden;pointer-events:none;z-index:2;';
viewport.appendChild(labelRenderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.075;
controls.target.set(0.22, 0, 0.61);
controls.minDistance = 1.15;
controls.maxDistance = 5.5;
controls.maxPolarAngle = Math.PI * 0.485;
controls.update();

const ambient = new THREE.HemisphereLight(0xd9f1ef, 0x16202a, 1.75);
scene.add(ambient);
const keyLight = new THREE.DirectionalLight(0xfff0d8, 3.0);
keyLight.position.set(-2.6, -3.3, 4.7);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(1024, 1024);
keyLight.shadow.camera.left = -2.3;
keyLight.shadow.camera.right = 2.3;
keyLight.shadow.camera.top = 2.3;
keyLight.shadow.camera.bottom = -2.3;
keyLight.shadow.bias = -0.0003;
scene.add(keyLight);
const rimLight = new THREE.DirectionalLight(0x58caba, 1.8);
rimLight.position.set(2.8, 2.1, 2.9);
scene.add(rimLight);
const fillLight = new THREE.PointLight(0x789fd4, 0.75, 6);
fillLight.position.set(0.4, -1.4, 1.7);
scene.add(fillLight);

const floorMaterial = new THREE.MeshStandardMaterial({ color: '#101e25', roughness: 0.93, metalness: 0.08 });
const floor = new THREE.Mesh(new THREE.PlaneGeometry(16, 16), floorMaterial);
floor.position.z = -0.014;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(4, 40, 0x3d665e, 0x213740);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.006;
grid.material.transparent = true;
grid.material.opacity = 0.43;
scene.add(grid);

const worldAxes = new THREE.AxesHelper(0.32);
worldAxes.position.set(-0.04, -0.04, 0.002);
scene.add(worldAxes);

const robot = new THREE.Group();
robot.position.z = BASE_HEIGHT;
scene.add(robot);

const materials = {
  housing: new THREE.MeshStandardMaterial({ color: '#25313a', metalness: 0.78, roughness: 0.32 }),
  shell: new THREE.MeshStandardMaterial({ color: '#bac7c6', metalness: 0.72, roughness: 0.25 }),
  shellLight: new THREE.MeshStandardMaterial({ color: '#d8e1db', metalness: 0.68, roughness: 0.23 }),
  dark: new THREE.MeshStandardMaterial({ color: '#111a20', metalness: 0.65, roughness: 0.35 }),
  accent: new THREE.MeshStandardMaterial({ color: '#f3925e', metalness: 0.52, roughness: 0.25, emissive: '#401907', emissiveIntensity: 0.22 }),
  accentSoft: new THREE.MeshStandardMaterial({ color: '#e2a076', metalness: 0.65, roughness: 0.3 }),
  teal: new THREE.MeshStandardMaterial({ color: '#5ee2bd', metalness: 0.38, roughness: 0.22, emissive: '#105943', emissiveIntensity: 0.65 }),
  fastener: new THREE.MeshStandardMaterial({ color: '#d3ddd8', metalness: 0.82, roughness: 0.19 }),
};

function makeCylinder(radiusTop, radiusBottom, height, material, radialSegments = 28) {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(radiusTop, radiusBottom, height, radialSegments, 1),
    material,
  );
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

function createBase() {
  const base = makeCylinder(0.22, 0.255, 0.12, materials.housing, 48);
  base.rotation.x = Math.PI / 2;
  base.position.z = 0.06;
  scene.add(base);

  const lowerTrim = makeCylinder(0.235, 0.235, 0.016, materials.accent, 48);
  lowerTrim.rotation.x = Math.PI / 2;
  lowerTrim.position.z = 0.025;
  scene.add(lowerTrim);

  const topPlate = makeCylinder(0.182, 0.202, 0.045, materials.shell, 48);
  topPlate.rotation.x = Math.PI / 2;
  topPlate.position.z = 0.137;
  scene.add(topPlate);

  const topRing = makeCylinder(0.16, 0.16, 0.012, materials.dark, 48);
  topRing.rotation.x = Math.PI / 2;
  topRing.position.z = 0.164;
  scene.add(topRing);

  const boltGeometry = new THREE.CylinderGeometry(0.012, 0.012, 0.008, 12);
  for (let i = 0; i < 6; i += 1) {
    const angle = (i / 6) * Math.PI * 2;
    const bolt = new THREE.Mesh(boltGeometry, materials.fastener);
    bolt.rotation.x = Math.PI / 2;
    bolt.position.set(Math.cos(angle) * 0.19, Math.sin(angle) * 0.19, 0.123);
    bolt.castShadow = true;
    scene.add(bolt);
  }
}

createBase();

const links = [];
const servos = [];
const frameHelpers = [];
const frameLabels = [];
const linkPalette = [materials.shell, materials.shellLight, materials.shell, materials.shellLight, materials.shell, materials.shellLight];

for (let i = 0; i < DH.length; i += 1) {
  const radius = i < 3 ? 0.052 : 0.043;
  const beam = makeCylinder(radius * 0.94, radius * 1.2, 1, linkPalette[i], 24);
  const band = makeCylinder(radius * 1.22, radius * 1.22, 1, i < 3 ? materials.accent : materials.accentSoft, 24);
  band.castShadow = true;
  robot.add(beam, band);
  links.push({ beam, band });
}

function createServo(index) {
  const group = new THREE.Group();
  const radius = index === 0 ? 0.124 : index < 3 ? 0.103 : 0.081;
  const thickness = index === 0 ? 0.108 : index < 3 ? 0.104 : 0.085;

  const can = makeCylinder(radius, radius * 0.94, thickness, materials.housing, 32);
  group.add(can);

  const outerRim = makeCylinder(radius * 0.91, radius * 0.91, 0.012, materials.accent, 32);
  outerRim.position.y = thickness * 0.5 + 0.005;
  group.add(outerRim);

  const face = makeCylinder(radius * 0.72, radius * 0.72, 0.016, materials.shell, 32);
  face.position.y = thickness * 0.5 + 0.014;
  group.add(face);

  const hub = makeCylinder(radius * 0.32, radius * 0.36, 0.025, materials.fastener, 24);
  hub.position.y = thickness * 0.5 + 0.031;
  group.add(hub);

  const hubCap = makeCylinder(radius * 0.17, radius * 0.17, 0.009, materials.accent, 20);
  hubCap.position.y = thickness * 0.5 + 0.048;
  group.add(hubCap);

  const screwGeometry = new THREE.SphereGeometry(0.008, 10, 8);
  for (const xSign of [-1, 1]) {
    for (const zSign of [-1, 1]) {
      const screw = new THREE.Mesh(screwGeometry, materials.fastener);
      screw.position.set(xSign * radius * 0.63, thickness * 0.5 + 0.014, zSign * radius * 0.63);
      screw.scale.set(1, 0.48, 1);
      group.add(screw);
    }
  }

  // A narrow encoder marker makes each rotary housing easy to read in the model.
  const encoder = new THREE.Mesh(new THREE.SphereGeometry(0.012, 12, 10), materials.teal);
  encoder.position.set(radius * 0.78, 0, 0);
  group.add(encoder);

  group.traverse((child) => {
    if (child.isMesh) {
      child.castShadow = true;
      child.receiveShadow = true;
    }
  });
  robot.add(group);
  servos.push(group);
}

JOINTS.forEach((_, index) => createServo(index));

for (let i = 0; i <= DH.length; i += 1) {
  const helper = new THREE.AxesHelper(i === 0 ? 0.15 : 0.12);
  helper.material.transparent = true;
  helper.material.opacity = 0.82;
  robot.add(helper);
  frameHelpers.push(helper);
}

for (let i = 0; i < JOINTS.length; i += 1) {
  const element = document.createElement('div');
  element.className = 'joint-label';
  element.innerHTML = `<b>J${i + 1}</b> &nbsp; 0°`;
  const label = new CSS2DObject(element);
  robot.add(label);
  frameLabels.push({ object: label, element });
}

const tool = new THREE.Group();
robot.add(tool);
const palm = makeBox(0.128, 0.108, 0.045, materials.housing, [0, 0, -0.119]);
tool.add(palm);
const palmPlate = makeBox(0.14, 0.11, 0.012, materials.accentSoft, [0, 0, -0.094]);
tool.add(palmPlate);
for (const side of [-1, 1]) {
  const finger = makeBox(0.024, 0.034, 0.108, materials.shellLight, [side * 0.054, 0, -0.054]);
  tool.add(finger);
  const fingerPad = makeBox(0.027, 0.039, 0.018, materials.accent, [side * 0.054, 0, -0.009]);
  tool.add(fingerPad);
}
const toolIndicator = new THREE.Mesh(new THREE.SphereGeometry(0.018, 18, 14), materials.teal);
tool.add(toolIndicator);

const jointControlRefs = [];
function buildControls() {
  sliderHost.innerHTML = JOINTS.map((joint, index) => `
    <div class="joint-control">
      <div class="joint-topline">
        <span class="joint-index">${String(index + 1).padStart(2, '0')}</span>
        <div class="joint-copy"><span class="joint-name">${joint.name}</span><span class="joint-axis">${joint.axis}</span></div>
        <label class="joint-angle-wrap">
          <input class="joint-angle-input" id="angle-${index}" type="number" min="${joint.min}" max="${joint.max}" step="0.5" value="${joint.home.toFixed(1)}" aria-label="Joint ${index + 1} angle in degrees" />
          <span>°</span>
        </label>
      </div>
      <input class="joint-range" id="slider-${index}" type="range" min="${joint.min}" max="${joint.max}" step="0.5" value="${joint.home}" aria-label="Joint ${index + 1}, ${joint.name}, angle in degrees" />
      <div class="range-limits"><span>${joint.min}°</span><span>${joint.max}°</span></div>
    </div>
  `).join('');

  JOINTS.forEach((_, index) => {
    const input = document.querySelector(`#slider-${index}`);
    const angleInput = document.querySelector(`#angle-${index}`);
    input.addEventListener('input', () => {
      stopDemo();
      angles[index] = Number(input.value);
      angleInput.value = angles[index].toFixed(1);
      updateModel();
    });
    angleInput.addEventListener('change', () => {
      stopDemo();
      const typedValue = angleInput.value.trim() === '' ? angles[index] : Number(angleInput.value);
      angles[index] = Number.isFinite(typedValue)
        ? THREE.MathUtils.clamp(typedValue, JOINTS[index].min, JOINTS[index].max)
        : angles[index];
      input.value = angles[index].toFixed(1);
      angleInput.value = angles[index].toFixed(1);
      updateModel();
    });
    angleInput.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') angleInput.blur();
    });
    jointControlRefs.push({ input, angleInput });
  });
}

function buildStaticTables() {
  dhBody.innerHTML = DH.map((row, index) => `
    <tr>
      <td>${index + 1}</td>
      <td id="dh-theta-${index}">0.0</td>
      <td>${row.d.toFixed(2)}</td>
      <td>${row.a.toFixed(2)}</td>
      <td>${row.alpha}</td>
    </tr>
  `).join('');

  matrixBody.innerHTML = Array.from({ length: 4 }, () => '<tr>' + Array.from({ length: 4 }, () => '<td>0.000</td>').join('') + '</tr>').join('');
  framesHost.innerHTML = Array.from({ length: 7 }, (_, index) => `
    <div class="frame-row"><span>F${index}</span><code id="frame-${index}">0.000, 0.000, 0.000</code></div>
  `).join('');
}

buildControls();
buildStaticTables();


function setBeam(mesh, start, end) {
  const direction = new THREE.Vector3().subVectors(end, start);
  const length = direction.length();
  mesh.visible = length > 0.0001;
  if (!mesh.visible) return { direction: UP, length: 0 };
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(UP, direction.clone().normalize());
  mesh.scale.set(1, length, 1);
  return { direction: direction.normalize(), length };
}

function setFrame(object, matrix) {
  object.position.setFromMatrixPosition(matrix);
  object.quaternion.setFromRotationMatrix(matrix);
}

function fmt(value, digits = 3) {
  const safe = Math.abs(value) < 0.0005 ? 0 : value;
  return safe.toFixed(digits);
}

function updatePoseReadout(transform) {
  const elements = transform.elements;
  const x = elements[12];
  const y = elements[13];
  const z = elements[14];
  document.querySelector('#pos-x').textContent = fmt(x);
  document.querySelector('#pos-y').textContent = fmt(y);
  document.querySelector('#pos-z').textContent = fmt(z);
  document.querySelector('#viewport-position').innerHTML = `X ${fmt(x)} <em>Y ${fmt(y)}</em> Z ${fmt(z)}`;

  const { roll, pitch, yaw } = matrixToRPYDegrees(transform);
  document.querySelector('#rpy-roll').textContent = `${roll.toFixed(1)}°`;
  document.querySelector('#rpy-pitch').textContent = `${pitch.toFixed(1)}°`;
  document.querySelector('#rpy-yaw').textContent = `${yaw.toFixed(1)}°`;

  const matrixRows = [];
  for (let row = 0; row < 4; row += 1) {
    const cells = [];
    for (let column = 0; column < 4; column += 1) {
      cells.push(`<td>${fmt(elements[column * 4 + row], 3)}</td>`);
    }
    matrixRows.push(`<tr>${cells.join('')}</tr>`);
  }
  matrixBody.innerHTML = matrixRows.join('');
}

function updateModel() {
  const { frames, jointOrigins, jointAxes, endTransform } = solveForwardKinematics(angles);

  for (let i = 0; i < DH.length; i += 1) {
    const start = new THREE.Vector3().setFromMatrixPosition(frames[i]);
    const end = new THREE.Vector3().setFromMatrixPosition(frames[i + 1]);
    const beamData = setBeam(links[i].beam, start, end);
    if (beamData.length > 0.0001) {
      const bandStart = start.clone().lerp(end, 0.78);
      links[i].band.visible = true;
      links[i].band.position.copy(bandStart);
      links[i].band.quaternion.setFromUnitVectors(UP, beamData.direction);
      links[i].band.scale.set(1, 0.022, 1);
    } else {
      links[i].band.visible = false;
    }

    servos[i].position.copy(jointOrigins[i]);
    servos[i].quaternion.setFromUnitVectors(UP, jointAxes[i]);
    document.querySelector(`#dh-theta-${i}`).textContent = angles[i].toFixed(1);

    frameLabels[i].element.innerHTML = `<b>J${i + 1}</b> &nbsp; ${angles[i].toFixed(0)}°`;
    frameLabels[i].object.position.copy(jointOrigins[i]);
    frameLabels[i].object.position.add(jointAxes[i].clone().multiplyScalar(0.12));
    frameLabels[i].object.position.z += 0.035;
  }

  for (let i = 0; i < frames.length; i += 1) {
    setFrame(frameHelpers[i], frames[i]);
    const position = new THREE.Vector3().setFromMatrixPosition(frames[i]);
    document.querySelector(`#frame-${i}`).textContent = `${fmt(position.x)}, ${fmt(position.y)}, ${fmt(position.z)}`;
  }

  setFrame(tool, endTransform);
  updatePoseReadout(endTransform);
}

function syncSliderValues() {
  angles.forEach((angle, index) => {
    const formatted = angle.toFixed(1);
    jointControlRefs[index].input.value = formatted;
    jointControlRefs[index].angleInput.value = formatted;
  });
}

function stopDemo() {
  if (!animationEnabled) return;
  animationEnabled = false;
  document.querySelector('#animate').checked = false;
}

function setHomePose() {
  stopDemo();
  angles = JOINTS.map(({ home }) => home);
  syncSliderValues();
  updateModel();
}

document.querySelector('#reset').addEventListener('click', setHomePose);
document.querySelector('#random').addEventListener('click', () => {
  stopDemo();
  angles = JOINTS.map(({ min, max }) => THREE.MathUtils.randFloat(min * 0.68, max * 0.68));
  syncSliderValues();
  updateModel();
});

document.querySelector('#showAxes').addEventListener('change', (event) => {
  worldAxes.visible = event.target.checked;
  grid.visible = event.target.checked;
});
document.querySelector('#showFrames').addEventListener('change', (event) => {
  frameHelpers.forEach((helper) => { helper.visible = event.target.checked; });
  frameLabels.forEach(({ object }) => { object.visible = event.target.checked; });
});
document.querySelector('#animate').addEventListener('change', (event) => {
  animationEnabled = event.target.checked;
  if (animationEnabled) animationStart = performance.now();
});

function resizeRenderer() {
  const width = Math.max(1, viewport.clientWidth);
  const height = Math.max(1, viewport.clientHeight);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
  labelRenderer.setSize(width, height);
}

const resizeObserver = new ResizeObserver(resizeRenderer);
resizeObserver.observe(viewport);
window.addEventListener('resize', resizeRenderer, { passive: true });
resizeRenderer();
updateModel();

function animate(time) {
  requestAnimationFrame(animate);

  if (animationEnabled && time - lastAnimationUpdate > 28) {
    const t = (time - animationStart) / 1000;
    const amplitudes = [55, 22, 36, 82, 35, 110];
    const frequencies = [0.47, 0.62, 0.52, 0.78, 0.67, 0.84];
    const phases = [0, 0.9, 2.1, 1.45, 2.75, 0.3];
    angles = JOINTS.map(({ home, min, max }, index) => {
      const angle = home + amplitudes[index] * Math.sin(t * frequencies[index] + phases[index]);
      return THREE.MathUtils.clamp(angle, min, max);
    });
    syncSliderValues();
    updateModel();
    lastAnimationUpdate = time;
  }

  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
}
requestAnimationFrame(animate);
