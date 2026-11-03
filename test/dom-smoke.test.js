import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { register } from 'node:module';
import { fileURLToPath } from 'node:url';

/**
 * Boots the real application (index.html + main.js + both views) inside jsdom
 * with a stubbed WebGL context. This catches runtime wiring errors that static
 * checks miss — for example a CSS2D label addressed through the wrong property,
 * which used to throw during mount and leave the 3D viewport blank.
 */

const root = fileURLToPath(new URL('..', import.meta.url));

// Vite resolves `import './style.css'`; give plain Node the same behaviour.
register('./support/css-hook.mjs', import.meta.url);

function fakeGL(canvas) {
  const constants = {};
  let next = 1;
  return new Proxy({ canvas }, {
    get(target, prop) {
      if (prop in target) return target[prop];
      if (typeof prop === 'string' && /^[A-Z0-9_]+$/.test(prop)) {
        if (!(prop in constants)) constants[prop] = next += 1;
        return constants[prop];
      }
      return (...args) => {
        switch (prop) {
          case 'getParameter': {
            const parameter = args[0];
            if (parameter === constants.VERSION) return 'WebGL 2.0';
            if (parameter === constants.SHADING_LANGUAGE_VERSION) return 'WebGL GLSL ES 3.00';
            if ([constants.SCISSOR_BOX, constants.VIEWPORT].includes(parameter)) return new Int32Array([0, 0, 800, 600]);
            if (parameter === constants.MAX_ELEMENT_INDEX) return 4294967295;
            return 4096;
          }
          case 'getShaderPrecisionFormat':
            return { rangeMin: 127, rangeMax: 127, precision: 23 };
          case 'getExtension':
          case 'getActiveAttrib':
          case 'getActiveUniform':
            return null;
          case 'getSupportedExtensions':
            return [];
          case 'getShaderParameter':
            return true;
          case 'getProgramParameter': {
            const parameter = args[1];
            const counts = [constants.ACTIVE_UNIFORMS, constants.ACTIVE_ATTRIBUTES, constants.ACTIVE_UNIFORM_BLOCKS];
            return counts.includes(parameter) ? 0 : true;
          }
          case 'getShaderInfoLog':
          case 'getProgramInfoLog':
            return '';
          case 'getError':
          case 'getAttribLocation':
            return 0;
          case 'getUniformLocation':
            return {};
          case 'getAttachedShaders':
            return [];
          case 'isContextLost':
            return false;
          case 'getContextAttributes':
            return { alpha: true, depth: true, antialias: true };
          case 'checkFramebufferStatus':
            return constants.FRAMEBUFFER_COMPLETE;
          default:
            return prop.startsWith('create') ? { id: (next += 1) } : undefined;
        }
      };
    },
    set(target, prop, value) {
      target[prop] = value;
      return true;
    },
  });
}

test('the app boots with both views mounted and no runtime errors', async (t) => {
  const { JSDOM } = await import('jsdom');
  const dom = new JSDOM(readFileSync(`${root}index.html`, 'utf8'), { pretendToBeVisual: true, url: 'http://localhost:5173/' });
  const { window } = dom;

  const restore = [];
  const override = (key, value) => {
    const had = Object.prototype.hasOwnProperty.call(globalThis, key);
    const previous = globalThis[key];
    Object.defineProperty(globalThis, key, { value, configurable: true, writable: true });
    restore.push(() => {
      if (had) Object.defineProperty(globalThis, key, { value: previous, configurable: true, writable: true });
      else delete globalThis[key];
    });
  };

  override('window', window);
  override('self', window);
  override('document', window.document);
  override('navigator', window.navigator);
  for (const key of ['HTMLElement', 'HTMLCanvasElement', 'HTMLImageElement', 'Image', 'Element', 'Node', 'Event', 'MouseEvent', 'WheelEvent']) {
    if (window[key]) override(key, window[key]);
  }

  for (const property of ['clientWidth', 'clientHeight']) {
    Object.defineProperty(window.Element.prototype, property, { get() { return 800; }, configurable: true });
  }
  window.Element.prototype.getBoundingClientRect = () => ({ x: 0, y: 0, top: 0, left: 0, right: 900, bottom: 640, width: 900, height: 640 });
  window.Element.prototype.setPointerCapture = () => {};
  window.Element.prototype.releasePointerCapture = () => {};
  window.ResizeObserver = class { observe() {} unobserve() {} disconnect() {} };
  override('ResizeObserver', window.ResizeObserver);

  let queue = [];
  const raf = (callback) => { queue.push(callback); return queue.length; };
  override('requestAnimationFrame', raf);
  window.requestAnimationFrame = raf;
  let clock = 0;
  const tick = (frames = 2, step = 20) => {
    for (let i = 0; i < frames; i += 1) {
      const pending = queue;
      queue = [];
      clock += step;
      pending.forEach((callback) => callback(clock));
    }
  };

  const context2d = () => new Proxy(
    { measureText: () => ({ width: 10 }), createRadialGradient: () => ({ addColorStop() {} }), createLinearGradient: () => ({ addColorStop() {} }) },
    { get: (target, prop) => (prop in target ? target[prop] : () => {}), set: (target, prop, value) => { target[prop] = value; return true; } },
  );
  window.HTMLCanvasElement.prototype.getContext = function (type) {
    if (/webgl/.test(type)) return fakeGL(this);
    return context2d();
  };

  const errors = [];
  window.addEventListener('error', (event) => errors.push(String(event.error?.stack || event.message)));
  process.on('unhandledRejection', (error) => errors.push(String(error?.stack || error)));

  t.after(() => restore.reverse().forEach((undo) => undo()));

  await import(`${root}src/main.js`);
  tick(3);

  const document = window.document;
  const $ = (selector) => document.querySelector(selector);

  assert.deepEqual(errors, [], 'mounting the app must not throw');
  assert.ok($('#six-viewport canvas'), 'the six-axis viewport gets a canvas');
  assert.ok($('#spatial-viewport canvas'), 'the shoulder-elbow viewport gets a canvas');
  assert.equal(document.querySelectorAll('#s-sliders .joint-control').length, 3, 'the three joints get controls');
  assert.equal(document.querySelectorAll('#dh-table tbody tr').length, 6, 'the DH table is still built');

  // The home pose is the sketch's A-frame: elbow up, tool back down on the bench.
  assert.equal($('#s-bench-state').textContent, 'ON PLATFORM A');
  assert.ok(Number($('#s-elbow-height').textContent.replace(/[^0-9.]/g, '')) > 0.2);
  assert.equal($('#s-pos-z').textContent, '0.740');

  // Both platforms are reachable, and the readouts said so on boot.
  assert.equal($('#s-reach-mount').textContent, 'REACHABLE');
  assert.equal($('#s-reach-lower').textContent, 'REACHABLE');
  assert.equal($('#s-height-lower').textContent, '0.45 m');

  // Switching tabs must not throw, even though the six-axis view is hidden.
  $('#tab-spatial').click();
  tick(2);
  assert.equal($('[data-tab].is-active').dataset.tab, 'spatial');

  // Goal control solves the joints for a point on the bench.
  $('[data-spatial-mode="ik"]').click();
  const goalX = $('#s-goal-x');
  const goalY = $('#s-goal-y');
  goalX.value = '0.45';
  goalX.dispatchEvent(new window.Event('change', { bubbles: true }));
  goalY.value = '0.15';
  goalY.dispatchEvent(new window.Event('change', { bubbles: true }));
  tick(2);
  assert.equal($('#s-reach-status').textContent, 'GOAL IN REACH');
  assert.equal($('#s-pos-x').textContent, '0.450');
  assert.equal($('#s-pos-y').textContent, '0.150');
  assert.equal($('#s-pos-z').textContent, '0.740');
  assert.equal($('#s-bench-state').textContent, 'ON PLATFORM A');

  // A point off the end of the arm is clamped to the nearest pose, not thrown.
  goalX.value = '4';
  goalX.dispatchEvent(new window.Event('change', { bubbles: true }));
  tick(2);
  assert.equal($('#s-reach-status').textContent, 'NEAREST POSE');
  assert.ok(Math.hypot(Number($('#s-pos-x').textContent), Number($('#s-pos-y').textContent)) < 0.9);

  // Clicking a platform moves the goal; dragging must not.
  $('[data-spatial-platform="mount"]').click();
  goalX.value = '0.4';
  goalX.dispatchEvent(new window.Event('change', { bubbles: true }));
  goalY.value = '0';
  goalY.dispatchEvent(new window.Event('change', { bubbles: true }));
  tick(2);
  const canvas = $('#spatial-viewport canvas');
  const beforeDrag = goalX.value;
  canvas.dispatchEvent(new window.MouseEvent('pointerdown', { bubbles: true, clientX: 500, clientY: 400, button: 0 }));
  canvas.dispatchEvent(new window.MouseEvent('pointerup', { bubbles: true, clientX: 570, clientY: 450, button: 0 }));
  tick(2);
  assert.equal(goalX.value, beforeDrag, 'an orbit drag must not move the goal');

  // The three camera presets are wired, including the side view of the A-frame.
  ['bench', 'side', 'plan'].forEach((view) => {
    $(`[data-spatial-view="${view}"]`).click();
    tick(2);
    assert.equal($('[data-spatial-view].is-active').dataset.spatialView, view);
  });

  // Home and randomise keep the arm inside its joint travel.
  $('#s-random').click();
  tick(2);
  document.querySelectorAll('#s-sliders .joint-control').forEach((control) => {
    const slider = control.querySelector('.joint-range');
    const value = Number(slider.value);
    assert.ok(value >= Number(slider.min) - 1e-9 && value <= Number(slider.max) + 1e-9, 'random pose stays in travel');
  });
  $('#s-reset').click();
  tick(2);
  assert.equal($('#s-pos-z').textContent, '0.740');

  // The scene graph itself must agree with the kinematics: the hinge chain in
  // three.js has to put the tool where the maths says it is.
  const lab = window.__robotLab;
  assert.ok(lab, 'main.js exposes the views for verification');
  const { scene } = lab.views.spatial;
  const { Vector3 } = await import('three');
  const observed = new Vector3().setFromMatrixPosition(scene.toolGroup.matrixWorld);
  const { solveForward, BENCH } = await import(`${root}src/crane-kinematics.js`);
  const predicted = solveForward(scene.angles).tip;
  for (const axis of ['x', 'y', 'z']) {
    assert.ok(Math.abs(observed[axis] - predicted[axis]) < 1e-6, `tool ${axis} matches the kinematics`);
  }
  assert.equal(scene.elbowGroup.parent, scene.pitchGroup, 'the elbow hinge hangs off the shoulder');
  assert.equal(scene.pitchGroup.parent, scene.yawGroup, 'the shoulder hinge sits on the yaw turret');
  assert.ok(scene.platformGroups.mount && scene.platformGroups.lower, 'both platforms are in the scene');
  // Home pose: tool on the bench top, elbow a good hand's width above it.
  assert.ok(Math.abs(observed.z - BENCH.height) < 1e-6, 'the home tool point rests on the bench');
  const elbowWorld = new Vector3().setFromMatrixPosition(scene.elbowGroup.matrixWorld);
  assert.ok(elbowWorld.z - BENCH.height > 0.2, 'the elbow is thrown above the bench');

  // Back to the six-axis tab; sliders and toggles still work.
  $('#tab-six').click();
  tick(2);
  const slider = $('#slider-1');
  slider.value = '60';
  slider.dispatchEvent(new window.Event('input', { bubbles: true }));
  tick(2);
  assert.equal($('#angle-1').value, '60.0');
  assert.deepEqual(errors, [], 'interacting with either tab must not throw');
});
