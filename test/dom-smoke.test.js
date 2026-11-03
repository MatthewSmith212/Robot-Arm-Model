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
  assert.ok($('#planar-viewport canvas'), 'the planar viewport gets a canvas');
  assert.equal(document.querySelectorAll('#p-sliders .joint-control').length, 3, 'planar joint controls are built');
  assert.equal(document.querySelectorAll('#dh-table tbody tr').length, 6, 'the DH table is still built');

  // Planar readouts come from the forward solution, and the tool stays in-plane.
  assert.equal($('#p-height').textContent, '0.000 m');
  assert.notEqual($('#p-pos-x').textContent, '0.000');

  // Switching tabs must not throw, even though the six-axis view is hidden.
  $('#tab-planar').click();
  tick(2);
  assert.equal($('[data-tab].is-active').dataset.tab, 'planar');

  // Goal-pose control solves joints and reports reachability.
  $('[data-planar-mode="ik"]').click();
  const targetX = $('#p-target-x');
  targetX.value = '0.5';
  targetX.dispatchEvent(new window.Event('change', { bubbles: true }));
  const targetY = $('#p-target-y');
  targetY.value = '0.3';
  targetY.dispatchEvent(new window.Event('change', { bubbles: true }));
  tick(2);
  assert.equal($('#p-reach-status').textContent, 'TARGET IN REACH');

  targetX.value = '4';
  targetX.dispatchEvent(new window.Event('change', { bubbles: true }));
  tick(2);
  assert.equal($('#p-reach-status').textContent, 'CLAMPED TO WORKSPACE');

  // Dragging on the floor plane moves the goal.
  const canvas = $('#planar-viewport canvas');
  canvas.dispatchEvent(new window.MouseEvent('pointerdown', { bubbles: true, clientX: 430, clientY: 300, button: 0 }));
  canvas.dispatchEvent(new window.MouseEvent('pointermove', { bubbles: true, clientX: 470, clientY: 320, button: 0 }));
  canvas.dispatchEvent(new window.MouseEvent('pointerup', { bubbles: true, clientX: 470, clientY: 320, button: 0 }));
  tick(2);

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
