import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('..', import.meta.url));
const html = readFileSync(`${root}index.html`, 'utf8');
const read = (file) => readFileSync(`${root}${file}`, 'utf8');

const ids = new Set([...html.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]));

test('the page offers both models as tabs wired to their panels', () => {
  const tabs = [...html.matchAll(/data-tab="([^"]+)"[^>]*/g)].map((match) => match[1]);
  assert.deepEqual(tabs.sort(), ['six', 'spatial']);
  assert.match(html, /id="panel-six"[^>]*role="tabpanel"/);
  assert.match(html, /id="panel-spatial"[^>]*role="tabpanel"/);
  assert.match(html, /aria-controls="panel-six"/);
  assert.match(html, /aria-controls="panel-spatial"/);
});

test('element ids are unique so each tab can scope its own queries', () => {
  const all = [...html.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]);
  const duplicates = all.filter((id, index) => all.indexOf(id) !== index);
  assert.deepEqual(duplicates, []);
  assert.ok(ids.has('six-viewport'));
  assert.ok(ids.has('spatial-viewport'));
  assert.ok(ids.has('six-panel'));
  assert.ok(ids.has('spatial-panel'));
});

test('the second tab models the bench crane from the sketch', () => {
  const panel = html.slice(html.indexOf('id="panel-spatial"'), html.indexOf('</main>'));
  assert.match(panel, /3<\/span> REVOLUTE DOF/);
  assert.match(panel, /TWO PLATFORMS/);
  assert.match(panel, /yaw/i);
  assert.match(panel, /shoulder/i);
  assert.match(panel, /elbow/i);
  assert.match(panel, /data-spatial-mode="fk"/);
  assert.match(panel, /data-spatial-mode="ik"/);
  assert.match(panel, /data-spatial-view="bench"/);
  assert.match(panel, /data-spatial-view="side"/);
  assert.match(panel, /data-spatial-view="plan"/);
  assert.match(panel, /id="s-goal-x"/);
  assert.match(panel, /id="s-goal-y"/);
  assert.match(panel, /id="s-bench-state"/);
  assert.match(panel, /id="s-radial"/);
  assert.match(panel, /id="s-band-lower"/);
  assert.match(panel, /Platform A · mount/);
  // Both working surfaces are in the panel: the mount and the lower platform.
  assert.match(panel, /data-spatial-platform="mount"/);
  assert.match(panel, /data-spatial-platform="lower"/);
  assert.match(panel, /id="s-height-mount"/);
  assert.match(panel, /id="s-height-lower"/);
  assert.match(panel, /id="s-band-mount"/);
  assert.match(panel, /id="s-band-lower"/);
  assert.match(panel, /id="s-reach-lower"/);
  assert.match(panel, /id="s-goal-platform"/);
  // The abandoned two-plane wording is gone for good.
  assert.doesNotMatch(panel, /data-spatial-plane/);
  assert.doesNotMatch(panel, /working planes/i);
});

test('every literal id queried by a view exists in the markup', () => {
  for (const file of ['src/spatial-view.js', 'src/six-axis-view.js']) {
    const source = read(file);
    // Template selectors (`#s-slider-${index}`) are covered by the test below.
    const selectors = new Set(
      [...source.matchAll(/\$\(\s*[`'"]#([A-Za-z0-9_-]+)/g)]
        .map((match) => match[1])
        .filter((selector) => !selector.endsWith('-')),
    );
    assert.ok(selectors.size > 10, `${file} should query the panel it controls`);
    const missing = [...selectors].filter((selector) => !ids.has(selector));
    assert.deepEqual(missing, [], `${file} queries ids that are not in index.html`);
  }
});

test('both views inject the ids their dynamic selectors expect', () => {
  assert.match(read('src/spatial-view.js'), /id="s-slider-\$\{index\}"/);
  assert.match(read('src/spatial-view.js'), /id="s-angle-\$\{index\}"/);
  assert.match(read('src/spatial-view.js'), /id="s-frame-\$\{index\}"/);
  assert.match(read('src/spatial-view.js'), /id="s-jac-\$\{row\}-\$\{column\}"/);
  assert.match(read('src/six-axis-view.js'), /id="slider-\$\{index\}"/);
  assert.match(read('src/six-axis-view.js'), /id="angle-\$\{index\}"/);
  assert.match(read('src/six-axis-view.js'), /id="dh-theta-\$\{index\}"/);
});

test('main.js mounts both views and renders only the active tab', () => {
  const source = read('src/main.js');
  assert.match(source, /createSixAxisView\(panels\.six\)/);
  assert.match(source, /createCraneView\(panels\.spatial\)/);
  assert.match(source, /views\[activeTab\]\.tick/);
  assert.match(source, /ArrowRight/);
  assert.match(source, /ArrowLeft/);
});

test('the crane view offers three camera angles, including a side view of the A-frame', () => {
  const source = read('src/spatial-view.js');
  assert.match(source, /data-spatial-view/);
  assert.match(source, /PerspectiveCamera/);
  assert.match(source, /OrbitControls/);
  assert.match(source, /createCraneView/);
  // The arm is a real hinge chain: the elbow group is a child of the shoulder.
  assert.match(source, /pitchGroup\.add\(elbowGroup\)/);
  assert.match(read('index.html'), /data-spatial-view="side"/);
});

test('the README documents both tabs', () => {
  const readme = read('README.md');
  assert.match(readme, /Six-axis articulated arm/);
  assert.match(readme, /Bench crane arm/);
  assert.match(readme, /#spatial/);
  assert.match(readme, /inverse kinematics/i);
});
