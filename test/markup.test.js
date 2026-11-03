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
  assert.deepEqual(tabs.sort(), ['planar', 'six']);
  assert.match(html, /id="panel-six"[^>]*role="tabpanel"/);
  assert.match(html, /id="panel-planar"[^>]*role="tabpanel"/);
  assert.match(html, /aria-controls="panel-six"/);
  assert.match(html, /aria-controls="panel-planar"/);
});

test('element ids are unique so each tab can scope its own queries', () => {
  const all = [...html.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]);
  const duplicates = all.filter((id, index) => all.indexOf(id) !== index);
  assert.deepEqual(duplicates, []);
  assert.ok(ids.has('six-viewport'));
  assert.ok(ids.has('planar-viewport'));
  assert.ok(ids.has('six-panel'));
  assert.ok(ids.has('planar-panel'));
});

test('the planar tab states its three degrees of freedom and floor mount', () => {
  const panel = html.slice(html.indexOf('id="panel-planar"'), html.indexOf('</main>'));
  assert.match(panel, /3<\/span> REVOLUTE DOF/);
  assert.match(panel, /FLOOR PLANE/);
  assert.match(panel, /data-planar-mode="fk"/);
  assert.match(panel, /data-planar-mode="ik"/);
  assert.match(panel, /data-planar-elbow="up"/);
  assert.match(panel, /data-planar-elbow="down"/);
  assert.match(panel, /id="p-target-x"/);
  assert.match(panel, /id="p-target-y"/);
  assert.match(panel, /id="p-target-phi"/);
});

test('every literal id queried by a view exists in the markup', () => {
  for (const file of ['src/planar-view.js', 'src/six-axis-view.js']) {
    const source = read(file);
    // Template selectors (`#p-slider-${index}`) are covered by the test below.
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
  assert.match(read('src/planar-view.js'), /id="p-slider-\$\{index\}"/);
  assert.match(read('src/planar-view.js'), /id="p-angle-\$\{index\}"/);
  assert.match(read('src/planar-view.js'), /id="p-link-angle-\$\{index\}"/);
  assert.match(read('src/six-axis-view.js'), /id="slider-\$\{index\}"/);
  assert.match(read('src/six-axis-view.js'), /id="angle-\$\{index\}"/);
  assert.match(read('src/six-axis-view.js'), /id="dh-theta-\$\{index\}"/);
});

test('main.js mounts both views and renders only the active tab', () => {
  const source = read('src/main.js');
  assert.match(source, /createSixAxisView\(panels\.six\)/);
  assert.match(source, /createPlanarView\(panels\.planar\)/);
  assert.match(source, /views\[activeTab\]\.tick/);
  assert.match(source, /ArrowRight/);
  assert.match(source, /ArrowLeft/);
});

test('the planar view exposes the mounting plane as a top-down capable scene', () => {
  const source = read('src/planar-view.js');
  assert.match(source, /OrthographicCamera/);
  assert.match(source, /data-planar-view/);
  assert.match(source, /new THREE\.Plane\(new THREE\.Vector3\(0, 0, 1\), 0\)/);
});

test('the README documents both tabs', () => {
  const readme = read('README.md');
  assert.match(readme, /Six-axis articulated arm/);
  assert.match(readme, /Planar arm on the floor/);
  assert.match(readme, /#planar/);
  assert.match(readme, /inverse kinematics/i);
});
