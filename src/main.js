import { createSixAxisView } from './six-axis-view.js';
import { createPlanarView } from './planar-view.js';
import './style.css';

const tabs = Array.from(document.querySelectorAll('[data-tab]'));
const panels = {
  six: document.querySelector('#panel-six'),
  planar: document.querySelector('#panel-planar'),
};
const activeModel = document.querySelector('#active-model');

let activeTab = 'six';

const views = {
  six: createSixAxisView(panels.six),
  planar: createPlanarView(panels.planar),
};

function selectTab(name, { focus = false } = {}) {
  if (!panels[name] || name === activeTab) return;
  activeTab = name;

  tabs.forEach((tab) => {
    const selected = tab.dataset.tab === name;
    tab.classList.toggle('is-active', selected);
    tab.setAttribute('aria-selected', String(selected));
    tab.tabIndex = selected ? 0 : -1;
    if (selected) {
      activeModel.textContent = tab.dataset.model;
      if (focus) tab.focus();
    }
  });

  Object.entries(panels).forEach(([key, panel]) => {
    const selected = key === name;
    panel.classList.toggle('is-active', selected);
    panel.hidden = !selected;
  });

  // Layout depends on the panel being visible; remeasure once it is on screen.
  requestAnimationFrame(() => views[name].onActivate?.());
}

tabs.forEach((tab) => {
  tab.addEventListener('click', () => selectTab(tab.dataset.tab));
  tab.addEventListener('keydown', (event) => {
    if (event.key !== 'ArrowRight' && event.key !== 'ArrowLeft') return;
    event.preventDefault();
    const index = tabs.indexOf(tab);
    const next = tabs[(index + (event.key === 'ArrowRight' ? 1 : tabs.length - 1)) % tabs.length];
    selectTab(next.dataset.tab, { focus: true });
  });
});

// Keep the address hash in step so a tab can be linked or bookmarked.
const initialTab = window.location.hash === '#planar' ? 'planar' : 'six';
if (initialTab !== activeTab) selectTab(initialTab);
tabs.forEach((tab) => {
  tab.addEventListener('click', () => {
    window.history.replaceState(null, '', `#${tab.dataset.tab}`);
  });
});

// Only the visible tab renders and animates, so the hidden arm costs nothing.
let lastTime = performance.now();
function frame(time) {
  requestAnimationFrame(frame);
  const delta = Math.min(0.1, (time - lastTime) / 1000);
  lastTime = time;
  views[activeTab].tick(time, delta);
}
requestAnimationFrame(frame);
