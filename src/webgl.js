import * as THREE from 'three';

/**
 * PCFSoftShadowMap was removed from three.js r186 (it falls back to
 * PCFShadowMap with a console warning), so use the supported soft filter.
 */
export const SHADOW_MAP_TYPE = THREE.PCFShadowMap;

/**
 * Create a WebGL renderer, or return null when the browser cannot provide a
 * context (no WebGL, blocklisted GPU, software rendering disabled). Callers
 * show a readable fallback instead of letting the page fail silently.
 */
export function createRenderer(options) {
  try {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('webgl2', { alpha: options.alpha === true, antialias: options.antialias !== false })
      || canvas.getContext('webgl', { alpha: options.alpha === true, antialias: options.antialias !== false });
    if (!context) return null;
    return new THREE.WebGLRenderer({ ...options, canvas, context });
  } catch (error) {
    console.warn('WebGL is unavailable; the 3D view cannot start.', error);
    return null;
  }
}

/** Replace a viewport's contents with a short explanation of what failed. */
export function showRendererFallback(viewport, { title = '3D view unavailable', detail } = {}) {
  const panel = document.createElement('div');
  panel.className = 'render-fallback';
  panel.setAttribute('role', 'alert');
  panel.innerHTML = `
    <h3>${title}</h3>
    <p>${detail || 'This browser could not create a WebGL context. Enable hardware acceleration or try another browser.'}</p>
    <p class="render-fallback-actions">
      <button type="button" class="button button-secondary">Reload the page</button>
    </p>
  `;
  panel.querySelector('button').addEventListener('click', () => window.location.reload());
  viewport.appendChild(panel);
  return panel;
}
