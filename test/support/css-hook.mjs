// Vite turns `import './style.css'` into an injected style element; under plain
// Node the same import only needs to resolve to an empty module.
export async function load(url, context, nextLoad) {
  if (url.endsWith('.css')) {
    return { format: 'module', source: 'export default {};', shortCircuit: true };
  }
  return nextLoad(url, context);
}
