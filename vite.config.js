import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    host: '0.0.0.0',
    allowedHosts: true,
  },
  preview: {
    host: '0.0.0.0',
    allowedHosts: true,
  },
  // The Three.js viewer is ~144 kB gzipped; allow its normal uncompressed vendor size.
  build: {
    chunkSizeWarningLimit: 650,
  },
});
