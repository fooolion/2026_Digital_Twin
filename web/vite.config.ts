import { defineConfig } from 'vite';
import tailwindcss from '@tailwindcss/vite';

export default defineConfig({
  // GitHub Pages 项目页挂在仓库子路径下（https://fooolion.github.io/2026_Digital_Twin/），
  // 本地 dev 时 BASE_URL 仍是 '/'，互不影响
  base: '/2026_Digital_Twin/',
  plugins: [tailwindcss()],
  build: {
    rollupOptions: {
      input: {
        index: 'index.html',
        about: 'about.html',
        live: 'live.html',
        sim: 'sim.html',
        teleop: 'teleop.html',
        workshop: 'workshop.html',
      },
    },
  },
});
