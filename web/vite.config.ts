import { defineConfig } from 'vite';
import tailwindcss from '@tailwindcss/vite';
import { compression } from 'vite-plugin-compression2';

export default defineConfig({
  // GitHub Pages 项目页挂在仓库子路径下（https://fooolion.github.io/2026_Digital_Twin/），
  // 本地 dev 时 BASE_URL 仍是 '/'，互不影响
  base: '/2026_Digital_Twin/',
  plugins: [
    tailwindcss(),
    // 传输压缩（GitHub Pages CDN 只压文本，wasm/STL 原样传输——构建时预压 .gz，
    // 前端 fetch .gz 后用 DecompressionStream 解压（sim.ts/home.ts 的 fetchBytes/fetchMesh））
    compression({ include: /\.(wasm|stl|xml|urdf)$/i, deleteOriginalAssets: false }),
  ],
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
