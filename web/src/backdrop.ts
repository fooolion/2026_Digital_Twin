/**
 * 全页环境背景 · 方案3+6（粒子星野 + 地平线厂房剪影）——总览/零件库/仿真规划三页统一。
 * 挂载进各页 .ambient 层（缺则自动补建：网格底 + 光斑，再叠加本模块的粒子与地平线）。
 * 粒子 ~90 颗低速漂移+微闪（rAF，页面隐藏自动停）；prefers-reduced-motion 时静帧不动画。
 * 另提供 buildHorizonTexture() / mountSceneHorizon()：给全屏 three.js 页（仿真页）与
 * 3D 预览视口（总览 URDF / 零件库）做"场景内地平线"——页面级背景对全屏画布不可见。
 * Rev: P2 · 2026-10-29
 */
import * as THREE from 'three';

/** 厂房剪影：锯齿屋顶厂房 ×2 + 烟囱 + 立罐 + 龙门吊 + 低库，少量绿色"窗灯"点缀 */
const HORIZON_SVG = `
<svg viewBox="0 0 1440 140" preserveAspectRatio="none" aria-hidden="true">
  <g fill="rgba(22,38,66,.55)">
    <path d="M0 140 V96 h90 v-18 h70 v18 h120 V70 h110 v70 h180 V88 h130 v-16 h80 v16 h150 V78 h140 v62 z"/>
  </g>
  <g fill="#0d1930">
    <path d="M40 140 V92 l36 -22 l34 22 v-4 h8 v-18 l36 -22 l34 22 V140 Z"/>
    <rect x="238" y="34" width="16" height="106"/>
    <path d="M300 140 V78 a26 26 0 0 1 52 0 V140 Z"/>
    <path d="M430 140 V56 h210 v10 h-14 v74 h-12 V66 h-158 v74 h-12 V66 h-14 Z"/>
    <rect x="700" y="96" width="260" height="44"/>
    <path d="M1010 140 V84 l36 -22 l34 22 v-6 h10 V60 l36 -22 l34 22 v14 h12 V140 Z"/>
    <rect x="1190" y="52" width="14" height="88"/>
  </g>
  <g fill="rgba(57,217,138,.16)">
    <rect x="86" y="104" width="5" height="7"/><rect x="120" y="104" width="5" height="7"/>
    <rect x="740" y="110" width="5" height="7"/><rect x="800" y="110" width="5" height="7"/>
    <rect x="1240" y="96" width="5" height="7"/>
  </g>
  <g fill="rgba(79,216,255,.14)">
    <rect x="452" y="70" width="5" height="7"/><rect x="1092" y="70" width="5" height="7"/>
  </g>
</svg>`;

/** 地平线纹理（共享）：竖直渐变（顶边透明渐隐）+ 辉光线 + 厂房剪影坐于辉光线上。
 *  剪影路径为 y 向下坐标（地面 y=140），直接正缩放绘制即正立——此前用负缩放导致
 *  剪影上下颠倒（用户实测）。 */
export function buildHorizonTexture(): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = 2048; cv.height = 640;
  const c = cv.getContext('2d')!;
  const g = c.createLinearGradient(0, 0, 0, 640);
  g.addColorStop(0, 'rgba(10,17,32,0)'); // 顶边透明渐隐：幕布上缘融入页面背景，不见接缝
  g.addColorStop(0.45, '#0d1c33');
  g.addColorStop(0.72, '#14304f'); g.addColorStop(0.78, '#0e2138'); g.addColorStop(1, '#0b1526');
  c.fillStyle = g; c.fillRect(0, 0, 2048, 640);
  const lg = c.createLinearGradient(0, 0, 2048, 0);
  lg.addColorStop(0, 'rgba(120,220,255,0)'); lg.addColorStop(0.5, 'rgba(140,225,255,.55)');
  lg.addColorStop(1, 'rgba(57,217,138,0)');
  c.fillStyle = lg; c.fillRect(0, 452, 2048, 2);
  // 剪影：正缩放、基线（路径 y=140）落在辉光线上（306+147=453），形状向上展开
  c.save();
  c.setTransform(2048 / 1440, 0, 0, 1.05, 0, 453 - 140 * 1.05);
  c.fillStyle = 'rgba(22,38,66,.55)';
  c.fill(new Path2D('M0 140 V96 h90 v-18 h70 v18 h120 V70 h110 v70 h180 V88 h130 v-16 h80 v16 h150 V78 h140 v62 z'));
  c.fillStyle = '#0c1830';
  c.fill(new Path2D('M40 140 V92 l36 -22 l34 22 v-4 h8 v-18 l36 -22 l34 22 V140 Z'));
  c.fill(new Path2D('M238 34 h16 v106 h-16 Z'));
  c.fill(new Path2D('M300 140 V78 a26 26 0 0 1 52 0 V140 Z'));
  c.fill(new Path2D('M430 140 V56 h210 v10 h-14 v74 h-12 V66 h-158 v74 h-12 V66 h-14 Z'));
  c.fill(new Path2D('M700 96 h260 v44 h-260 Z'));
  c.fill(new Path2D('M1010 140 V84 l36 -22 l34 22 v-6 h10 V60 l36 -22 l34 22 v14 h12 V140 Z'));
  c.fill(new Path2D('M1190 52 h14 v88 h-14 Z'));
  c.fillStyle = 'rgba(57,217,138,.16)';
  for (const [wx, wy] of [[86, 104], [120, 104], [740, 110], [800, 110], [1240, 96]] as const) {
    c.fillRect(wx, wy, 5, 7);
  }
  c.fillStyle = 'rgba(79,216,255,.14)';
  for (const [wx, wy] of [[452, 70], [1092, 70]] as const) {
    c.fillRect(wx, wy, 5, 7);
  }
  c.restore();
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

let dotTex: THREE.Texture | null = null;
/** 圆形光点贴图（Points 默认渲染为方块——用径向渐变圆点更自然） */
export function dotTexture(): THREE.Texture {
  if (dotTex) return dotTex;
  const cv = document.createElement('canvas'); cv.width = cv.height = 64;
  const c = cv.getContext('2d')!;
  const g = c.createRadialGradient(32, 32, 0, 32, 32, 30);
  g.addColorStop(0, 'rgba(255,255,255,1)'); g.addColorStop(0.45, 'rgba(255,255,255,.55)');
  g.addColorStop(1, 'rgba(255,255,255,0)');
  c.fillStyle = g; c.fillRect(0, 0, 64, 64);
  dotTex = new THREE.CanvasTexture(cv);
  return dotTex;
}

export interface SceneHorizonOpts {
  /** 幕布到目标点的水平距离（m） */ distance?: number;
  /** 幕布宽/高（m） */ width?: number; height?: number;
  /** 辉光线（地平线）的世界 z 高度 */ glowZ?: number;
  /** 尘埃粒子数（0 = 不加） */ dust?: number;
}

/** 场景内地平线（3D 预览视口用）：透明渲染器 + 该幕布立于模型后方，正对相机初始位；
 *  附带少量尘埃粒子。相机为 OrbitControls 时的静态幕布（初始方位即构图）。 */
export function mountSceneHorizon(
  scene: THREE.Scene,
  camera: THREE.Camera,
  opts: SceneHorizonOpts = {},
): void {
  const { distance = 1.5, width = 2.6, height = 0.9, glowZ = 0.05, dust = 80 } = opts;
  const tex = buildHorizonTexture();
  const backdrop = new THREE.Mesh(
    new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({ map: tex }),
  );
  // 幕布立于目标点另一侧（模型背后、面向相机），辉光线（纹理 70.8% 高度处）对齐 glowZ
  const cp = camera.position;
  const hl = Math.hypot(cp.x, cp.y) || 1;
  backdrop.position.set(-cp.x / hl * distance, -cp.y / hl * distance, glowZ + height * 0.208);
  backdrop.up.set(0, 0, 1);
  backdrop.lookAt(cp.x, cp.y, glowZ);
  scene.add(backdrop);
  if (dust > 0) {
    const pos = new Float32Array(dust * 3);
    for (let i = 0; i < dust; i++) {
      pos[i * 3] = (Math.random() - 0.5) * width * 1.1;
      pos[i * 3 + 1] = (Math.random() - 0.5) * distance * 0.9;
      pos[i * 3 + 2] = Math.random() * height * 0.9;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const points = new THREE.Points(geo, new THREE.PointsMaterial({
      color: 0x9fd4ff, size: 0.006, transparent: true, opacity: 0.5, map: dotTexture(),
      blending: THREE.AdditiveBlending, depthWrite: false, sizeAttenuation: true,
    }));
    scene.add(points);
  }
}

interface P { x: number; y: number; r: number; vx: number; vy: number; a: number; ph: number; g: boolean; }

/** 页面级背景（总览/零件库等 DOM 页）：挂载进 .ambient——地平线剪影 + 粒子星野。 */
export function mountBackdrop(): void {
  let amb = document.querySelector('.ambient');
  if (!amb) {
    amb = document.createElement('div');
    amb.className = 'ambient';
    amb.setAttribute('aria-hidden', 'true');
    amb.innerHTML = '<div class="grid-bg"></div><div class="orb orb-b"></div>';
    document.body.prepend(amb);
  }
  if (amb.querySelector('.fx-particles')) return; // 幂等

  // 地平线层（厂房剪影 + 屋顶光带）
  const hz = document.createElement('div');
  hz.className = 'horizon';
  hz.innerHTML = `${HORIZON_SVG}<div class="hz-glow"></div>`;
  amb.appendChild(hz);

  // 粒子星野层
  const cv = document.createElement('canvas');
  cv.className = 'fx-particles';
  amb.appendChild(cv);

  const ctx = cv.getContext('2d');
  if (!ctx) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
  let W = 0, H = 0, P: P[] = [];

  const seed = (): void => {
    W = cv.clientWidth; H = cv.clientHeight;
    cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const n = Math.max(50, Math.min(130, Math.round((W * H) / 16000)));
    P = Array.from({ length: n }, () => ({
      x: Math.random() * W, y: Math.random() * H,
      r: Math.random() * 1.3 + 0.4,
      vx: (Math.random() - 0.5) * 0.16, vy: (Math.random() - 0.5) * 0.10,
      a: Math.random() * 0.45 + 0.12, ph: Math.random() * Math.PI * 2,
      g: Math.random() < 0.16, // 少量绿色点缀，呼应强调色
    }));
  };
  seed();
  window.addEventListener('resize', seed);

  if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
    draw(0); // 静帧：只画一帧，不做动画
    return;
  }
  let t = 0;
  (function loop(): void {
    t += 0.016;
    draw(t);
    requestAnimationFrame(loop);
  })();

  function draw(t: number): void {
    ctx!.clearRect(0, 0, W, H);
    for (const p of P) {
      if (t > 0) {
        p.x = (p.x + p.vx + W) % W;
        p.y = (p.y + p.vy + H) % H;
      }
      const tw = 0.72 + 0.28 * Math.sin(t * 1.4 + p.ph);
      ctx!.beginPath();
      ctx!.arc(p.x, p.y, p.r, 0, 7);
      const [r, g, b] = p.g ? [57, 217, 138] : [150, 210, 255];
      ctx!.fillStyle = `rgba(${r},${g},${b},${(p.a * tw).toFixed(3)})`;
      ctx!.fill();
    }
  }
}
