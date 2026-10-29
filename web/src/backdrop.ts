/**
 * 全页环境背景 · 方案3+6（粒子星野 + 地平线厂房剪影）——总览/零件库/仿真规划三页统一。
 * 挂载进各页 .ambient 层（缺则自动补建：网格底 + 光斑，再叠加本模块的粒子与地平线）。
 * 粒子 ~90 颗低速漂移+微闪（rAF，页面隐藏自动停）；prefers-reduced-motion 时静帧不动画。
 * Rev: P1 · 2026-10-29
 */

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

interface P { x: number; y: number; r: number; vx: number; vy: number; a: number; ph: number; g: boolean; }

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
    // 静帧：只画一帧，不做动画
    draw(0);
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
