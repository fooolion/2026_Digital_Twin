/**
 * 运行监控页（老师工作流第④步：执行阶段观察窗口）。
 * 布局：左视频流（PICO 大屏+双相机占位）+ 右急停/链路/速度档 + 底部 ECharts 关节遥测。
 * 数据：STATS 静态配置、遥测为滚动演示数据。
 * P5 接入点：视频 src（WebRTC/MJPEG）、STATS 轮询、遥测换 WebSocket 实时流。
 * Rev: R4 · 2026-10-25
 */
import { mountNav, toast, renderStatList, type StatItem } from './nav';
import { COMPANION_BASE, connectBridge, watchCompanion } from './companion';
import { pullWhep } from './whep';
import './theme.css';
import './monitor.css';

mountNav('live');

/* ===== 链路状态（数据驱动；P5 接入后改为轮询伴生服务） ===== */
const STATS: (StatItem & { probe?: 'cams' | 'pico' | 'ws' })[] = [
  { name: 'PICO · Unity VR 串流', lamp: 'warn', state: '探测中…', probe: 'pico' },
  { name: '相机 A / 相机 B', lamp: 'warn', state: '探测中…', probe: 'cams' },
  { name: 'WebSocket 桥', lamp: 'warn', state: '探测中…', probe: 'ws' },
  { name: '实体臂 B601-DM', lamp: 'off', state: '离线 · 未使能' },
];
renderStatList(document.getElementById('mn-stats')!, STATS);

/* ===== 三块共享探测：watchCompanion 统一驱动（周期重探；服务后启动也能恢复） ===== */
watchCompanion((h) => {
  if (!h?.ok) {
    picoState('warn', '未启动（演示模式）');
    camStat('warn', '未启动（演示模式）');
    wsStat('warn', '未启动（演示模式）');
    return;
  }
  void setupPico();
  void setupCams();
  void setupBridge();
});

/* ===== P5-C：PICO 大屏三层降级（WHEP 实时流 > 预录视频 > 占位层） ===== */
const picoVideo = document.getElementById('pico-video') as HTMLVideoElement;
const picoPh = document.getElementById('pico-ph')!;

function picoState(lamp: 'ok' | 'warn' | 'off', state: string): void {
  const i = STATS.findIndex((s) => s.probe === 'pico');
  if (i >= 0) {
    STATS[i] = { ...STATS[i], lamp, state };
    renderStatList(document.getElementById('mn-stats')!, STATS);
  }
}

function showPicoVideo(srcType: 'stream' | 'file'): void {
  picoPh.style.display = 'none';
  picoVideo.style.display = 'block';
  void srcType;
}

async function setupPico(): Promise<void> {
  let cfg: { whep: string; video: boolean; video_url: string | null } | null = null;
  try {
    cfg = await (await fetch(`${COMPANION_BASE}/api/stream`)).json();
  } catch { /* 配置查询失败按占位处理 */ }

  if (cfg?.whep) {
    // ① 实时流（MediaMTX WHEP）
    try {
      const session = await pullWhep(cfg.whep, picoVideo);
      showPicoVideo('stream');
      picoState('ok', '实时流 · WebRTC');
      session.pc.addEventListener('connectionstatechange', () => {
        if (session.pc.connectionState === 'failed') {
          picoState('off', '实时流中断');
        }
      });
      return;
    } catch {
      // WHEP 失败（MediaMTX 未推流）→ 降级预录
    }
  }
  if (cfg?.video && cfg.video_url) {
    // ② 预录视频（老师认可的过渡形态；相对当前源——伴生服务同源 / Pages 部署时伴生服务不在则探不到）
    picoVideo.src = cfg.video_url;
    picoVideo.loop = true;
    picoVideo.muted = true; // 自动播放策略要求静音
    void picoVideo.play().catch(() => { /* 首次交互后恢复 */ });
    showPicoVideo('file');
    picoState('ok', '预录视频 · 循环播放');
    return;
  }
  // ③ 占位层（文案更新为可操作的指引）
  const hint = picoPh.querySelector('span');
  if (hint) hint.textContent = cfg?.whep ? 'MediaMTX 未推流 · 请检查 OBS' : '放入 web/public/media/pico.mp4 或配置 PICO_WHEP_URL';
  picoState('warn', '待接入（预录视频/实时流均未就位）');
}

/* ===== P5-B：伴生服务在线 → 双摄像头 MJPEG 填 src + 状态灯；断流回退占位 ===== */
const camImgs = [
  document.getElementById('cam-a') as HTMLImageElement,
  document.getElementById('cam-b') as HTMLImageElement,
];
const camPhs = [
  document.getElementById('cam-a-box')!.querySelector('.mn-video-ph') as HTMLElement,
  document.getElementById('cam-b-box')!.querySelector('.mn-video-ph') as HTMLElement,
];

function camStat(lamp: 'ok' | 'warn' | 'off', state: string): void {
  const i = STATS.findIndex((s) => s.probe === 'cams');
  if (i >= 0) {
    STATS[i] = { ...STATS[i], lamp, state };
    renderStatList(document.getElementById('mn-stats')!, STATS);
  }
}

function attachCam(slot: 0 | 1): void {
  const img = camImgs[slot];
  img.src = `${COMPANION_BASE}/cam/${slot}.mjpg?t=${Date.now()}`; // t 参数防旧流缓存
  img.style.display = 'block';
  camPhs[slot].style.display = 'none';
  img.onerror = (): void => { // 服务中途停止 → 回退占位层
    img.style.display = 'none';
    img.removeAttribute('src');
    camPhs[slot].style.display = 'flex';
  };
}

async function setupCams(): Promise<void> {
  attachCam(0);
  attachCam(1);
  // 相机在线状态（工业相机可能未接——MJPEG 流有占位帧，状态灯如实显示）
  try {
    const res = await fetch(`${COMPANION_BASE}/api/cams`);
    const { cams } = await res.json() as { cams: { slot: number; online: boolean }[] };
    const i = STATS.findIndex((s) => s.probe === 'cams');
    if (i >= 0) {
      const n = cams.filter((c) => c.online).length;
      STATS[i] = { ...STATS[i], lamp: n === cams.length ? 'ok' : n > 0 ? 'warn' : 'off',
        state: n === cams.length ? '双路在线' : n > 0 ? `${n}/${cams.length} 路在线` : '相机未接入（占位帧）' };
      renderStatList(document.getElementById('mn-stats')!, STATS);
    }
  } catch { /* 状态查询失败不影响流 */ }
}

/* ===== 速度档（stretch_web_teleop 三件套之一；默认 3 档） ===== */
const GEARS = ['1', '2', '3', '4', '5'];
const gearEl = document.getElementById('speed-gear')!;
gearEl.innerHTML = GEARS.map((g, i) => `
  <button type="button" role="radio" class="gear${i === 2 ? ' active' : ''}" data-g="${g}" aria-checked="${i === 2}" tabindex="${i === 2 ? 0 : -1}">${g}</button>`).join(''); // 初始即 roving：仅 active 一个 tab stop
function setActiveGear(btn: HTMLElement): void {
  gearEl.querySelectorAll('.gear').forEach((gEl) => {
    const g = gEl as HTMLElement;
    const on = g === btn;
    g.classList.toggle('active', on);
    g.setAttribute('aria-checked', String(on));
    g.tabIndex = on ? 0 : -1; // roving tabindex
  });
  btn.focus();
}
gearEl.addEventListener('click', (e) => {
  const btn = (e.target as HTMLElement).closest('.gear') as HTMLElement | null;
  if (!btn) return;
  setActiveGear(btn);
  toast(`速度档 ${btn.dataset.g}（P5 接入后下发真机）`);
});
// ARIA radio 模式：左右方向键切档（遥操作习惯）
gearEl.addEventListener('keydown', (e) => {
  const gears = [...gearEl.querySelectorAll('.gear')] as HTMLElement[];
  const idx = gears.findIndex((g) => g.classList.contains('active'));
  const next = e.key === 'ArrowRight' ? (idx + 1 + gears.length) % gears.length
    : e.key === 'ArrowLeft' ? (idx - 1 + gears.length) % gears.length : -1;
  if (next >= 0) { e.preventDefault(); setActiveGear(gears[next]); }
});

/* ===== 急停（视觉最重；IEC 60204 式闭锁——触发即锁定，复位解锁；真机链路 P5） ===== */
const estop = document.getElementById('estop-btn')!;
const estopReset = document.getElementById('estop-reset')!;
let estopLatched = false;
estop.addEventListener('click', () => {
  if (estopLatched) return;
  estopLatched = true;
  estop.classList.add('latched');
  estopReset.hidden = false;
  toast('急停已触发并锁定 · 复位需显式操作');
});
estopReset.addEventListener('click', () => {
  estopLatched = false;
  estop.classList.remove('latched');
  estopReset.hidden = true;
  toast('急停已复位');
});

/* ===== 全屏（兼容 Safari 的 webkit 前缀；无 API 时兜底 toast 可达） ===== */
type FsElement = HTMLElement & { webkitRequestFullscreen?: () => Promise<void> };
document.getElementById('fs-btn')!.addEventListener('click', () => {
  const box = document.getElementById('pico-video-box') as FsElement;
  const doc = document as Document & { webkitFullscreenElement?: Element; webkitExitFullscreen?: () => Promise<void> };
  if (document.fullscreenElement ?? doc.webkitFullscreenElement) {
    void (document.exitFullscreen ?? doc.webkitExitFullscreen!)?.call(document);
  } else if (box.requestFullscreen) {
    void box.requestFullscreen().catch(() => toast('进入全屏被浏览器拒绝'));
  } else if (box.webkitRequestFullscreen) {
    void box.webkitRequestFullscreen().catch(() => toast('进入全屏被浏览器拒绝'));
  } else {
    toast('当前浏览器不支持元素全屏');
  }
});

/* ===== P5-E：接桥——上行 telemetry 驱动 ECharts 实时曲线；无桥时保持滚动演示 ===== */
let feedReal: ((j: number[]) => void) | null = null; // 桥在线时由下方闭包注入；离线回落演示数据

let wsDownFlag = false; // 刷新器读的桥状态（wsStat 同步写入）
function wsStat(lamp: 'ok' | 'warn' | 'off', state: string): void {
  wsDownFlag = lamp === 'off';
  const i = STATS.findIndex((s) => s.probe === 'ws');
  if (i >= 0) {
    STATS[i] = { ...STATS[i], lamp, state };
    renderStatList(document.getElementById('mn-stats')!, STATS);
  }
}

async function setupBridge(): Promise<void> {
  connectBridge(
    (payload, src) => {
      // 上行 telemetry（pc 端发）→ 实时曲线注入
      if (src === 'telemetry' && payload && typeof payload === 'object' && 'j' in payload) {
        const j = (payload as { j: unknown }).j;
        if (Array.isArray(j) && j.length >= 6 && j.every((v) => typeof v === 'number')) {
          feedReal?.(j as number[]);
        }
      }
    },
    (reason) => wsStat('off', `已断开（${reason}）`),
  );
  wsStat('ok', '已连接 · 实时遥测');
}

/* ===== 关节遥测趋势（ECharts：桥在线走实时流；离线滚动演示） ===== */
void (async () => {
  const echarts = await import('echarts/core');
  const chartEl = document.getElementById('trend-chart')!;
  const { LineChart } = await import('echarts/charts');
  const { GridComponent, TooltipComponent, LegendComponent } = await import('echarts/components');
  const { CanvasRenderer } = await import('echarts/renderers');
  echarts.use([LineChart, GridComponent, TooltipComponent, LegendComponent, CanvasRenderer]);
  const chart = echarts.init(chartEl, undefined, { renderer: 'canvas' });

  // 滚动窗口：真流（feedReal 注入）与离线演示共用同一条管线（追加+滑窗+局部 setOption）
  const WIN = 120;
  let realMode = false; // 桥注入第一帧后置真——演示步进立即停
  const state = Array.from({ length: 6 }, (_, j) => ({ phase: j * 1.1, amp: 18 - j * 2.2, k: 0.5 + j * 0.13 }));
  const data: number[][] = Array.from({ length: 6 }, () => []);
  const labels: string[] = [];
  let tick = 0;
  const step = (): void => {
    tick += 0.1;
    labels.push(tick.toFixed(1));
    if (labels.length > WIN) labels.shift();
    for (let j = 0; j < 6; j++) {
      const s = state[j];
      s.phase += s.k * 0.1;
      data[j].push(+(Math.sin(s.phase) * s.amp).toFixed(2));
      if (data[j].length > WIN) data[j].shift();
    }
  };
  for (let i = 0; i < WIN; i++) step();
  const demoStep = step; // 真流模式下间隔器只刷视图不产数据

  const series = data.map((d, j) => ({
    name: `J${j + 1}`,
    type: 'line' as const,
    showSymbol: false,
    smooth: true,
    data: d,
    lineStyle: { width: 1.6 },
    emphasis: { focus: 'series' as const },
  }));

  chart.setOption({
    backgroundColor: 'transparent',
    color: ['#00d4ff', '#2e8bff', '#2ee584', '#ffd166', '#ff7a45', '#b58cff'],
    grid: { left: 44, right: 16, top: 30, bottom: 26 },
    tooltip: { trigger: 'axis', backgroundColor: 'rgba(8,20,38,.92)', borderColor: 'rgba(0,212,255,.3)', textStyle: { color: '#dfe7f3', fontSize: 11 } },
    legend: { top: 0, right: 0, textStyle: { color: '#8fa0b5', fontSize: 11 }, itemWidth: 14, itemHeight: 8 },
    xAxis: { type: 'category', data: labels, axisLine: { lineStyle: { color: 'rgba(143,160,181,.25)' } }, axisLabel: { color: '#56637a', fontSize: 10 } },
    yAxis: { type: 'value', name: '°', nameTextStyle: { color: '#56637a' }, splitLine: { lineStyle: { color: 'rgba(143,160,181,.12)' } }, axisLabel: { color: '#56637a', fontSize: 10 } },
    animation: false,
    series,
  });

  // feedReal：桥在线时注入真数据（realMode 置位后演示步进停、窗口清零重来）
  let lastRealTs = 0; // 最后一帧真数据的时刻（断线回落判据）
  feedReal = (j: number[]): void => {
    if (!realMode) {
      realMode = true;
      data.forEach((d) => (d.length = 0));
      labels.length = 0;
      const cap = document.getElementById('trend-src');
      if (cap) cap.textContent = '（实时遥测 · 来自 PC 桥）';
    }
    lastRealTs = performance.now();
    tick += 0.1;
    labels.push(tick.toFixed(1));
    if (labels.length > WIN) labels.shift();
    for (let idx = 0; idx < 6; idx++) {
      data[idx].push(+j[idx].toFixed(2));
      if (data[idx].length > WIN) data[idx].shift();
    }
  };

  // 滚动刷新（200ms 局部 setOption；真流模式只刷视图；断线回落演示——冻结曲线+无提示是死角）
  let timer = 0;
  const start = (): void => {
    window.clearInterval(timer);
    timer = window.setInterval(() => {
      // 断线回落：曾走过真流、桥已报断、真帧停更 >2s → 回演示（数据保留当前真流尾巴接着滚）
      if (realMode && wsDownFlag && performance.now() - lastRealTs > 2000) {
        realMode = false;
        const cap = document.getElementById('trend-src');
        if (cap) cap.textContent = '（演示数据 · 实时流已断开）';
      }
      if (!realMode) demoStep();
      chart.setOption({ xAxis: { data: labels.slice() }, series: series.map((s) => ({ data: s.data.slice() })) });
    }, 200);
  };
  start();

  // 容器级监听：纵向滚动条出现压缩视口宽 ~17px 时 window resize 不触发
  new ResizeObserver(() => chart.resize()).observe(chartEl);
  // 页面隐藏暂停、回来恢复（只清不恢复会让曲线"切走一次就死"）
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) window.clearInterval(timer);
    else start();
  });
})();
