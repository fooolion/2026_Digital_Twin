/**
 * 运行监控页（R4，2026-10-25）：老师工作流第④步的观察窗口。
 * 布局照 Bambu Studio Device tab + stretch_web_teleop 规范：
 * 左视频流（PICO 大屏 + 双相机小窗）· 右急停/链路/速度档 · 底部关节遥测趋势。
 * 当前视频与遥测均为占位/演示数据——P5 伴生服务（FastAPI+WebSocket+MJPEG）接入后替换。
 */
import { mountNav, toast } from './nav';
import './theme.css';
import './monitor.css';

mountNav('live');

/* ===== 链路状态（数据驱动；P5 接入后改为轮询伴生服务） ===== */
const STATS = [
  { name: 'PICO · Unity VR 串流', lamp: 'warn', state: '待接入' },
  { name: '相机 A / 相机 B', lamp: 'warn', state: '待接入' },
  { name: 'WebSocket 桥', lamp: 'warn', state: '未部署（P5）' },
  { name: '实体臂 B601-DM', lamp: 'off', state: '离线 · 未使能' },
];
const LAMP_CLS: Record<string, string> = { ok: '', warn: 'lamp-warn', off: 'lamp-alarm' };
const VAL_CLS: Record<string, string> = { ok: 'on', warn: 'warn', off: '' };
document.getElementById('mn-stats')!.innerHTML = STATS.map((s) => `
  <li><span class="lamp ${LAMP_CLS[s.lamp]}"></span><b>${s.name}</b><span class="val ${VAL_CLS[s.lamp]}">${s.state}</span></li>`).join('');

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

/* ===== 关节遥测趋势（ECharts · 滚动演示数据；P5 后换 WebSocket 实时流） ===== */
void (async () => {
  const echarts = await import('echarts/core');
  const chartEl = document.getElementById('trend-chart')!;
  const { LineChart } = await import('echarts/charts');
  const { GridComponent, TooltipComponent, LegendComponent } = await import('echarts/components');
  const { CanvasRenderer } = await import('echarts/renderers');
  echarts.use([LineChart, GridComponent, TooltipComponent, LegendComponent, CanvasRenderer]);
  const chart = echarts.init(chartEl, undefined, { renderer: 'canvas' });

  // 演示数据：六关节相位错开的缓变正弦，滚动窗口模拟实时遥测
  // （P5 后改为：WebSocket 推送 → push 进各 series data → setOption 局部更新）
  const WIN = 120;
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

  // 滚动更新：追加 + 窗口滑动（局部 setOption + 快照数组，避免全量重建/引用比较跳过）
  let timer = 0;
  const start = (): void => {
    window.clearInterval(timer);
    timer = window.setInterval(() => {
      step();
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
