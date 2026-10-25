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
  <button type="button" role="radio" class="gear${i === 2 ? ' active' : ''}" data-g="${g}" aria-checked="${i === 2}">${g}</button>`).join('');
gearEl.addEventListener('click', (e) => {
  const btn = (e.target as HTMLElement).closest('.gear') as HTMLElement | null;
  if (!btn) return;
  gearEl.querySelectorAll('.gear').forEach((g) => {
    g.classList.toggle('active', g === btn);
    g.setAttribute('aria-checked', String(g === btn));
  });
  toast(`速度档 ${btn.dataset.g}（P5 接入后下发真机）`);
});

/* ===== 急停（视觉最重；当前无后端，按下即红闪 + toast） ===== */
const estop = document.getElementById('estop-btn')!;
estop.addEventListener('click', () => {
  estop.classList.add('triggered');
  toast('急停指令已记录 · 下发链路 P5 接入');
  window.setTimeout(() => estop.classList.remove('triggered'), 1200);
});

/* ===== 全屏 ===== */
document.getElementById('fs-btn')!.addEventListener('click', () => {
  const box = document.getElementById('pico-video-box')!;
  if (document.fullscreenElement) void document.exitFullscreen();
  else void box.requestFullscreen().catch(() => toast('当前环境不支持全屏'));
});

/* ===== 关节遥测趋势（ECharts · 演示数据） ===== */
void (async () => {
  const echarts = await import('echarts');
  const chartEl = document.getElementById('trend-chart')!;
  const chart = echarts.init(chartEl, undefined, { renderer: 'canvas' });

  // 演示数据：六关节相位错开的缓变正弦（P5 后换成 WebSocket 推送的实时环形缓冲）
  const N = 120;
  const t = Array.from({ length: N }, (_, i) => i * 0.1);
  const series = [0, 1, 2, 3, 4, 5].map((j) => ({
    name: `J${j + 1}`,
    type: 'line' as const,
    showSymbol: false,
    smooth: true,
    data: t.map((x) => +(Math.sin(x * (0.5 + j * 0.13) + j * 1.1) * (18 - j * 2.2)).toFixed(2)),
    lineStyle: { width: 1.6 },
    emphasis: { focus: 'series' as const },
  }));

  chart.setOption({
    backgroundColor: 'transparent',
    color: ['#00d4ff', '#2e8bff', '#2ee584', '#ffd166', '#ff7a45', '#b58cff'],
    grid: { left: 44, right: 16, top: 30, bottom: 26 },
    tooltip: { trigger: 'axis', backgroundColor: 'rgba(8,20,38,.92)', borderColor: 'rgba(0,212,255,.3)', textStyle: { color: '#dfe7f3', fontSize: 11 } },
    legend: { top: 0, right: 0, textStyle: { color: '#8fa0b5', fontSize: 11 }, itemWidth: 14, itemHeight: 8 },
    xAxis: { type: 'category', data: t.map((v) => v.toFixed(1)), axisLine: { lineStyle: { color: 'rgba(143,160,181,.25)' } }, axisLabel: { color: '#56637a', fontSize: 10 } },
    yAxis: { type: 'value', name: '°', nameTextStyle: { color: '#56637a' }, splitLine: { lineStyle: { color: 'rgba(143,160,181,.12)' } }, axisLabel: { color: '#56637a', fontSize: 10 } },
    series,
  });

  const onResize = (): void => chart.resize();
  addEventListener('resize', onResize);
})();
