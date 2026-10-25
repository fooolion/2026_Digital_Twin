/**
 * 遥操作页（R6，2026-10-25）：老师工作流第③步——PICO 遥操设定的网页镜像。
 *
 * 职责：把键鼠操作实时映射为 PICO 手柄状态并可视化（左/右手柄 SVG 驱动），
 * 叠加三级防护状态机（LOCKED → 长按解锁 → ARMED ⇄ E-STOP）。
 * 当前为纯前端镜像（无下行链路）；P5 伴生服务接入后，输入状态经 WebSocket 直通 PC 权威端。
 *
 * 键鼠映射（数据驱动，改 MAPPINGS 即全站生效）：
 *   左手柄（漫游）：WASD/方向键→摇杆，Shift→握持，Q→X，E→A
 *   右手柄（操作）：鼠标位移→摇杆（视口相对），左键→A 急停半键，右键→B 急停半键，空格→Y
 *   A+B 同按 = 急停（映射 PICO 双键急停）；长按按钮 1s = 解锁使能
 */
import { mountNav, toast } from './nav';
import './theme.css';
import './teleop.css';

mountNav('teleop');

/* ================= 数据驱动配置 ================= */

const MAPPINGS: { key: string; pad: 'L' | 'R' | '—'; to: string; desc: string }[] = [
  { key: 'W A S D / ↑←↓→', pad: 'L', to: '左摇杆', desc: '漫游移动（VR 视点）' },
  { key: 'Shift（按住）', pad: 'L', to: '握持键', desc: '抓取/握持物体' },
  { key: 'Q / E', pad: 'L', to: 'X / A', desc: '功能键（镜像端对应 PICO 左手柄 A/X）' },
  { key: '鼠标移动', pad: 'R', to: '右摇杆', desc: '目标点微调（视口相对位移）' },
  { key: '鼠标左键 + 右键', pad: 'R', to: 'A + B', desc: '双键急停（≤5ms 切断动力）' },
  { key: '空格', pad: 'R', to: 'Y', desc: '模式切换（自动/手动）' },
  { key: '长按「解锁」按钮 1s', pad: '—', to: '使能链路', desc: 'LOCKED → ARMED（对应长按右摇杆）' },
];

const STATS = [
  { name: 'PICO 头显 · Unity', lamp: 'warn', state: '待接入' },
  { name: 'WebSocket 下行桥', lamp: 'warn', state: '未部署（P5）' },
  { name: '安全联锁状态机', lamp: 'ok', state: '本地运行' },
];

const LAMP_CLS: Record<string, string> = { ok: '', warn: 'lamp-warn', off: 'lamp-alarm' };
const VAL_CLS: Record<string, string> = { ok: 'on', warn: 'warn', off: '' };
document.getElementById('tp-stats')!.innerHTML = STATS.map((s) => `
  <li><span class="lamp ${LAMP_CLS[s.lamp]}"></span><b>${s.name}</b><span class="val ${VAL_CLS[s.lamp]}">${s.state}</span></li>`).join('');

document.getElementById('tp-map')!.innerHTML = MAPPINGS.map((m) => `
  <tr><td class="m-key">${m.key}</td><td class="m-pad">${m.pad}</td><td class="m-to">${m.to}</td><td class="m-desc">${m.desc}</td></tr>`).join('');

/* ================= 输入状态（帧循环渲染到 SVG） ================= */

const input = {
  lStick: { x: 0, y: 0 },   // -1..1
  rStick: { x: 0, y: 0 },
  lGrip: false,
  lX: false, lA: false,
  rB: false, rA: false, rY: false, // rA=鼠标左键, rB=鼠标右键（急停双键）
};

const keys = new Set<string>();
addEventListener('keydown', (e) => {
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  keys.add(k);
  if (k === 'q') input.lX = true;
  if (k === 'e') input.lA = true;
  if (k === ' ') { input.rY = true; e.preventDefault(); }
  updateKeyboardStick();
});
addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  keys.delete(k);
  if (k === 'q') input.lX = false;
  if (k === 'e') input.lA = false;
  if (k === ' ') input.rY = false;
  updateKeyboardStick();
});
function updateKeyboardStick(): void {
  const x = (keys.has('d') || keys.has('arrowright') ? 1 : 0) - (keys.has('a') || keys.has('arrowleft') ? 1 : 0);
  const y = (keys.has('w') || keys.has('arrowup') ? 1 : 0) - (keys.has('s') || keys.has('arrowdown') ? 1 : 0);
  input.lStick.x = x; input.lStick.y = y;
  input.lGrip = keys.has('shift');
}

// 鼠标 → 右摇杆：视口中心的相对位移，回中衰减（模拟摇杆自动回中）
const view = document.querySelector('.tp-view') as HTMLElement;
let mouseNX = 0, mouseNY = 0;
view.addEventListener('mousemove', (e) => {
  const r = view.getBoundingClientRect();
  mouseNX = Math.max(-1, Math.min(1, ((e.clientX - r.left) / r.width - 0.5) * 2.4));
  mouseNY = Math.max(-1, Math.min(1, -((e.clientY - r.top) / r.height - 0.5) * 2.4));
});
view.addEventListener('mouseleave', () => { mouseNX = 0; mouseNY = 0; });
view.addEventListener('mousedown', (e) => {
  if (e.button === 0) input.rA = true;
  if (e.button === 2) input.rB = true;
  checkEstopCombo();
});
addEventListener('mouseup', (e) => {
  if (e.button === 0) input.rA = false;
  if (e.button === 2) input.rB = false;
});
view.addEventListener('contextmenu', (e) => e.preventDefault()); // 右键是 B 键，别弹菜单

/* ================= 安全联锁状态机（三级防护） ================= */

type LockState = 'LOCKED' | 'ARMED' | 'ESTOP';
let lock: LockState = 'LOCKED';
const lockLamp = document.getElementById('lock-lamp')!;
const lockName = document.getElementById('lock-name')!;
const lockDesc = document.getElementById('lock-desc')!;
const holdBtn = document.getElementById('hold-btn') as HTMLElement;
const holdFill = document.getElementById('hold-fill') as HTMLElement;
const estopBtn = document.getElementById('tp-estop') as HTMLElement;

function setLock(s: LockState): void {
  lock = s;
  const conf: Record<LockState, [string, string, string, string]> = {
    LOCKED: ['lamp-alarm', 'LOCKED · 闭锁', '上电默认闭锁，禁止一切下行指令', 'rgba(255,77,79,.5)'],
    ARMED: ['', 'ARMED · 已使能', '下行指令通道开放（P5 桥接后生效）', 'rgba(46,229,132,.5)'],
    ESTOP: ['lamp-warn', 'E-STOP · 已急停', '双键急停已触发，长按解锁前保持切断', 'rgba(255,180,84,.6)'],
  };
  const [lamp, name, desc, glow] = conf[s];
  lockLamp.className = `lamp ${lamp}`;
  lockName.textContent = name;
  lockDesc.textContent = desc;
  (document.getElementById('lock-card') as HTMLElement).style.setProperty('--lock-glow', glow);
  holdBtn.hidden = s === 'ARMED';
  estopBtn.hidden = s !== 'ARMED';
}

// 长按解锁（pointer 事件，兼容触屏）
let holdT0 = 0;
const HOLD_MS = 1000;
let holdRaf = 0;
function holdStep(): void {
  const held = performance.now() - holdT0;
  holdFill.style.width = `${Math.min(100, (held / HOLD_MS) * 100)}%`;
  if (held >= HOLD_MS) {
    setLock('ARMED');
    logEvent(lock === 'ESTOP' ? '急停复位 → ARMED' : '解锁 → ARMED');
    toast('已使能（ARMED）· 下行链路 P5 接入后生效');
    holdFill.style.width = '0%';
    return;
  }
  holdRaf = requestAnimationFrame(holdStep);
}
holdBtn.addEventListener('pointerdown', () => {
  holdT0 = performance.now();
  holdRaf = requestAnimationFrame(holdStep);
});
const holdEnd = (): void => { cancelAnimationFrame(holdRaf); holdFill.style.width = '0%'; };
holdBtn.addEventListener('pointerup', holdEnd);
holdBtn.addEventListener('pointerleave', holdEnd);

// A+B 双键急停（鼠标双键同按）
function checkEstopCombo(): void {
  if (input.rA && input.rB && lock === 'ARMED') {
    setLock('ESTOP');
    logEvent('A+B 双键急停 → E-STOP');
    toast('双键急停已触发 · ≤5ms 切断（镜像演示）');
  }
}
estopBtn.addEventListener('click', () => {
  if (lock === 'ARMED') {
    setLock('ESTOP');
    logEvent('急停按钮 → E-STOP');
    toast('急停已触发');
  }
});

/* ================= 事件日志 ================= */

const logEl = document.getElementById('tp-log')!;
const logs: { t: string; msg: string }[] = [];
function logEvent(msg: string): void {
  const d = new Date();
  logs.unshift({ t: `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`, msg });
  if (logs.length > 8) logs.pop();
  logEl.innerHTML = logs.map((l) => `<li><span class="log-t">${l.t}</span><span>${l.msg}</span></li>`).join('');
}

/* ================= 帧循环：输入状态 → 手柄 SVG ================= */

const lThumb = document.getElementById('l-thumb')!;
const rThumb = document.getElementById('r-thumb')!;
const padIOs = { l: document.getElementById('l-io')!, r: document.getElementById('r-io')! };
const STICK_R = 8.5; // 摇杆头可移动半径（SVG 单位）

function setBtn(el: HTMLElement, on: boolean): void {
  el.classList.toggle('on', on);
}
let lastRStick = { x: 0, y: 0 };

function frame(): void {
  // 右摇杆：鼠标位置向目标平滑（回中由 mouseleave 置 0 驱动）
  lastRStick.x += (mouseNX - lastRStick.x) * 0.25;
  lastRStick.y += (mouseNY - lastRStick.y) * 0.25;
  input.rStick.x = lastRStick.x; input.rStick.y = lastRStick.y;

  lThumb.setAttribute('transform', `translate(${input.lStick.x * STICK_R} ${-input.lStick.y * STICK_R})`);
  rThumb.setAttribute('transform', `translate(${input.rStick.x * STICK_R} ${-input.rStick.y * STICK_R})`);
  setBtn(document.getElementById('l-grip')!, input.lGrip);
  setBtn(document.getElementById('l-x')!, input.lX);
  setBtn(document.getElementById('l-a')!, input.lA);
  setBtn(document.getElementById('r-b')!, input.rB);
  setBtn(document.getElementById('r-y')!, input.rY);
  // 右手柄 A 圈（=鼠标左键）标注在 io 行；B 按钮 on 态由 rB

  const fmt = (v: { x: number; y: number }): string => `(${v.x.toFixed(1)}, ${v.y.toFixed(1)})`;
  padIOs.l.textContent = [
    `摇杆 ${fmt(input.lStick)}`,
    input.lGrip ? '握持●' : '握持—',
    input.lX ? 'X●' : 'X—', input.lA ? 'A●' : 'A—',
  ].join('  ');
  padIOs.r.textContent = [
    `摇杆 ${fmt(input.rStick)}`,
    input.rA ? 'A●' : 'A—', input.rB ? 'B●' : 'B—',
    input.rY ? 'Y●' : 'Y—',
  ].join('  ');

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

/* ================= 初始 ================= */

setLock('LOCKED');
logEvent('镜像就绪 · LOCKED');
