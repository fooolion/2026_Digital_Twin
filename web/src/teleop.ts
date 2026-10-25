/**
 * 遥操作页（老师工作流第③步：PICO 遥操的网页镜像）。
 * 布局：左安全联锁（三级防护状态机）+ 中 VR 视口（左右手柄 SVG 键鼠实时驱动）+ 右映射表/事件日志。
 * 状态机：LOCKED →(长按1s)→ ARMED →(A+B双键)→ E-STOP →(长按)→ ARMED；ARMED 可点按回 LOCKED。
 * P5 接入点：input 状态对象即 WebSocket 下行 payload。
 * Rev: R6 · 2026-10-25
 */
import { mountNav, toast, renderStatList, type StatItem } from './nav';
import { probeCompanion, connectBridge } from './companion';
import './theme.css';
import './teleop.css';

mountNav('teleop');

/* ===== 数据驱动配置 ===== */

const MAPPINGS: { key: string; pad: 'L' | 'R' | '—'; to: string; desc: string }[] = [
  { key: 'W A S D / ↑←↓→', pad: 'L', to: '左摇杆', desc: '漫游移动（VR 视点）' },
  { key: 'Shift（按住）', pad: 'L', to: '握持键', desc: '抓取/握持物体' },
  { key: 'Q', pad: 'L', to: 'X', desc: '菜单/功能键（PICO 4 左手柄 = X/Y）' },
  { key: '空格 / E', pad: 'L', to: 'Y', desc: '模式切换（自动/手动）' },
  { key: '鼠标移动', pad: 'R', to: '右摇杆', desc: '目标点微调（视口相对位移）' },
  { key: '鼠标左键 / 右键', pad: 'R', to: 'A / B', desc: '单键功能位（PICO 4 右手柄 = A/B；触屏双指同按等价）' },
  { key: '鼠标左键 + 右键', pad: 'R', to: 'A + B 同按', desc: '双键急停（≤5ms 切断动力）' },
  { key: '长按「解锁」按钮 1s', pad: '—', to: '使能链路', desc: 'LOCKED → ARMED（对应长按右摇杆）' },
];

const STATS: (StatItem & { probe?: 'bridge' })[] = [
  { name: 'PICO 头显 · Unity', lamp: 'warn', state: '待接入' },
  { name: 'WebSocket 下行桥', lamp: 'warn', state: '探测中…', probe: 'bridge' },
  { name: '安全联锁状态机', lamp: 'ok', state: '本地运行' },
];

renderStatList(document.getElementById('tp-stats')!, STATS);

document.getElementById('tp-map')!.innerHTML = MAPPINGS.map((m) => `
  <tr><td class="m-key">${m.key}</td><td class="m-pad">${m.pad}</td><td class="m-to">${m.to}</td><td class="m-desc">${m.desc}</td></tr>`).join('');

/* ===== 输入状态（帧循环渲染到 SVG） ===== */

const input = {
  lStick: { x: 0, y: 0 },   // -1..1
  rStick: { x: 0, y: 0 },
  lGrip: false,
  lX: false, lY: false,     // 左柄 X/Y（PICO 4 真机布局）
  rA: false, rB: false,     // 右柄 A/B（rA=鼠标左键, rB=鼠标右键，双键急停）
};

const keys = new Set<string>();
const onInteractive = (e: Event): boolean =>
  !!(e.target as HTMLElement)?.closest?.('button,a,input,textarea,select');
addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  // 方向键驱动摇杆：别滚动页面；空格只防滚动、不杀焦点按钮的激活
  if (k.startsWith('arrow') && !onInteractive(e)) e.preventDefault();
  if (k === ' ' && !onInteractive(e)) e.preventDefault();
  if (e.repeat) return;
  if (onInteractive(e)) return; // 焦点在按钮/输入上：按键属于控件操作语境，不做手柄镜像
  keys.add(k);
  if (k === 'q') input.lX = true;
  if (k === 'e' || k === ' ') input.lY = true;
  updateKeyboardStick();
});
addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  keys.delete(k);
  if (k === 'q') input.lX = false;
  if (k === 'e' || k === ' ') input.lY = false;
  updateKeyboardStick();
});
function updateKeyboardStick(): void {
  const x = (keys.has('d') || keys.has('arrowright') ? 1 : 0) - (keys.has('a') || keys.has('arrowleft') ? 1 : 0);
  const y = (keys.has('w') || keys.has('arrowup') ? 1 : 0) - (keys.has('s') || keys.has('arrowdown') ? 1 : 0);
  input.lStick.x = x; input.lStick.y = y;
  input.lGrip = keys.has('shift');
}

// 失焦清全部输入：按住 W 切窗口时 keyup 丢失，回来摇杆会卡在前进（sim 页拖拽同款坑）；
// 蓄力长按同理——rAF 暂停而时钟照走，切回会"瞬满解锁"，失焦即视为放弃
addEventListener('blur', () => {
  holdEnd();
  keys.clear();
  input.lStick.x = 0; input.lStick.y = 0;
  input.lGrip = false; input.lX = false; input.lY = false;
  input.rA = false; input.rB = false;
  mouseNX = 0; mouseNY = 0;
});

// 鼠标 → 右摇杆：视口中心的相对位移，回中衰减（模拟摇杆自动回中）
const view = document.querySelector('.tp-view') as HTMLElement;
let mouseNX = 0, mouseNY = 0;
// rect 缓存：mousemove 可达 125Hz+，每次 getBoundingClientRect 在布局脏时强制同步 reflow
let viewRect = view.getBoundingClientRect();
const refreshRect = (): void => { viewRect = view.getBoundingClientRect(); };
addEventListener('resize', refreshRect);
new ResizeObserver(refreshRect).observe(view);
let lastPointerType = 'mouse';
view.addEventListener('mousemove', (e) => {
  if (lastPointerType === 'touch') return; // 触屏 tap 补发的兼容 mousemove 会卡死右摇杆偏转
  mouseNX = Math.max(-1, Math.min(1, ((e.clientX - viewRect.left) / viewRect.width - 0.5) * 2.4));
  mouseNY = Math.max(-1, Math.min(1, -((e.clientY - viewRect.top) / viewRect.height - 0.5) * 2.4));
});
view.addEventListener('mouseleave', () => { mouseNX = 0; mouseNY = 0; });
// 按键状态用 mouse/mousedown（Pointer Events 规范：鼠标按住期间按第二个键只派发 mousedown，
// 不派发第二个 pointerdown——双键急停的"第二键"必须靠 mousedown 才收得到）；
// pointerdown 仅负责 setPointerCapture（键在视口外释放也能收到 up，防陈旧态）与触屏双指。
view.addEventListener('pointerdown', (e) => {
  lastPointerType = e.pointerType;
  try { view.setPointerCapture(e.pointerId); } catch { /* 忽略 */ }
  if (e.pointerType !== 'mouse') {
    // 触屏：每指一个 pointerdown，主指=A 副指=B（双指同按即 A+B）
    if (e.isPrimary) input.rA = true; else input.rB = true;
    checkEstopCombo();
  }
});
view.addEventListener('mousedown', (e) => {
  if (lastPointerType === 'touch') return; // 触屏兼容 mousedown 会双触发 rA
  if (e.button === 0) input.rA = true;
  if (e.button === 2) input.rB = true;
  checkEstopCombo();
});
view.addEventListener('mouseup', (e) => {
  if (lastPointerType === 'touch') return;
  if (e.button === 0) input.rA = false;
  if (e.button === 2) input.rB = false;
});
view.addEventListener('pointerup', (e) => {
  if (e.pointerType !== 'mouse') { if (e.isPrimary) input.rA = false; else input.rB = false; }
});
view.addEventListener('pointercancel', () => { input.rA = false; input.rB = false; });
view.addEventListener('contextmenu', (e) => e.preventDefault()); // 右键是 B 键，别弹菜单

/* ===== P5-D：伴生服务桥——input 下行 =====
 * 协议（对齐 UDP 9003 演进）：payload = { seq, armed, estop, lStick, rStick, lGrip, lX, lY, rA, rB }
 *   - armed=true：正常遥操流（ARMED 态 30Hz，零输入也发=心跳）；armed=false + 全零：终结帧（切出 ARMED 时发）
 *   - estop=true：急停通告帧（E-STOP 触发瞬间发一帧）
 *   - 下游（pc/真机）规则：以 armed/estop 标志为主、心跳超时（>200ms 无帧）为兜底——
 *     "断流=停止"不可依赖（TCP 断开可能延迟感知），必须显式标志。
 * 发送时机：ARMED 态 30Hz 节流；LOCKED/ESTOP 不发常规流（但状态切换瞬间发终结帧/急停帧）。 */
let bridgeWs: WebSocket | null = null;
let lastSendTs = 0;
let downSeq = 0;
const SEND_HZ = 30;

function sendDownlinkFrame(armed: boolean, estop: boolean): void {
  if (!bridgeWs || bridgeWs.readyState !== WebSocket.OPEN) return;
  downSeq += 1;
  const zero = { x: 0, y: 0 };
  const body = armed ? input : { lStick: zero, rStick: zero, lGrip: false, lX: false, lY: false, rA: false, rB: false };
  bridgeWs.send(JSON.stringify({
    type: 'input',
    payload: { seq: downSeq, armed, estop, ...body },
  }));
}

function sendInputDownlink(): void {
  if (!bridgeWs || bridgeWs.readyState !== WebSocket.OPEN) return;
  if (lock !== 'ARMED') return; // 闭锁/急停态不发常规流（状态机是下行总闸）
  const now = performance.now();
  if (now - lastSendTs < 1000 / SEND_HZ) return;
  lastSendTs = now;
  sendDownlinkFrame(true, false);
}

function bridgeStat(lamp: 'ok' | 'warn' | 'off', state: string): void {
  const i = STATS.findIndex((s) => s.probe === 'bridge');
  if (i >= 0) {
    STATS[i] = { ...STATS[i], lamp, state };
    renderStatList(document.getElementById('tp-stats')!, STATS);
  }
}

void (async () => {
  const h = await probeCompanion();
  if (!h?.ok) {
    bridgeStat('warn', '未启动（演示模式）');
    return;
  }
  bridgeWs = connectBridge(
    () => { /* 页面暂不消费上行 state（P5-E 监控页消费） */ },
    (reason) => {
      bridgeStat('off', `已断开（${reason}）`);
      // 发现 4：链路断开即自动闭锁（fail-safe）——操作员不能对着死链继续"遥操"
      if (lock === 'ARMED') {
        setLock('LOCKED');
        logEvent(`链路断开（${reason}）→ 自动闭锁`);
        toast('链路断开 · 已自动闭锁');
      }
    },
  );
  if (bridgeWs) bridgeStat('ok', '已连接 · ARMED 时 30Hz 下行');
})();

/* ===== 安全联锁状态机（三级防护） ===== */

type LockState = 'LOCKED' | 'ARMED' | 'ESTOP';
let lock: LockState = 'LOCKED';
const lockLamp = document.getElementById('lock-lamp')!;
const lockName = document.getElementById('lock-name')!;
const lockDesc = document.getElementById('lock-desc')!;
const holdBtn = document.getElementById('hold-btn') as HTMLElement;
const holdFill = document.getElementById('hold-fill') as HTMLElement;
const estopBtn = document.getElementById('tp-estop') as HTMLElement;

function setLock(s: LockState): void {
  const prev = lock;
  lock = s;
  // 发现 2：切出 ARMED 必须发终结帧——线上最后一帧是陈旧非零帧时，
  // "保持最后指令"语义的下游（真机链路常见默认）会带伤继续运动
  if (prev === 'ARMED' && s !== 'ARMED') {
    sendDownlinkFrame(false, s === 'ESTOP'); // armed=false 全零帧；E-STOP 另带 estop 标志
  }
  // 灯色语义：LOCKED=琥珀（未就绪警示）；ESTOP=红闪（最高警示等级，红钮本身恒红）
  const conf: Record<LockState, [string, string, string, string]> = {
    LOCKED: ['lamp-warn', 'LOCKED · 闭锁', '上电默认闭锁，禁止一切下行指令', 'rgba(255,180,84,.4)'],
    ARMED: ['', 'ARMED · 已使能', '下行指令通道开放（P5 桥接后生效）', 'rgba(46,229,132,.5)'],
    ESTOP: ['lamp-alarm', 'E-STOP · 已急停', '双键急停已触发，长按复位前保持切断', 'rgba(255,77,79,.55)'],
  };
  const [lamp, name, desc, glow] = conf[s];
  lockLamp.className = `lamp ${lamp}`;
  lockName.textContent = name;
  lockDesc.textContent = desc;
  const card = document.getElementById('lock-card') as HTMLElement;
  card.style.setProperty('--lock-glow', glow); // 卡片辉光（teleop.css box-shadow 消费）
  card.classList.toggle('armed', s === 'ARMED'); // 标题竖条变绿（.tp-lock-card.armed）
  holdBtn.hidden = false;
  estopBtn.hidden = s !== 'ARMED';
  holdBtn.querySelector('.hold-label')!.textContent =
    s === 'ESTOP' ? '长按 1.0s 复位急停' : s === 'ARMED' ? '点按重新闭锁' : '长按 1.0s 解锁';
}

// 长按解锁（pointer 事件，兼容触屏）
let holdT0 = 0;
const HOLD_MS = 1000;
let holdRaf = 0;
let holding = false; // 多指/多输入源哨兵：防两条 rAF 链互相覆盖导致松手后仍解锁
function holdStart(): void {
  if (holding) return;
  holding = true;
  holdT0 = performance.now();
  holdRaf = requestAnimationFrame(holdStep);
}
function holdStep(): void {
  if (!holding) return; // 孤儿链自杀
  const held = performance.now() - holdT0;
  holdFill.style.width = `${Math.min(100, (held / HOLD_MS) * 100)}%`;
  if (held >= HOLD_MS) {
    // 先记 prevState 再 setLock（setLock 后 lock 恒为新值，后判必错）
    const from = lock;
    setLock('ARMED');
    logEvent(from === 'ESTOP' ? '急停复位 → ARMED' : '解锁 → ARMED');
    toast('已使能（ARMED）· 下行链路 P5 接入后生效');
    holdFill.style.width = '0%';
    holding = false; // 成功即收（防极端的 up 丢失后 holding 卡 true）
    estopBtn.focus({ preventScroll: true }); // 键盘路径：焦点从隐藏的解锁钮转移到急停钮
    checkEstopCombo(); // 兜底：若 A/B 键此刻已按住，ARMED 瞬间立即复查触发
    return;
  }
  holdRaf = requestAnimationFrame(holdStep);
}
holdBtn.addEventListener('pointerdown', (e) => {
  if (e.pointerType === 'mouse' && e.button !== 0) return; // 右/中键不蓄力（也不弹菜单，见下）
  if (lock === 'ARMED') { // ARMED 态复用为"点按闭锁"
    setLock('LOCKED');
    logEvent('手动闭锁 → LOCKED');
    toast('已闭锁（LOCKED）');
    return;
  }
  holdStart();
});
holdBtn.addEventListener('contextmenu', (e) => e.preventDefault());
const holdEnd = (): void => { holding = false; cancelAnimationFrame(holdRaf); holdFill.style.width = '0%'; };
holdBtn.addEventListener('pointerup', holdEnd);
holdBtn.addEventListener('pointerleave', holdEnd);
holdBtn.addEventListener('pointercancel', holdEnd);
// 键盘长按路径（Enter/Space 蓄力；按钮聚焦时 Space 不再被全局 preventDefault 杀掉）
holdBtn.addEventListener('keydown', (e) => {
  if ((e.key === 'Enter' || e.key === ' ') && !e.repeat) holdStart();
});
holdBtn.addEventListener('keyup', (e) => {
  if (e.key === 'Enter' || e.key === ' ') holdEnd();
});

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

/* ===== 事件日志 ===== */

const logEl = document.getElementById('tp-log')!;
const logs: { t: string; msg: string }[] = [];
function logEvent(msg: string): void {
  const d = new Date();
  logs.unshift({ t: `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`, msg });
  if (logs.length > 8) logs.pop();
  logEl.innerHTML = logs.map((l) => `<li><span class="log-t">${l.t}</span><span>${l.msg}</span></li>`).join('');
}

/* ===== 帧循环：输入状态 → 手柄 SVG ===== */

const lThumb = document.getElementById('l-thumb')!;
const rThumb = document.getElementById('r-thumb')!;
const padIOs = { l: document.getElementById('l-io')!, r: document.getElementById('r-io')! };
const btnEls = {
  lGrip: document.getElementById('l-grip')!, lX: document.getElementById('l-x')!, lY: document.getElementById('l-y')!,
  rA: document.getElementById('r-a')!, rB: document.getElementById('r-b')!,
};
const STICK_R = 8.5; // 摇杆头可移动半径（SVG 单位）

function setBtn(el: HTMLElement, on: boolean): void {
  el.classList.toggle('on', on);
}
let lastRStick = { x: 0, y: 0 };

// DOM 写入去抖：空闲帧不再全量写 textContent/transform（每帧写必脏布局）
let lastLOutput = '', lastROutput = '', lastLTf = '', lastRTf = '';

function frame(): void {
  // 右摇杆：鼠标位置向目标平滑（回中由 mouseleave 置 0 驱动）
  lastRStick.x += (mouseNX - lastRStick.x) * 0.25;
  lastRStick.y += (mouseNY - lastRStick.y) * 0.25;
  input.rStick.x = lastRStick.x; input.rStick.y = lastRStick.y;

  const lTf = 'translate(' + (input.lStick.x * STICK_R).toFixed(2) + ' ' + (-input.lStick.y * STICK_R).toFixed(2) + ')';
  const rTf = 'translate(' + (input.rStick.x * STICK_R).toFixed(2) + ' ' + (-input.rStick.y * STICK_R).toFixed(2) + ')';
  if (lTf !== lastLTf) { lThumb.setAttribute('transform', lTf); lastLTf = lTf; }
  if (rTf !== lastRTf) { rThumb.setAttribute('transform', rTf); lastRTf = rTf; }
  setBtn(btnEls.lGrip, input.lGrip);
  setBtn(btnEls.lX, input.lX);
  setBtn(btnEls.lY, input.lY);
  setBtn(btnEls.rA, input.rA);
  setBtn(btnEls.rB, input.rB);

  const fmt = (v: { x: number; y: number }): string => '(' + v.x.toFixed(1) + ',' + v.y.toFixed(1) + ')';
  const lOut = '杆' + fmt(input.lStick) + ' ' + (input.lGrip ? '握●' : '握—') + ' ' + (input.lX ? 'X●' : 'X—') + ' ' + (input.lY ? 'Y●' : 'Y—');
  const rOut = '杆' + fmt(input.rStick) + ' ' + (input.rA ? 'A●' : 'A—') + ' ' + (input.rB ? 'B●' : 'B—');
  if (lOut !== lastLOutput) { padIOs.l.textContent = lOut; lastLOutput = lOut; }
  if (rOut !== lastROutput) { padIOs.r.textContent = rOut; lastROutput = rOut; }

  sendInputDownlink(); // P5-D：ARMED 态 30Hz 下行（内部节流+状态机总闸）

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

/* ===== 初始 ===== */

setLock('LOCKED');
logEvent('镜像就绪 · LOCKED');
