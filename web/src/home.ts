/**
 * 总览驾驶舱（2026-10-25 R2 重写，指导老师方向：操作员工作台而非宣传页）。
 * 布局：左系统状态 + 中三维视口 + 右 KPI + 底部四步工作流。
 *
 * ⚠ 数据驱动预留：SYSTEM/SCENE/KPIS/FLOW 全部集中在本文件顶部配置对象——
 * 场景尺寸、电池建模、真机链路状态确定后改这里即可，不动 DOM 结构。
 * 三维视口为轻量程序化场景（零模型下载，保首屏速度）；完整 MuJoCo 仿真在 sim.html。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { mountNav } from './nav';
import './theme.css';
import './cockpit.css';

/* ================= 配置区（数据驱动预留） ================= */

// 系统链路状态：lamp = ok | warn | off
const SYSTEM = [
  { name: '仿真引擎（MuJoCo WASM）', lamp: 'ok', state: '在线', href: 'sim.html' },
  { name: 'PICO · Unity VR 链路', lamp: 'warn', state: '待接入（P5）', href: 'live.html' },
  { name: '实体机械臂 B601-DM', lamp: 'off', state: '离线 · 未使能', href: 'teleop.html' },
  { name: '伴生服务（视频/桥接）', lamp: 'warn', state: '未部署（P5）', href: 'live.html' },
];

// 场景配置（占位值——实际场景位置尺寸与电池建模确定后在此更新）
const SCENE = [
  { name: '电池模型', value: '占位样件（40mm 立方）' },
  { name: '工位尺寸', value: '待定' },
  { name: '夹爪类型', value: '齿轮-齿条双指' },
  { name: '相机配置', value: '双工业相机（待标定）' },
];

const KPIS = [
  { v: '6+1', u: 'DOF' }, { v: '±0.2', u: 'mm 重复定位' },
  { v: '≤5', u: 'ms 急停响应' }, { v: 'mm 级', u: 'TCP 拖拽跟踪' },
];

// 四步工作流（对应导航①-④）：state = done | now | todo
const FLOW = [
  { no: 1, key: 'workshop', title: '零件库', sub: 'STEP 01 · 导入电池零件', state: 'todo' },
  { no: 2, key: 'sim', title: '仿真规划', sub: 'STEP 02 · 路径与动作', state: 'now' },
  { no: 3, key: 'teleop', title: '遥操作', sub: 'STEP 03 · 拆卸步骤设定', state: 'todo' },
  { no: 4, key: 'live', title: '运行监控', sub: 'STEP 04 · VR 与相机画面', state: 'todo' },
];

/* ================= 渲染（配置 → DOM） ================= */

mountNav('home');

const LAMP_CLS: Record<string, string> = { ok: '', warn: 'lamp-warn', off: 'lamp-alarm' };
const VAL_CLS: Record<string, string> = { ok: 'on', warn: 'warn', off: '' };

document.getElementById('stat-list')!.innerHTML = SYSTEM.map((s) => `
  <li><span class="lamp ${LAMP_CLS[s.lamp]}"></span><b>${s.name}</b><span class="val ${VAL_CLS[s.lamp]}">${s.state}</span></li>`).join('');

document.getElementById('scene-list')!.innerHTML = SCENE.map((s) => `
  <li><b>${s.name}</b><span class="val">${s.value}</span></li>`).join('');

document.getElementById('kpi-list')!.innerHTML = KPIS.map((k) => `
  <div class="kpi"><b class="kpi-num">${k.v}</b><span>${k.u}</span></div>`).join('');

const FLOW_HREF: Record<string, string> = { workshop: 'workshop.html', sim: 'sim.html', teleop: 'teleop.html', live: 'live.html' };
document.getElementById('flow-bar')!.innerHTML = FLOW.map((f) => `
  <a class="flow-step2 ${f.state}" href="${FLOW_HREF[f.key]}">
    <span class="step-no">${f.no}</span>
    <span class="fs-txt"><span>${f.title}</span><small>${f.sub}</small></span>
  </a>`).join('');

/* ================= 中央轻量三维视口 ================= */

const host = document.getElementById('cockpit-3d')!;
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
host.appendChild(renderer.domElement);

// Z-up（与 sim 视口一致）
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 50);
camera.up.set(0, 0, 1);
camera.position.set(0.72, -0.92, 0.5);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.24);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.5; controls.maxDistance = 2.4;
controls.enablePan = false;

// 灯光：环境 + 两盏方向光（工业视口常见的冷白打光）
scene.add(new THREE.AmbientLight(0xbfd4ff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(1.5, -2, 2.5); scene.add(key);
const rim = new THREE.DirectionalLight(0x4da3ff, 0.5); rim.position.set(-2, 1.5, 1); scene.add(rim);

// 程序化轻量臂（示意：底座 + 肩 + 大臂 + 小臂 + 腕 + 双指）
const MAT_BODY = new THREE.MeshStandardMaterial({ color: 0x9aa7b8, roughness: 0.5, metalness: 0.35 });
const MAT_JOINT = new THREE.MeshStandardMaterial({ color: 0x2c3a4e, roughness: 0.4, metalness: 0.5 });
const MAT_ACC = new THREE.MeshStandardMaterial({ color: 0x2ee584, roughness: 0.35, metalness: 0.2, emissive: 0x0a3d22 });
const MAT_FLOOR = new THREE.MeshStandardMaterial({ color: 0x0c1a30, roughness: 0.9, metalness: 0.1, transparent: true, opacity: 0.85 });

const floor = new THREE.Mesh(new THREE.CircleGeometry(0.85, 64), MAT_FLOOR);
// Z-up 世界：CircleGeometry 默认 XY 平面即水平地面，无需旋转（投放环 bug 的教训）
floor.position.z = -0.001;
// 网格地面（大屏质感）；GridHelper 默认建在 XZ 平面（Y-up 惯例），Z-up 下需放平 90°
const grid = new THREE.GridHelper(1.7, 24, 0x1f4a78, 0x122a44);
grid.rotation.x = Math.PI / 2;
scene.add(floor, grid);

// 注：Z-up 世界，竖直圆柱需 rotation.x = π/2（CylinderGeometry 默认轴向 Y）；横向关节圆柱保持默认
const base = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.105, 0.06, 32), MAT_BODY);
base.rotation.x = Math.PI / 2; base.position.z = 0.03; scene.add(base);

const j1 = new THREE.Group(); j1.position.z = 0.06; scene.add(j1);
const shoulder = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.09, 24), MAT_JOINT);
shoulder.rotation.x = Math.PI / 2; shoulder.position.z = 0.02; j1.add(shoulder);

const j2 = new THREE.Group(); j2.position.z = 0.065; j1.add(j2);
const upper = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.05, 0.3), MAT_BODY);
upper.position.z = 0.15; j2.add(upper);
const elbow = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 0.075, 24), MAT_JOINT);
elbow.position.z = 0.3; j2.add(elbow);

const j3 = new THREE.Group(); j3.position.z = 0.3; j2.add(j3);
const fore = new THREE.Mesh(new THREE.BoxGeometry(0.042, 0.04, 0.24), MAT_BODY);
fore.position.z = 0.12; j3.add(fore);
const wrist = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.06, 20), MAT_JOINT);
wrist.position.z = 0.24; j3.add(wrist);

const j4 = new THREE.Group(); j4.position.z = 0.25; j3.add(j4);
const palm = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.05, 0.045), MAT_ACC);
palm.position.z = 0.022; j4.add(palm);
const fingerL = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.032, 0.085), MAT_BODY);
fingerL.position.set(0.022, 0, 0.085);
const fingerR = fingerL.clone(); fingerR.position.x = -0.022;
j4.add(fingerL, fingerR);

// TCP 指示点（与仿真工作台同语义：绿色目标/位置球）
const tcpDot = new THREE.Mesh(new THREE.SphereGeometry(0.011, 16, 16), MAT_ACC.clone());
tcpDot.material.emissiveIntensity = 2.2;
tcpDot.position.z = 0.135; j4.add(tcpDot);

function resize(): void {
  const w = host.clientWidth || 1, h = host.clientHeight || 1;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
const t0 = performance.now();
renderer.setAnimationLoop(() => {
  if (!reduced) {
    const t = (performance.now() - t0) / 1000;
    j1.rotation.z = Math.sin(t * 0.35) * 0.55;                 // 底座回转
    j2.rotation.x = -0.95 + Math.sin(t * 0.5) * 0.22;          // 大臂俯仰
    j3.rotation.x = 0.75 + Math.sin(t * 0.5 + 1.1) * 0.2;      // 小臂俯仰
    j4.rotation.x = 0.2 + Math.sin(t * 0.7 + 2) * 0.12;        // 腕部
    const grip = 0.5 + Math.sin(t * 0.9) * 0.5;                // 夹爪开合
    fingerL.position.x = 0.016 + grip * 0.016;
    fingerR.position.x = -0.016 - grip * 0.016;
  }
  controls.update();
  renderer.render(scene, camera);
});
