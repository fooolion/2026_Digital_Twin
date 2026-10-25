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
const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;

// 灯光：环境 + 两盏方向光（工业视口常见的冷白打光）
scene.add(new THREE.AmbientLight(0xbfd4ff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(1.5, -2, 2.5); scene.add(key);
const rim = new THREE.DirectionalLight(0x4da3ff, 0.5); rim.position.set(-2, 1.5, 1); scene.add(rim);

// ===== 真机模型（官方 URDF + 低模视觉网格）=====
// 资产链：官方 reBot-DevArm/Rebot_Arm_description/DM 的 URDF + visual 网格
//   → tools/make_visual_lo.py 减面（21MB→4.4MB，主体件 5000 面/硬件件 1200 面）
//   → web/public/models/visual_lo/ + ReBot_Arm_DM_preview.urdf（mesh 路径已重写）
// URDF 本身 Z-up，与视口约定一致，无需根旋转。
const MAT_FLOOR = new THREE.MeshStandardMaterial({ color: 0x0c1a30, roughness: 0.9, metalness: 0.1, transparent: true, opacity: 0.85 });
const floor = new THREE.Mesh(new THREE.CircleGeometry(0.85, 64), MAT_FLOOR);
floor.position.z = -0.001;
// 网格地面（大屏质感）；GridHelper 默认建在 XZ 平面（Y-up 惯例），Z-up 下需放平 90°
const grid = new THREE.GridHelper(1.7, 24, 0x1f4a78, 0x122a44);
grid.rotation.x = Math.PI / 2;
scene.add(floor, grid);

type UrdfRobot = { joints: Record<string, { setJointValue(v: number): boolean }> } & THREE.Object3D;
let robot: UrdfRobot | null = null;
const loadTag = document.createElement('div');
loadTag.style.cssText = 'position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);z-index:2;font-size:12px;letter-spacing:.08em;color:rgba(0,212,255,.85)';
loadTag.textContent = '加载真机模型…';
host.appendChild(loadTag);

void (async () => {
  const { default: URDFLoader } = await import('urdf-loader');
  try {
    const loader = new URDFLoader();
    const r = await loader.loadAsync(`${import.meta.env.BASE_URL}models/ReBot_Arm_DM_preview.urdf`) as unknown as UrdfRobot;
    robot = r;
    loadTag.remove();
    scene.add(r);
    (window as unknown as Record<string, unknown>).__cpRobot = r; // 调试句柄（现场检查用）
    if (reduced) {
      // 静态展示姿态（与仿真零位一致的伸展）
      r.joints.joint1.setJointValue(0);
      r.joints.joint2.setJointValue(-1.0);
      r.joints.joint3.setJointValue(-0.9);
      r.joints.joint4.setJointValue(0.5);
      r.joints.joint5.setJointValue(0);
      r.joints.joint6.setJointValue(0);
    }
  } catch (e) {
    loadTag.textContent = '模型加载失败（检查网络后刷新）';
    loadTag.style.color = 'rgba(255,180,84,.9)';
    console.error(e);
  }
})();

function resize(): void {
  const w = host.clientWidth || 1, h = host.clientHeight || 1;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

const t0 = performance.now();
renderer.setAnimationLoop(() => {
  if (!reduced && robot) {
    const t = (performance.now() - t0) / 1000;
    robot.joints.joint1.setJointValue(Math.sin(t * 0.35) * 0.55);            // 底座回转
    robot.joints.joint2.setJointValue(-0.95 + Math.sin(t * 0.5) * 0.22);     // 大臂俯仰
    robot.joints.joint3.setJointValue(0.75 + Math.sin(t * 0.5 + 1.1) * 0.2); // 小臂俯仰
    robot.joints.joint4.setJointValue(0.2 + Math.sin(t * 0.7 + 2) * 0.12);   // 腕部
    robot.joints.joint5.setJointValue(Math.sin(t * 0.45 + 0.5) * 0.3);
    const grip = 0.5 + Math.sin(t * 0.9) * 0.5;                              // 夹爪开合（0~0.02m 行程）
    robot.joints.finger_left.setJointValue(grip * 0.02);
    robot.joints.finger_right.setJointValue(grip * 0.02);
  }
  controls.update();
  renderer.render(scene, camera);
});
