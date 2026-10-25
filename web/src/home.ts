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
controls.minDistance = 0.4; controls.maxDistance = 2.8;
controls.enablePan = true; // 右键拖拽平移
controls.maxPolarAngle = Math.PI * 0.55; // 允许略低于地平线，但别钻到地下
const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;

// 灯光：环境 + 两盏方向光（工业视口常见的冷白打光）
scene.add(new THREE.AmbientLight(0xbfd4ff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(1.5, -2, 2.5); scene.add(key);
const rim = new THREE.DirectionalLight(0x4da3ff, 0.5); rim.position.set(-2, 1.5, 1); scene.add(rim);

// ===== 真机模型（官方 URDF + 原始全分辨率视觉网格，~21MB）=====
// 资产链：官方 reBot-DevArm/Rebot_Arm_description/DM 的 URDF + visual 分色网格
//   → 原样拷贝 web/public/models/visual/（用户 2026-10-25 决策：不减面，保真优先；
//     曾试减面 21MB→4.4MB 但视觉损失明显被否决）+ ReBot_Arm_DM_preview.urdf（路径重写）
// 首次加载 ~21MB：进度条 + 说明；浏览器缓存后秒开。URDF 原生 Z-up 与视口一致。
const MAT_FLOOR = new THREE.MeshStandardMaterial({ color: 0x0c1a30, roughness: 0.9, metalness: 0.1, transparent: true, opacity: 0.85 });
const floor = new THREE.Mesh(new THREE.CircleGeometry(0.85, 64), MAT_FLOOR);
floor.position.z = -0.001;
// 网格地面（大屏质感）；GridHelper 默认建在 XZ 平面（Y-up 惯例），Z-up 下需放平 90°
const grid = new THREE.GridHelper(1.7, 24, 0x1f4a78, 0x122a44);
grid.rotation.x = Math.PI / 2;
scene.add(floor, grid);

type UrdfRobot = { joints: Record<string, { setJointValue(v: number): boolean }> } & THREE.Object3D;
let robot: UrdfRobot | null = null;

// 演示摆动姿态方程（URDF 限位内；动画循环与自动取景共用）
function setPose(rob: UrdfRobot, t: number): void {
  rob.joints.joint1.setJointValue(Math.sin(t * 0.35) * 0.55);
  rob.joints.joint2.setJointValue(-0.95 + Math.sin(t * 0.5) * 0.22);
  rob.joints.joint3.setJointValue(-0.9 + Math.sin(t * 0.5 + 1.1) * 0.2);
  rob.joints.joint4.setJointValue(0.2 + Math.sin(t * 0.7 + 2) * 0.12);
  rob.joints.joint5.setJointValue(Math.sin(t * 0.45 + 0.5) * 0.3);
  const grip = 0.5 + Math.sin(t * 0.9) * 0.5;
  rob.joints.finger_left.setJointValue(grip * 0.02);   // [0, 0.05]
  rob.joints.finger_right.setJointValue(-grip * 0.02); // [-0.05, 0]
}

// 加载覆盖层：说明 + 字节级进度条
const loadMask = document.createElement('div');
loadMask.style.cssText = 'position:absolute;inset:0;z-index:2;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px;background:rgba(6,13,26,.55);backdrop-filter:blur(2px);transition:opacity .5s ease';
loadMask.innerHTML = `
  <div style="font-size:13px;letter-spacing:.1em;color:rgba(0,212,255,.9)">正在加载真机模型（reBot B601-DM）</div>
  <div style="width:min(280px,70%);height:4px;border-radius:2px;background:rgba(0,212,255,.12);overflow:hidden">
    <div id="cp-load-bar" style="width:0%;height:100%;border-radius:2px;background:linear-gradient(90deg,#2e8bff,#00d4ff);box-shadow:0 0 10px rgba(0,212,255,.6);transition:width .25s ease"></div>
  </div>
  <div id="cp-load-txt" style="font-size:11px;color:rgba(255,255,255,.45)">准备中…</div>
  <div style="font-size:10.5px;color:rgba(255,255,255,.30);max-width:300px;text-align:center;line-height:1.7">完整视觉模型约 21 MB，首次加载需 10~60 秒（取决于网络）；<br>加载一次后浏览器缓存，之后秒开</div>`;
host.appendChild(loadMask);
const barEl = loadMask.querySelector('#cp-load-bar') as HTMLElement;
const txtEl = loadMask.querySelector('#cp-load-txt') as HTMLElement;

void (async () => {
  const { default: URDFLoader } = await import('urdf-loader');
  try {
    const manager = new THREE.LoadingManager();
    manager.onProgress = (_url, loaded, total) => {
      const pct = total > 0 ? Math.round((loaded / total) * 100) : 0;
      barEl.style.width = `${pct}%`;
      txtEl.textContent = `网格 ${Math.min(loaded, total)}/${total} · ${pct}%`;
    };
    manager.onError = (url) => { txtEl.textContent = `部分资源加载失败：${url.split('/').pop()}`; };
    const loader = new URDFLoader(manager);
    const r = await loader.loadAsync(`${import.meta.env.BASE_URL}models/ReBot_Arm_DM_preview.urdf`) as unknown as UrdfRobot;
    robot = r;
    barEl.style.width = '100%';
    txtEl.textContent = '完成';
    loadMask.style.opacity = '0';
    window.setTimeout(() => loadMask.remove(), 550);
    scene.add(r);
    // 自动取景：按摆动包络取景——采样动画方程 4 个相位取并集包围盒，
    // 使任意动画相位下臂都大致居中（URDF 零位是折叠态，不能按零位量）
    requestAnimationFrame(() => {
      const box = new THREE.Box3();
      const tmp = new THREE.Box3();
      for (const tk of [0, 1.6, 3.1, 4.7]) {
        setPose(r, tk);
        tmp.setFromObject(r);
        box.union(tmp);
      }
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const radius = size.length() * 0.5;
      const dist = Math.min(Math.max((radius / Math.sin(THREE.MathUtils.degToRad(camera.fov * 0.5))) * 1.3, 1.15), controls.maxDistance);
      const dir = new THREE.Vector3().subVectors(camera.position, controls.target).normalize();
      controls.target.copy(center);
      camera.position.copy(center).addScaledVector(dir, dist);
      controls.update();
    });
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
    txtEl.textContent = '模型加载失败（检查网络后刷新）';
    txtEl.style.color = 'rgba(255,180,84,.9)';
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
  if (!reduced && robot) setPose(robot, (performance.now() - t0) / 1000);
  controls.update();
  renderer.render(scene, camera);
});
