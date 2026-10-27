/**
 * 总览驾驶舱（老师工作流入口：操作员工作台）。
 * 布局：左系统状态 + 中真机 URDF 三维视口 + 右 KPI + 底部四步工作流。
 * 数据：SYSTEM/SCENE/KPIS/FLOW 顶部配置对象（场景尺寸/电池建模/链路状态定稿后改配置即联动）。
 * P5 接入点：SYSTEM 状态灯改轮询伴生服务。
 * Rev: R2 · 2026-10-25
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { mountNav, renderStatList, type StatItem } from './nav';
import { watchCompanion } from './companion';
import { PAGE_HREF } from './nav';
import './theme.css';
import './cockpit.css';

/* ===== 配置区（数据驱动预留） ===== */

// 系统链路状态：lamp = ok | warn | off
const SYSTEM: (StatItem & { href: string; probe?: 'companion' })[] = [
  { name: '仿真引擎（MuJoCo WASM）', lamp: 'ok', state: '在线', href: 'sim.html' },
  { name: 'PICO · Unity VR 链路', lamp: 'warn', state: '待接入（P5）', href: 'live.html' },
  { name: '实体机械臂 B601-DM', lamp: 'off', state: '离线 · 未使能', href: 'teleop.html' },
  { name: '伴生服务（视频/桥接）', lamp: 'warn', state: '探测中…', href: 'live.html', probe: 'companion' },
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

// 四步工作流（对应导航①-④）：state = done | now | todo。
// 总览不属于任何步骤——默认全 todo（"下一步去哪"由当前任务卡与 CTA 按钮承担，bar 不冒充进度）；
// done/now 样式钩子保留，将来接真实进度（如桥接状态驱动）直接填值。
const FLOW = [
  { no: 1, key: 'workshop', title: '零件库', sub: 'STEP 01 · 导入电池零件', state: 'todo' },
  { no: 2, key: 'sim', title: '仿真规划', sub: 'STEP 02 · 路径与动作', state: 'todo' },
  { no: 3, key: 'teleop', title: '遥操作', sub: 'STEP 03 · 拆卸步骤设定', state: 'todo' },
  { no: 4, key: 'live', title: '运行监控', sub: 'STEP 04 · VR 与相机画面', state: 'todo' },
];

/* ===== 渲染（配置 → DOM） ===== */

mountNav('home');

renderStatList(document.getElementById('stat-list')!, SYSTEM);

// 双模探测：统一走 companion.ts 的 watchCompanion（周期重探+变化才回调，服务后启动也能转绿）
watchCompanion((h) => {
  const idx = SYSTEM.findIndex((s) => s.probe === 'companion');
  if (idx < 0) return;
  if (h?.ok) {
    SYSTEM[idx] = { ...SYSTEM[idx], lamp: 'ok', state: `在线 · 桥 ${h.bridge_clients} 客户端` };
  } else {
    SYSTEM[idx] = { ...SYSTEM[idx], lamp: 'warn', state: '未启动（演示模式）' };
  }
  renderStatList(document.getElementById('stat-list')!, SYSTEM);
});


document.getElementById('scene-list')!.innerHTML = SCENE.map((s) => `
  <li><b>${s.name}</b><span class="val">${s.value}</span></li>`).join('');

document.getElementById('kpi-list')!.innerHTML = KPIS.map((k) => `
  <div class="kpi"><b class="kpi-num">${k.v}</b><span>${k.u}</span></div>`).join('');


document.getElementById('flow-bar')!.innerHTML = FLOW.map((f) => `
  <a class="flow-step2 ${f.state}" href="${PAGE_HREF[f.key]}">
    <span class="step-no">${f.no}</span>
    <span class="fs-txt"><span>${f.title}</span><small>${f.sub}</small></span>
  </a>`).join('');

/* ===== 中央轻量三维视口 ===== */

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
  // 小幅摆动：包络紧凑，臂始终保持在画面中心附近（大幅摆动会漂出取景）
  rob.joints.joint1.setJointValue(Math.sin(t * 0.35) * 0.28);
  rob.joints.joint2.setJointValue(-0.95 + Math.sin(t * 0.5) * 0.12);
  rob.joints.joint3.setJointValue(-0.9 + Math.sin(t * 0.5 + 1.1) * 0.12);
  rob.joints.joint4.setJointValue(0.2 + Math.sin(t * 0.7 + 2) * 0.08);
  rob.joints.joint5.setJointValue(Math.sin(t * 0.45 + 0.5) * 0.18);
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
    // 传输压缩（审查杠杆 1）：构建时已预压 .gz（21MB visual → ~6.5MB 传输）。
    // URDF 文本与 STL 全部手动 fetch .gz → 解压 → blob URL；loader 经 URL 重写消费 blob。
    const MODEL_BASE = `${import.meta.env.BASE_URL}models`;
    const gzBytes = async (relUrl: string): Promise<ArrayBuffer> => {
      const r = await fetch(`${MODEL_BASE}/${relUrl}.gz`);
      if (!r.ok) throw new Error(`HTTP ${r.status}: ${relUrl}.gz`);
      const stream = r.body!.pipeThrough(new DecompressionStream('gzip'));
      return new Response(stream).arrayBuffer();
    };
    const urdfRel = 'ReBot_Arm_DM_preview.urdf';
    const urdfText = new TextDecoder().decode(await gzBytes(urdfRel));

    const stlRefs = [...new Set([...urdfText.matchAll(/([\w./-]+\.stl)/g)].map((m) => m[1]))];
    const total = stlRefs.length;
    const blobMap = new Map<string, string>();
    let done = 0;
    await Promise.all(
      stlRefs.map(async (rel) => {
        const bytes = await gzBytes(rel);
        blobMap.set(rel, URL.createObjectURL(new Blob([bytes])));
        done += 1;
        const pct = total > 0 ? Math.round((done / total) * 100) : 0;
        barEl.style.width = `${pct}%`;
        txtEl.textContent = `网格 ${done}/${total} · ${pct}%`;
      }),
    );

    const mgr = new THREE.LoadingManager();
    mgr.setURLModifier((url: string) => {
      const rel = url.split('/models/')[1]?.split('?')[0];
      return (rel && blobMap.get(rel)) || url;
    });
    const loader = new URDFLoader(mgr);
    const r = await new Promise<UrdfRobot>((res, rej) => {
      loader.load(`${MODEL_BASE}/${urdfRel}`, (robot) => res(robot as unknown as UrdfRobot), undefined, rej);
    });
    robot = r;
    barEl.style.width = '100%';
    txtEl.textContent = '完成';
    loadMask.style.opacity = '0';
    window.setTimeout(() => loadMask.remove(), 550);
    scene.add(r);
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
new ResizeObserver(resize).observe(host); // 滚动条出现压缩容器宽时 window resize 不触发（与 workshop 同款）
resize();

// 自动取景（两段）：① 密采样摆动包络（16 相位并集）定距离，保证摆动全程不出画面；
// ② NDC 屏幕投影迭代校正，居中目标=展示姿态（t=0）包围盒——包络中心偏向末端扫掠侧，
//    会让臂身恒定偏一侧，臂身居中才是视觉正确。固定视线方向（方位 30°/仰角 25°）+
//    up=(0,0,1)+lookAt → 地平面在画面中数学上水平（无 roll）。
function doFit(r: UrdfRobot): void {
  const box = new THREE.Box3();
  const tmp = new THREE.Box3();
  for (let k = 0; k < 16; k++) {
    setPose(r, (k / 16) * (Math.PI * 2 / 0.35)); // 覆盖最慢关节的完整周期
    tmp.setFromObject(r);
    if (tmp.isEmpty()) return;
    box.union(tmp);
  }
  if (box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3());
  const radius = size.length() * 0.5;
  if (!Number.isFinite(radius) || radius <= 0) return;
  const dist = Math.min(Math.max((radius / Math.sin(THREE.MathUtils.degToRad(camera.fov * 0.5))) * 1.5, 1.3), controls.maxDistance);
  const AZ = THREE.MathUtils.degToRad(18);
  const EL = THREE.MathUtils.degToRad(35);
  const dir = new THREE.Vector3(Math.cos(EL) * Math.sin(AZ), -Math.cos(EL) * Math.cos(AZ), Math.sin(EL));
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(dir, dist);
  camera.up.set(0, 0, 1);
  camera.lookAt(controls.target);
  controls.update();

  setPose(r, 0);
  const vbox = new THREE.Box3().setFromObject(r);
  if (vbox.isEmpty()) return;
  const corners: THREE.Vector3[] = [];
  for (const x of [vbox.min.x, vbox.max.x]) for (const y of [vbox.min.y, vbox.max.y]) for (const z of [vbox.min.z, vbox.max.z]) corners.push(new THREE.Vector3(x, y, z));
  const right = new THREE.Vector3();
  const up = new THREE.Vector3();
  for (let it = 0; it < 3; it++) {
    camera.updateMatrixWorld();
    let sx = 0, sy = 0;
    for (const c of corners) {
      const v = c.clone().project(camera);
      if (!Number.isFinite(v.x) || !Number.isFinite(v.y)) return;
      sx += v.x; sy += v.y;
    }
    sx /= corners.length; sy /= corners.length;
    if (Math.abs(sx) < 0.01 && Math.abs(sy) < 0.01) break;
    const halfH = Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5)) * dist;
    right.setFromMatrixColumn(camera.matrixWorld, 0);
    up.setFromMatrixColumn(camera.matrixWorld, 1);
    // 符号：target/相机沿 +right(+up) 平移 → 画面内容左移(下移)——投影偏右(sx>0)时正需要左移
    controls.target.addScaledVector(right, sx * halfH * camera.aspect);
    controls.target.addScaledVector(up, sy * halfH);
    camera.position.copy(controls.target).addScaledVector(dir, dist);
    camera.lookAt(controls.target);
    controls.update();
  }
  // 构图微调：臂略偏左上，底部按钮区留白（用户认可构图）
  const halfH2 = Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5)) * dist;
  right.setFromMatrixColumn(camera.matrixWorld, 0);
  up.setFromMatrixColumn(camera.matrixWorld, 1);
  controls.target.addScaledVector(right, 0.16 * halfH2 * camera.aspect);
  controls.target.addScaledVector(up, 0.09 * halfH2);
  camera.position.copy(controls.target).addScaledVector(dir, dist);
  camera.lookAt(controls.target);
  controls.update();
  // 地台跟随：移到臂身（展示姿态）中心正下方
  const cxy = vbox.getCenter(new THREE.Vector3());
  floor.position.set(cxy.x, cxy.y, -0.001);
  grid.position.set(cxy.x, cxy.y, 0);
}

const t0 = performance.now();
let fitted = false;
const countMeshes = (root: THREE.Object3D): number => {
  let n = 0;
  root.traverse((o) => { if ((o as THREE.Mesh).isMesh && (o as THREE.Mesh).geometry?.attributes?.position) n++; });
  return n;
};
renderer.setAnimationLoop(() => {
  if (robot) {
    if (!reduced) setPose(robot, (performance.now() - t0) / 1000);
    // fit 必须等全部网格就位：loadAsync 在骨架建好时即 resolve，网格异步继续到，
    // 提前量包围盒会得到空盒（NDC 投影 NaN、相机飞走）——每帧检查，齐了执行一次
    if (!fitted && countMeshes(robot) >= 30) {
      fitted = true;
      doFit(robot);
    }
  }
  controls.update();
  renderer.render(scene, camera);
});
