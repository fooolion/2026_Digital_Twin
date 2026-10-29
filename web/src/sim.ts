/**
 * 仿真规划工作台（老师工作流第②步）：浏览器内真 MuJoCo 物理 + three.js 渲染。
 * 功能：TCP 拖拽（绿球）/ 关节面板 / 一键夹取演示 / 路径录制回放 / 下发实体臂弹窗（UI 先行）。
 * 控制：CTC ctrl = bias + KP·e − KD·q̇（KP/KD 照参考站真机同步参数；timestep 1ms）。
 * P5 接入点：「下发实体臂」弹窗接真机链路后开放。
 * Rev: P1-P4 · 2026-10-22
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';
import { createTcpIk } from './tcp-ik';
import { createDemo } from './demo';
import { mountNav, toast } from './nav';
import { mountBackdrop } from './backdrop';
import './theme.css';
import './sim.css';

mountBackdrop();
mountNav('sim');
import { createJointPanel } from './panel';

// VFS 内路径（MuJoCo 虚拟文件系统里 meshdir="meshes" 相对模型解析）
const MODEL_VFS = 'ReBot_Arm_DM.xml';
const MESH_VFS_PREFIX = 'meshes/';
// HTTP 路径（vite public/ 目录原样伺服）。必须拼 BASE_URL：Pages 部署在仓库子路径下，
// 写死 '/models/...' 会请求到域名根 → 404
const MODEL_URL = `${import.meta.env.BASE_URL}models/ReBot_Arm_DM.xml`;
const MESH_URL = `${import.meta.env.BASE_URL}models/meshes/`;

const statusEl = document.getElementById('status') as HTMLElement;
const setStatus = (s: string) => { statusEl.textContent = s; };

/** 传输压缩取数：优先 .gz（构建时预压，STL 压到 ~30%、wasm ~25%）+ 浏览器原生解压；
 *  .gz 404（旧部署/伴生服务旧版）→ 回退原文件。 */
async function fetchBytesCompressed(url: string): Promise<Uint8Array> {
  try {
    const r = await fetch(`${url}.gz`);
    if (r.ok) {
      const ds = new DecompressionStream('gzip');
      const stream = r.body!.pipeThrough(ds);
      return new Uint8Array(await new Response(stream).arrayBuffer());
    }
  } catch { /* 解压/网络失败回退原文件 */ }
  const r = await fetch(url);
  if (!r.ok) throw new Error(`HTTP ${r.status}: ${url}`);
  return new Uint8Array(await r.arrayBuffer());
}

// 兼容旧名（loadMujoco 内部与网格循环共用）
const fetchBytes = fetchBytesCompressed;

/** 从 MJCF 文本提取全部网格相对路径（asset 里的 file=".stl" 属性） */
function meshFilesIn(xml: string): string[] {
  const out = new Set<string>();
  const re = /file="([^"]+\.(?:stl|msh|obj))"/gi;
  let m: RegExpExecArray | null;
  while ((m = re.exec(xml))) out.add(m[1]);
  return [...out];
}

async function main(): Promise<void> {
  // 加载并行化：WASM 与 XML/网格预取同时开始（原实现严格串行，
  // 10.75MB wasm + 21MB 网格带宽不重叠——首屏多等 ~10s @10Mbps）
  setStatus('加载 MuJoCo WASM 与模型（~8MB 传输压缩后）…');
  const meshesPromise = fetchBytes(MODEL_URL).then(async (xmlBytes) => {
    const xmlText = new TextDecoder().decode(xmlBytes);
    const meshes = await Promise.all(
      meshFilesIn(xmlText).map(async (f) => ({ f, bytes: await fetchBytes(MESH_URL + f) })),
    );
    return { xmlBytes, meshes };
  });
  const [mujoco, modelBundle] = await Promise.all([loadMujoco(), meshesPromise]);
  const { xmlBytes, meshes } = modelBundle;

  const vfs = new mujoco.MjVFS();
  vfs.addBuffer(MODEL_VFS, xmlBytes);
  for (const { f, bytes } of meshes) {
    vfs.addBuffer(MESH_VFS_PREFIX + f, bytes);
  }

  setStatus('编译 MJCF…');
  const model: any = mujoco.MjModel.from_xml_path(MODEL_VFS, vfs);
  vfs.delete(); // 编译完成，MjVFS 里 ~21MB 网格 buffer 已拷入模型——释放堆内存
  const data: any = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  (window as any).__mj = { mujoco, model, data }; // 调试句柄（控制台可现场检查）
  console.log(
    `[MJ] 编译成功 nq=${model.nq} nv=${model.nv} nbody=${model.nbody} ` +
    `ngeom=${model.ngeom} nu=${model.nu} neq=${model.neq} ntendon=${model.ntendon}`,
  );

  // ===== Three.js 场景（Z-up，与 MuJoCo 一致）=====
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setClearColor(0x000000, 0); // 透明清屏：露出 .ambient 统一背景（粒子星野+地平线）
  renderer.setSize(innerWidth, innerHeight);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  document.body.appendChild(renderer.domElement);
  window.addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  const scene = new THREE.Scene();
  scene.background = null; // 透明场景：页面背景层（粒子/地平线）从画布边缘透出（三页统一）

  // ===== 方案3+6 场景内背景：地平线渐变幕布 + 厂房剪影 + 尘埃粒子 =====
  // （仿真页是全屏 3D，页面背景层只露出边缘——把同一视觉语言搬进场景才能看见）
  {
    const cv = document.createElement('canvas');
    cv.width = 2048; cv.height = 640;
    const c = cv.getContext('2d')!;
    const g = c.createLinearGradient(0, 0, 0, 640);
    g.addColorStop(0, 'rgba(10,17,32,0)'); // 顶边透明渐隐：幕布上缘融入页面背景，不见接缝
    g.addColorStop(0.45, '#0d1c33');
    g.addColorStop(0.72, '#14304f'); g.addColorStop(0.78, '#0e2138'); g.addColorStop(1, '#0b1526');
    c.fillStyle = g; c.fillRect(0, 0, 2048, 640);
    const lg = c.createLinearGradient(0, 0, 2048, 0);
    lg.addColorStop(0, 'rgba(120,220,255,0)'); lg.addColorStop(0.5, 'rgba(140,225,255,.55)');
    lg.addColorStop(1, 'rgba(57,217,138,0)');
    c.fillStyle = lg; c.fillRect(0, 452, 2048, 2);
    c.save();
    c.setTransform(2048 / 1440, 0, 0, -1.05, 0, 453); // 剪影坐在辉光线上、向上展开
    c.fillStyle = 'rgba(22,38,66,.55)';
    c.fill(new Path2D('M0 140 V96 h90 v-18 h70 v18 h120 V70 h110 v70 h180 V88 h130 v-16 h80 v16 h150 V78 h140 v62 z'));
    c.fillStyle = '#0c1830';
    c.fill(new Path2D('M40 140 V92 l36 -22 l34 22 v-4 h8 v-18 l36 -22 l34 22 V140 Z'));
    c.fill(new Path2D('M238 34 h16 v106 h-16 Z'));
    c.fill(new Path2D('M300 140 V78 a26 26 0 0 1 52 0 V140 Z'));
    c.fill(new Path2D('M430 140 V56 h210 v10 h-14 v74 h-12 V66 h-158 v74 h-12 V66 h-14 Z'));
    c.fill(new Path2D('M700 96 h260 v44 h-260 Z'));
    c.fill(new Path2D('M1010 140 V84 l36 -22 l34 22 v-6 h10 V60 l36 -22 l34 22 v14 h12 V140 Z'));
    c.fill(new Path2D('M1190 52 h14 v88 h-14 Z'));
    c.fillStyle = 'rgba(57,217,138,.16)';
    c.fillRect(86 / 1.42, 0, 0, 0); // 占位（窗灯在下方逐个画）
    c.restore();
    // 窗灯（世界感点缀）
    c.fillStyle = 'rgba(57,217,138,.16)';
    for (const [wx, wy] of [[86, 104], [120, 104], [740, 110], [800, 110], [1240, 96]] as const) {
      c.fillRect(wx * (2048 / 1440), 453 - wy * 1.05, 5 * 1.42, 7 * 1.05);
    }
    c.fillStyle = 'rgba(79,216,255,.14)';
    for (const [wx, wy] of [[452, 70], [1092, 70]] as const) {
      c.fillRect(wx * (2048 / 1440), 453 - wy * 1.05, 5 * 1.42, 7 * 1.05);
    }
    const bdTex = new THREE.CanvasTexture(cv);
    bdTex.colorSpace = THREE.SRGBColorSpace;
    const backdrop = new THREE.Mesh(
      new THREE.PlaneGeometry(16, 5.2),
      new THREE.MeshBasicMaterial({ map: bdTex }),
    );
    backdrop.position.set(0, -4.2, 1.7);
    backdrop.up.set(0, 0, 1);
    backdrop.lookAt(0, 1.7, 0.75);
    scene.add(backdrop);

    // 尘埃粒子（机械臂周围缓慢漂浮）
    const N = 260;
    const pos = new Float32Array(N * 3);
    for (let i = 0; i < N; i++) {
      pos[i * 3] = (Math.random() - 0.5) * 5;
      pos[i * 3 + 1] = (Math.random() - 0.5) * 4 - 0.5;
      pos[i * 3 + 2] = 0.05 + Math.random() * 2.2;
    }
    const dustGeo = new THREE.BufferGeometry();
    dustGeo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const dust = new THREE.Points(dustGeo, new THREE.PointsMaterial({
      color: 0x9fd4ff, size: 0.012, transparent: true, opacity: 0.5,
      blending: THREE.AdditiveBlending, depthWrite: false, sizeAttenuation: true,
    }));
    scene.add(dust);
    (window as any).__dust = dust; // 渲染循环里做缓慢旋涡（见 tick）
  }

  const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.01, 20);
  camera.up.set(0, 0, 1);
  camera.position.set(0, 1.7, 0.75); // 自 +y 朝向机械臂（定稿）：臂、电池、样件与投放环全景入画

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.35);
  controls.enableDamping = true;

  scene.add(new THREE.AmbientLight(0xffffff, 0.5));
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(1.5, -1, 2);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x88aaff, 0.6);
  fill.position.set(-1.5, 1, 0.5);
  scene.add(fill);

  const grid = new THREE.GridHelper(2, 20, 0x223355, 0x14203a);
  grid.rotation.x = Math.PI / 2; // GridHelper 默认在 XZ 平面，转到 Z-up 的 XY 平面
  scene.add(grid);

  // ===== 从编译结果生成 Three.js 网格 =====
  const meshGeoCache = new Map<number, THREE.BufferGeometry>();
  function meshGeometry(mi: number): THREE.BufferGeometry {
    const cached = meshGeoCache.get(mi);
    if (cached) return cached;

    const vadr = model.mesh_vertadr[mi];
    const vnum = model.mesh_vertnum[mi];
    const fadr = model.mesh_faceadr[mi];
    const fnum = model.mesh_facenum[mi];
    const vert = model.mesh_vert as Float32Array;
    const face = model.mesh_face as Int32Array;

    // 实测（2026-10-18 浏览器现场核查）：3.16.0 编译器把顶点直接存进 geom 系
    // （geom_pos/quat ≡ mesh_pos/quat），顶点原样使用，勿再叠 mesh_pos 烘焙（会双重偏移）
    const pos = new Float32Array(vnum * 3);
    for (let i = 0; i < vnum * 3; i++) pos[i] = vert[vadr * 3 + i];
    const idx = new Uint32Array(fnum * 3);
    for (let i = 0; i < fnum * 3; i++) idx[i] = face[fadr * 3 + i];

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setIndex(new THREE.BufferAttribute(idx, 1));
    geo.computeVertexNormals();
    meshGeoCache.set(mi, geo);
    return geo;
  }

  const robot = new THREE.Group();
  scene.add(robot);
  const rgba = model.geom_rgba as Float32Array; // 4*ngeom
  // 每帧同步图形：绑定未暴露 geom_xquat，用 body 世界位姿 ∘ geom 静态偏移组合
  // 预烘焙每个 geom 相对其 body 的局部位姿，运行时只做一次四元数乘法
  const bodyXpos = data.xpos as Float64Array; // 3*nbody 实时视图
  const bodyXquat = data.xquat as Float64Array; // 4*nbody (w,x,y,z)
  const geomBindings: { mesh: THREE.Mesh; body: number; gp: THREE.Vector3; gq: THREE.Quaternion }[] = [];
  for (let g = 0; g < model.ngeom; g++) {
    const type = model.geom_type[g];
    if (model.geom_group[g] === 3) continue; // group 3 = 调试碰撞盒（查看器惯例默认隐藏）
    let geo: THREE.BufferGeometry;
    if (type === 7) {
      geo = meshGeometry(model.geom_dataid[g]); // mjGEOM_MESH
    } else if (type === 6) {
      const s = model.geom_size as Float32Array; // box 半尺寸 ×3 → 演示样件等 box 体
      geo = new THREE.BoxGeometry(s[3 * g] * 2, s[3 * g + 1] * 2, s[3 * g + 2] * 2);
    } else if (type === 0) {
      const s = model.geom_size as Float32Array; // plane（工作台面）
      geo = new THREE.PlaneGeometry(s[3 * g] * 2, s[3 * g + 1] * 2);
    } else continue;
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[4 * g], rgba[4 * g + 1], rgba[4 * g + 2]),
      metalness: 0.35,
      roughness: 0.55,
      side: THREE.DoubleSide, // plane 需要双面
    });
    const mesh = new THREE.Mesh(geo, mat);
    robot.add(mesh);
    geomBindings.push({
      mesh,
      body: model.geom_bodyid[g] as number,
      gp: new THREE.Vector3(model.geom_pos[3 * g], model.geom_pos[3 * g + 1], model.geom_pos[3 * g + 2]),
      gq: new THREE.Quaternion(
        model.geom_quat[4 * g + 1], model.geom_quat[4 * g + 2],
        model.geom_quat[4 * g + 3], model.geom_quat[4 * g],
      ),
    });
  }
  console.log(`[MJ] 渲染 ${geomBindings.length}/${model.ngeom} 个 geom`);
  const Qb = new THREE.Quaternion();
  const V = new THREE.Vector3();
  function syncGraphics(): void {
    for (const { mesh, body, gp, gq } of geomBindings) {
      const bp = 3 * body, bq = 4 * body;
      Qb.set(bodyXquat[bq + 1], bodyXquat[bq + 2], bodyXquat[bq + 3], bodyXquat[bq]);
      mesh.quaternion.copy(Qb).multiply(gq);
      V.copy(gp).applyQuaternion(Qb);
      mesh.position.set(bodyXpos[bp] + V.x, bodyXpos[bp + 1] + V.y, bodyXpos[bp + 2] + V.z);
    }
  }
  syncGraphics();

  // ===== TCP 拖拽（绿球=目标点，臂用 CTC 追它——与 Unity 的 MJ_Target 同一语义）=====
  const ik = createTcpIk(mujoco, model, data);
  const tcpSite: number = ik.siteId; // tcp-ik 里已做过 mj_name2id + 校验
  const siteXpos = data.site_xpos as Float64Array;
  const readTcp = (v: THREE.Vector3): THREE.Vector3 =>
    v.set(siteXpos[3 * tcpSite], siteXpos[3 * tcpSite + 1], siteXpos[3 * tcpSite + 2]);
  const TCP_TMP = new THREE.Vector3(); // tick 里复用，避免每帧分配
  const ikTarget = readTcp(new THREE.Vector3());

  // ===== 关节控制面板（P1）=====
  // 目标角三个来源：任务（演示/回放写 demo.jointTarget）/ TCP 拖拽（IK 每帧写 targetQ）/ 手动（滑条），互斥。
  const DEG = 180 / Math.PI;
  const JOINT_NAMES = ['J1 · 底座回转', 'J2 · 肩', 'J3 · 肘', 'J4 · 腕俯仰', 'J5 · 腕滚转', 'J6 · 法兰'];
  const jointSpecs = JOINT_NAMES.map((label, i) => ({
    label,
    minDeg: +(model.jnt_range[i * 2] * DEG).toFixed(1),
    maxDeg: +(model.jnt_range[i * 2 + 1] * DEG).toFixed(1),
  }));
  const gripOpenRad: number = model.jnt_range[6 * 2 + 1]; // 齿轮最大角（≈343°）
  const GRIP_MM_PER_RAD = 8; // 小齿轮半径 0.008m → 1 rad = 8mm 单指行程
  let manualMode = false;
  const manualTargets = new Float64Array(6); // 手动模式的目标角（rad）
  let gripTargetRad = 0; // 夹爪目标角（滑条 mm / 开合按钮共同写这里）
  const panel = createJointPanel(document.getElementById('jpanel') as HTMLElement, jointSpecs, gripOpenRad * GRIP_MM_PER_RAD, {
    onManual(joint: number, deg: number): void {
      if (dragging) endDrag(); // 拖球中另一输入源碰滑条：先结束拖拽再切模式
      // 手动操控优先：停止演示/回放（录制不冲突——只在拖球时采样）
      if (demo.running) { demo.stop(); panel.setDemoRunning(false); }
      if (demo.replayRunning) demo.stopReplay();
      if (!manualMode) {
        // 首次切入手动：先从当前 IK 解播种全部 6 轴，否则其余关节目标瞬间归零、整臂"立正"
        for (let i = 0; i < 6; i++) manualTargets[i] = ik.angles[i];
      }
      manualMode = true;
      manualTargets[joint] = deg / DEG;
      panel.setMode(true);
    },
    onGripValue(mm: number): void {
      gripTargetRad = mm / GRIP_MM_PER_RAD;
    },
    onGrip(open: boolean): void {
      gripTargetRad = open ? gripOpenRad : 0;
    },
    onReset(): void {
      if (dragging) endDrag();
      // 复位抢占一切任务：停演示/回放/录制并同步面板，避免组合态
      if (demo.running) { demo.stop(); panel.setDemoRunning(false); panel.setBusbarRunning(false); }
      if (demo.replayRunning) demo.stopReplay();
      if (demo.recRunning) { demo.stopRec(); panel.setRecRunning(false); }
      panel.setPathInfo('未记录', false);
      demo.resetCube(); // 样件放回出生点（复位=整场景回到初始）
      demo.resetBusbar(); // 母线放回极柱（同语义）
      manualMode = true;
      manualTargets.fill(0); // 复位语义：6 轴归零 + 夹爪闭合
      gripTargetRad = 0;
      panel.setGripOpen(false);
      panel.setMode(true);
    },
    onBadgeClick(): void {
      if (manualMode && !dragging) { manualMode = false; panel.setMode(false); }
    },
    onBallToggle(hidden: boolean): void {
      ballHidden = hidden;
    },
    onDemo(): void {
      if (demo.running) { demo.stop(); panel.setDemoRunning(false); panel.setBusbarRunning(false); return; }
      try {
        if (dragging) endDrag();
        demo.stopRec(); demo.stopReplay(); panel.setRecRunning(false);
        manualMode = false; panel.setMode(false);
        demo.start();
        panel.setDemoRunning(true);
        toast('一键夹取演示开始');
      } catch (e) {
        toast(e instanceof Error ? e.message : String(e));
      }
    },
    onBusbar(): void {
      if (demo.running) { demo.stop(); panel.setDemoRunning(false); panel.setBusbarRunning(false); return; }
      try {
        if (dragging) endDrag();
        demo.stopRec(); demo.stopReplay(); panel.setRecRunning(false);
        manualMode = false; panel.setMode(false);
        demo.startBusbar();
        panel.setBusbarRunning(true);
        toast('母线拆取演示开始');
      } catch (e) {
        toast(e instanceof Error ? e.message : String(e));
      }
    },
    onRec(): void {
      if (demo.recRunning) {
        demo.stopRec();
        panel.setRecRunning(false);
        panel.setPathInfo(`已记录 ${demo.recCount} 点`, demo.recCount >= 2);
        return;
      }
      if (demo.running) { demo.stop(); panel.setDemoRunning(false); }
      if (manualMode) { manualMode = false; panel.setMode(false); }
      demo.startRec();
      panel.setRecRunning(true);
      panel.setPathInfo('记录中：拖动绿球…', true);
    },
    onReplay(): void {
      if (demo.replayRunning) { demo.stopReplay(); return; }
      try {
        if (dragging) endDrag();
        if (demo.running) { demo.stop(); panel.setDemoRunning(false); }
        if (demo.recRunning) { demo.stopRec(); panel.setRecRunning(false); } // 停录制必须同步按钮
        if (manualMode) { manualMode = false; panel.setMode(false); }
        demo.startReplay();
        panel.setPathInfo('回放中…', true);
      } catch (e) {
        toast(e instanceof Error ? e.message : String(e));
      }
    },
    onClearPath(): void {
      if (demo.recRunning) { demo.stopRec(); panel.setRecRunning(false); }
      demo.clearPath();
      panel.setPathInfo('未记录', false);
      toast('已清除录制路径');
    },
    onSendReal(): void {
      const modal = document.getElementById('send-modal');
      modal?.classList.add('open');
      document.getElementById('sm-cancel')?.focus(); // 焦点进弹窗（Esc/Tab 语义正确）
    },
  });
  panel.setGripOpen(false);
  let ballHidden = false; // TCP 球组隐藏开关（观察夹爪时用）

  const BALL = 0x30ff90, BALL_ACTIVE = 0xff8c1a; // 绿（空闲）→ 橙（拖拽中）
  const ball = new THREE.Mesh(
    new THREE.SphereGeometry(0.012, 20, 14),
    new THREE.MeshBasicMaterial({ color: BALL }),
  );
  scene.add(ball);
  // 大球"球网"：跟随实际 TCP，展示臂末端在哪（对照参考站交互）
  const netGeo = new THREE.SphereGeometry(0.034, 12, 8);
  const netFill = new THREE.Mesh(netGeo, new THREE.MeshBasicMaterial({
    color: BALL, transparent: true, opacity: 0.06, depthWrite: false,
  }));
  const netWire = new THREE.Mesh(netGeo, new THREE.MeshBasicMaterial({
    color: BALL, wireframe: true, transparent: true, opacity: 0.35, depthWrite: false,
  }));
  scene.add(netFill, netWire);
  // 拖拽连线：小球(目标) → 大球(实际 TCP)，运动到位过程的可视化
  const linkGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const linkLine = new THREE.Line(linkGeo, new THREE.LineBasicMaterial({
    color: BALL_ACTIVE, transparent: true, opacity: 0.9,
  }));
  linkLine.visible = false;
  scene.add(linkLine);
  // 投放区标记：降落坪样式（半透明圆盘 + 亮环 + 中心点），水平放置于桌面
  const dropRing = new THREE.Group();
  const padMat = (opacity: number) => new THREE.MeshBasicMaterial({
    color: 0x2ee584, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false,
  });
  const padDisc = new THREE.Mesh(new THREE.CircleGeometry(0.036, 40), padMat(0.32));
  const padRing = new THREE.Mesh(new THREE.RingGeometry(0.030, 0.036, 40), padMat(1.0));
  const padDot = new THREE.Mesh(new THREE.CircleGeometry(0.006, 20), padMat(0.95));
  // 注意：本场景为 Z-up（MuJoCo 约定），Circle/Ring 默认 XY 平面即是水平面——切勿再按
  // three.js Y-up 习惯加 rotation.x=-90°（那会把标记立成竖直，俯视成一条线）
  padRing.position.z = 0.0008; // 微分层避免 z-fighting
  padDot.position.z = 0.0016;
  dropRing.add(padDisc, padRing, padDot);
  scene.add(dropRing);
  // 母线放置区（A11 阶段二）：黄色降落坪，圆心 (0.30,0.30) 半径 0.10（用户指定）
  const busRing = new THREE.Group();
  const busMat = (opacity: number) => new THREE.MeshBasicMaterial({
    color: 0xffd166, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false,
  });
  const busDisc = new THREE.Mesh(new THREE.CircleGeometry(0.10, 48), busMat(0.10));
  const busRingEdge = new THREE.Mesh(new THREE.RingGeometry(0.093, 0.100, 48), busMat(0.9));
  const busDot = new THREE.Mesh(new THREE.CircleGeometry(0.008, 20), busMat(0.95));
  busRingEdge.position.z = 0.0008;
  busDot.position.z = 0.0016;
  busRing.add(busDisc, busRingEdge, busDot);
  scene.add(busRing);
  // 录制路径可视化（橙色细线，随采样实时生长；预分配属性避免每帧重建）
  const PATH_MAX = 4000;
  let recDrawn = 0; // 已写入 pathPos 的点数（增量绘制的游标；路径清空时复位）
  const pathPos = new Float32Array(PATH_MAX * 3);
  const pathGeo = new THREE.BufferGeometry();
  pathGeo.setAttribute('position', new THREE.BufferAttribute(pathPos, 3));
  pathGeo.setDrawRange(0, 0);
  const pathLine = new THREE.Line(pathGeo, new THREE.LineBasicMaterial({
    color: 0xffb454, transparent: true, opacity: 0.55,
  }));
  pathLine.visible = false;
  scene.add(pathLine);

  // 圆顶形工作空间钳制（照抄参考站 clampToWorkspace，半径换成我们 767mm 臂展）
  const REACH = 0.72;
  const clipNum = (v: number, a: number, b: number) => Math.min(b, Math.max(a, v));
  function clampTarget(v: THREE.Vector3): void {
    v.z = clipNum(v.z, 0.02, REACH);
    const planarLimit = Math.max(0.03, REACH * Math.sqrt(Math.max(0, 1 - (v.z / REACH) ** 2)));
    const planar = Math.hypot(v.x, v.y);
    if (planar > planarLimit) {
      const s = planarLimit / planar;
      v.x *= s;
      v.y *= s;
    }
  }

  const ray = new THREE.Raycaster();
  const dragPlane = new THREE.Plane();
  const ndc = new THREE.Vector2();
  const hitPt = new THREE.Vector3();
  const camDir = new THREE.Vector3();
  let dragging = false;
  let dragPointerId: number | null = null;
  // 死区回位（参考站 settle 语义 + 拖拽中卡死兜底）：
  //  - 松手后 >4mm 且 1.2s 内没收敛 → 小球回到夹爪（与大球同心）
  //  - 拖拽中鼠标停住 >0.6s 且物理误差仍 >30mm → 同样回位（关节顶限位/不可达）
  const SETTLE_ERR = 0.004, RECALL_TIMEOUT = 1200, DRAG_STALL_MS = 600, DRAG_STALL_ERR = 0.03;
  const LINK_SHOW_ERR = 0.006; // 橙线显示阈值：两球距离 >6mm 才画
  let outOfReachSince = 0;
  let lastMouseMove = performance.now();
  const setNDC = (e: PointerEvent): void => {
    ndc.set((e.clientX / innerWidth) * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
  };

  // capture 阶段抢在 OrbitControls 之前：点到球就拖球（锁视角），点空白才转视角
  window.addEventListener('pointerdown', (e: PointerEvent) => {
    if (e.button !== 0 || e.target !== renderer.domElement || dragging || ballHidden) return;
    setNDC(e);
    ray.setFromCamera(ndc, camera);
    if (!ray.intersectObject(ball, false).length) return;
    dragging = true;
    dragPointerId = e.pointerId;
    controls.enabled = false;
    renderer.domElement.setPointerCapture(e.pointerId);
    // 参考站语义：抓球瞬间目标点吸附到夹爪末端，并把 IK 解重播种到物理姿态
    // （其源码 ikAngles = captureAngles()——否则残留 IK 解会造成初始跳变）
    readTcp(ikTarget);
    ik.resetToPhysics();
    outOfReachSince = 0;
    lastMouseMove = performance.now(); // 抓球算一次"有操作"，防陈旧时间戳立刻触发回位
    // 人工随时抢占：抓球停止演示/回放；录制不抢——录制本身就是拖球记录
    if (demo.running) { demo.stop(); panel.setDemoRunning(false); }
    if (demo.replayRunning) demo.stopReplay();
    if (manualMode) { manualMode = false; panel.setMode(false); } // 抓球 = IK 接管
    camera.getWorldDirection(camDir);
    dragPlane.setFromNormalAndCoplanarPoint(camDir, ikTarget); // 相机平行面拖拽
    e.stopPropagation();
  }, true);
  window.addEventListener('pointermove', (e: PointerEvent) => {
    // 自愈：拖拽中左键已被系统释放（如失焦吞掉 pointerup）→ 立刻结束，防球脱手跟鼠标走
    if (dragging && !(e.buttons & 1)) { endDrag(e); return; }
    setNDC(e);
    if (dragging) {
      lastMouseMove = performance.now();
      ray.setFromCamera(ndc, camera);
      if (ray.ray.intersectPlane(dragPlane, hitPt)) {
        ikTarget.copy(hitPt);
        clampTarget(ikTarget);
      }
    } else if (e.target === renderer.domElement) {
      ray.setFromCamera(ndc, camera);
      renderer.domElement.style.cursor = ray.intersectObject(ball, false).length ? 'grab' : '';
    }
  });
  const endDrag = (e?: PointerEvent): void => {
    // 多指触摸：只响应发起拖拽的那根手指（别的手指抬起不结束拖拽）
    if (e && dragPointerId !== null && e.pointerId !== dragPointerId) return;
    dragging = false;
    dragPointerId = null;
    controls.enabled = true;
  };
  window.addEventListener('pointerup', endDrag);
  window.addEventListener('pointercancel', endDrag);
  window.addEventListener('blur', () => endDrag()); // 拖拽中切窗口丢 pointerup 也不卡死

  // ===== 物理：计算力矩（bias 前馈 + PD 反馈）追踪 IK 目标 + 固定步长循环 =====
  // 与桌面端 rebot_control.py 的 CTC 同思路：ctrl = bias + kp·e − kd·q̇。
  // 逐关节增益照抄参考站（源自其真机同步 mujoco_sync.py）：J2/J3 100/10 远硬于我此前的
  // 统一 40/8，是"丝滑跟手"的主因；我们的 J3 damping 6.0（真机线缆阻力）更需要高增益。
  // 物理参数（XML 摩擦/阻尼标定值）不动，只调控制器与步长。
  const nu: number = model.nu;
  const bias = data.qfrc_bias as Float64Array;
  const ctrl = data.ctrl as Float64Array;
  const qpos = data.qpos as Float64Array;
  const qvel = data.qvel as Float64Array;
  const trnid = model.actuator_trnid as Int32Array; // nu×2，motor 关联的 dof id
  const dofOf: number[] = [];
  for (let a = 0; a < nu; a++) dofOf.push(trnid[2 * a]);
  const KP = [80, 100, 100, 35, 25, 18, 300]; // a0..a5 臂 + a6 夹爪
  const KD = [8, 10, 10, 4, 3, 2.5, 40];
  model.opt.timestep = 0.001; // 网页侧降到 1kHz（参考站同款，保证高增益稳定）；桌面 XML 不动
  const dt: number = model.opt.timestep;
  const targetQ = new Float64Array(nu); // 每执行器目标角：臂 0..5 = IK 解，夹爪 = 0（闭合）
  const errEl = document.getElementById('ikerr') as HTMLElement;
  const snapQ = new Float64Array(data.qpos.length); // HUD 影子 FK 快照（演示误差计算用）
  const HUDP = new THREE.Vector3();
  let demoWasRunning = false; // 演示结束沿检测：把绿球/IK 解收回夹爪（防 servo 追陈旧球位）
  let acc = 0;
  let last = performance.now();
  let lastUi = 0;
  let lastPanel = 0;

  // ===== P4：一键夹取演示 + 路径录制回放 =====
  const demo = createDemo({
    ik, data, model, mujoco, ikTarget,
    isDragging: () => dragging,
    setGripTarget: (rad: number) => {
      gripTargetRad = rad;
      panel.setGripOpen(rad > gripOpenRad / 2);
    },
    gripOpenRad,
  });
  dropRing.position.set(demo.dropPoint.x, demo.dropPoint.y, 0.002); // 投放环与演示共用同一点
  busRing.position.set(demo.busbarDropPoint.x, demo.busbarDropPoint.y, 0.002); // 母线放置黄圈

  function tick(now: number): void {
    requestAnimationFrame(tick);
    const frameDt = Math.min((now - last) / 1000, 0.05); // 切后台回来不追帧
    acc += frameDt;
    last = now;

    // 任务推进：演示/回放走关节插值通路（不经 IK——回放也离线解成关节轨迹）；录制走伺服通路
    demo.busbarIdleTick(); // 母线钉住（freejoint 无碰撞，不钉会坠落穿桌）——任何状态每帧执行
    if (demo.running || demo.replayRunning) {
      demo.tick(frameDt, now);
      for (let i = 0; i < 6; i++) targetQ[i] = demo.jointTarget[i];
    } else {
      if (demoWasRunning) { // 任务结束/被打断：目标点收回夹爪（否则 servo 追陈旧球位，臂会甩走）
        readTcp(ikTarget);
        ik.resetToPhysics();
      }
      if (demo.recRunning) demo.tick(frameDt, now);
      if (manualMode) {
        for (let i = 0; i < 6; i++) targetQ[i] = manualTargets[i];
        readTcp(ikTarget); // 绿球贴随夹爪——手动模式下没有活动目标，残留旧目标会误导（假连线+莫名回位）
      } else {
        ik.servo([ikTarget.x, ikTarget.y, ikTarget.z], frameDt);
        for (let i = 0; i < 6; i++) targetQ[i] = ik.angles[i];
      }
    }
    demoWasRunning = demo.running || demo.replayRunning;
    targetQ[6] = gripTargetRad;

    while (acc >= dt) {
      for (let a = 0; a < nu; a++) {
        const d = dofOf[a];
        ctrl[a] = bias[d] + KP[a] * (targetQ[a] - qpos[d]) - KD[a] * qvel[d];
      }
      mujoco.mj_step(model, data);
      acc -= dt;
    }
    // 大球跟实际 TCP、小球=目标；拖拽中球变橙 + 橙线连两端（到位过程可视化）
    const tcpNow = readTcp(TCP_TMP);
    netFill.position.copy(tcpNow);
    netWire.position.copy(tcpNow);
    ball.position.copy(ikTarget);
    const showTcp = !ballHidden && !demo.running; // 演示中隐藏球组（臂独立表演）
    ball.visible = netFill.visible = netWire.visible = showTcp;
    (ball.material as THREE.MeshBasicMaterial).color.set(dragging ? BALL_ACTIVE : BALL);
    const gap = tcpNow.distanceTo(ikTarget);
    linkLine.visible = showTcp && gap > LINK_SHOW_ERR;
    if (linkLine.visible) {
      const posAttr = linkGeo.attributes.position as THREE.BufferAttribute;
      posAttr.setXYZ(0, tcpNow.x, tcpNow.y, tcpNow.z);
      posAttr.setXYZ(1, ikTarget.x, ikTarget.y, ikTarget.z);
      posAttr.needsUpdate = true;
    }

    // 死区回位：物理侧误差是唯一真值（IK 解误差在关节顶限位/自碰撞时会骗人）
    // 回位时同步把 IK 解重播种到物理姿态——否则 CTC 目标角瞬间跳变，臂会"弹"一下
    // 演示/回放/录制运行中禁用回位（任务自己管理节奏，否则回位会和任务抢 ikTarget 打架）
    const taskBusy = demo.running || demo.replayRunning || demo.recRunning;
    const physErr = gap;
    if (taskBusy) {
      outOfReachSince = 0;
    } else if (dragging) {
      if (physErr > DRAG_STALL_ERR && now - lastMouseMove > DRAG_STALL_MS) {
        ikTarget.copy(tcpNow); // 鼠标停住仍追不上 → 不可达，球回夹爪
        ik.resetToPhysics();
        outOfReachSince = 0;
      }
    } else if (physErr > SETTLE_ERR) {
      if (!outOfReachSince) outOfReachSince = now;
      if (now - outOfReachSince > RECALL_TIMEOUT) {
        ikTarget.copy(tcpNow); // 参考站 settle：松手后超时未收敛 → 回位
        ik.resetToPhysics();
        outOfReachSince = 0;
      }
    } else {
      outOfReachSince = 0;
    }

    if (now - lastUi > 200) {
      // 演示中的误差 = 物理 TCP vs 关节插值目标的正运动学（影子 FK），而非陈旧的拖拽球
      let hudErr = physErr;
      if (demo.running) {
        snapQ.set(qpos);
        for (let i = 0; i < 6; i++) qpos[i] = demo.jointTarget[i];
        mujoco.mj_forward(model, data);
        hudErr = tcpNow.distanceTo(readTcp(HUDP));
        qpos.set(snapQ);
        mujoco.mj_forward(model, data);
      }
      // 手动模式没有 TCP 目标语义，显示"手动控制"而非误导性的 0.0mm
      errEl.textContent = manualMode ? '手动控制' : `TCP 误差 ${(hudErr * 1000).toFixed(1)} mm`;
      lastUi = now;
    }
    // 面板读数 10Hz；IK 模式下滑条跟随 IK 解（手动模式滑条就是输入，不回写）
    if (now - lastPanel > 100) {
      lastPanel = now;
      const curDeg: number[] = [];
      for (let i = 0; i < 6; i++) curDeg.push(qpos[i] * DEG);
      const travelMm = qpos[6] * 0.008 * 1000; // 齿轮角 × 小齿轮半径 0.008m → 单指行程 mm
      panel.syncReadouts(curDeg, travelMm);
      panel.setDemoRunning(demo.running && demo.planKind === 'cube'); // 演示自然结束（ret 完成）时复位按钮态
      panel.setBusbarRunning(demo.running && demo.planKind === 'busbar');
      panel.setRecRunning(demo.recRunning);
      // 录制路径线可视化（增量写入：只追加新采样点，不清不重写——录制千点时
      // 全量重写是每帧数千次浮点拷贝的纯浪费）
      const pts = demo.getPath();
      const n = Math.min(pts.length, PATH_MAX);
      pathLine.visible = n >= 2;
      if (n >= 2) {
        for (let i = recDrawn; i < n; i++) {
          pathPos[3 * i] = pts[i].x;
          pathPos[3 * i + 1] = pts[i].y;
          pathPos[3 * i + 2] = pts[i].z;
        }
        recDrawn = n;
        pathGeo.setDrawRange(0, n);
        (pathGeo.attributes.position as THREE.BufferAttribute).needsUpdate = true;
      } else {
        recDrawn = 0; // 路径已清空（clearPath/startRec）：下轮从头写
      }
      if (demo.running) panel.setPathInfo(`演示 · ${demo.label}`, true);
      else if (demo.recRunning) panel.setPathInfo(`记录中：${demo.recCount} 点`, true);
      else if (demo.replayRunning) panel.setPathInfo('回放中…', true);
      else if (pts.length >= 2) panel.setPathInfo(`已记录 ${pts.length} 点`, false);
      else panel.setPathInfo('—', false);
      if (!manualMode) {
        // 演示/回放中滑条跟关节插值目标（面板目标读数=轨迹目标），否则显示陈旧 IK 解
        const src = demo.running || demo.replayRunning ? demo.jointTarget : ik.angles;
        const deg: number[] = [];
        for (let i = 0; i < 6; i++) deg.push(src[i] * DEG);
        panel.syncSliders(deg);
      }
    }
    syncGraphics();
    controls.update();
    const dust = (window as any).__dust as THREE.Points | undefined;
    if (dust) dust.rotation.z += 0.00035;
    renderer.render(scene, camera);
  }

  setStatus(`运行中 · nq=${model.nq} · ngeom=${model.ngeom} · nu=${nu}`);
  Object.assign((window as any).__mj, { camera, ball, ikTarget, ik, tcpSite, demo }); // 调试句柄（控制台现场检查用）
  requestAnimationFrame(tick);

  // ===== 下发实体臂弹窗（UI 先行：真机链路 P5 接入后开放）=====
  const modal = document.getElementById('send-modal') as HTMLElement;
  const smCancel = document.getElementById('sm-cancel');
  const closeModal = (): void => {
    modal.classList.remove('open');
    if (smCancel) smCancel.focus();
  };
  document.getElementById('sm-cancel')?.addEventListener('click', closeModal);
  document.getElementById('sm-confirm')?.addEventListener('click', () => {
    closeModal();
    toast('真机下发链路将在 P5 接入后开放 · 当前为界面预览');
  });
  modal.addEventListener('click', (e) => { if (e.target === modal) closeModal(); });
  window.addEventListener('keydown', (e) => { if (e.key === 'Escape') closeModal(); });
}

main().catch((e: unknown) => {
  console.error(e);
  setStatus('加载失败：' + (e instanceof Error ? e.message : String(e)));
});
