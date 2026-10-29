/**
 * 零件库（老师工作流第①步：导入电池零件）。
 * 布局：左零件列表/导入卡/应用卡 + 右 three.js 预览（自转、居中落地、自适应取景）。
 * 导入：STL/GLB/GLTF/OBJ 本地解析不上传；毫米单位自动识别；「投放仿真」为预留接口。
 * Rev: R5 · 2026-10-25
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { mountBackdrop } from './backdrop';
import { mountNav, toast } from './nav';
import { loadBattery } from './battery';
import './theme.css';
import './parts.css';

mountBackdrop();
mountNav('workshop');

/* ===== 零件数据 ===== */

interface Part {
  id: string;
  name: string;
  fmt: string;
  tris: number;
  meta?: string;                 // 列表显示覆盖：内置件用人话描述，导入件默认显示三角形数
  build: () => THREE.Object3D;   // 惰性构建（导入件缓存解析结果）
}

const parts: Part[] = [];
let selectedId: string | null = null;

/* ===== 预览视口 ===== */

const host = document.getElementById('part-3d')!;
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
host.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.001, 100);
camera.up.set(0, 0, 1);
camera.position.set(0.3, -0.38, 0.22);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.03);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.enablePan = false;

scene.add(new THREE.AmbientLight(0xbfd4ff, 0.6));
const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(1, -2, 2.5); scene.add(key);
const rim = new THREE.DirectionalLight(0x4da3ff, 0.5); rim.position.set(-2, 1.5, 1); scene.add(rim);

const grid = new THREE.GridHelper(0.5, 20, 0x1f4a78, 0x122a44);
grid.rotation.x = Math.PI / 2; // Z-up 放平
scene.add(grid);

const modelGroup = new THREE.Group(); // 当前预览件（自动居中 + 落地）
const spinGroup = new THREE.Group();  // 展示自转
spinGroup.add(modelGroup);
scene.add(spinGroup);

function resize(): void {
  const w = host.clientWidth || 1, h = host.clientHeight || 1;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
new ResizeObserver(resize).observe(host); // 滚动条出现压缩容器宽时 window resize 不触发
resize();

const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
const t0 = performance.now();
renderer.setAnimationLoop(() => {
  if (!reduced) spinGroup.rotation.z = ((performance.now() - t0) / 1000) * 0.35;
  controls.update();
  renderer.render(scene, camera);
});

/** 模型入预览：XY 居中到原点、底面贴地（z=0）、按包围盒自适应相机 */
function preview(obj: THREE.Object3D): void {
  // 关键：先量再加——setFromObject 会乘父级上帧 matrixWorld（spinGroup 在自转），
  // 挂载后测得的中心是已旋转世界系坐标，当局部偏移用会让导入件偏心公转
  const box = new THREE.Box3().setFromObject(obj);
  modelGroup.clear();
  modelGroup.add(obj);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  obj.position.set(-c.x, -c.y, -box.min.z); // 中心 XY 到原点；最低点抬到 z=0
  const size = box.getSize(new THREE.Vector3());
  const radius = Math.max(size.length() * 0.5, 0.005);
  const dist = Math.max((radius / Math.sin(THREE.MathUtils.degToRad(camera.fov * 0.5))) * 1.5, 0.1);
  const dir = new THREE.Vector3(0.62, -0.72, 0.31).normalize();
  controls.target.set(0, 0, size.z * 0.4);
  camera.position.copy(controls.target).addScaledVector(dir, dist);
  controls.update();
}

/* ===== 内置零件 ===== */

// 构建一次模板，build() 只 clone（与导入件同模式，反复选中不泄漏 GPU 资源）
const builtinTemplate = (() => {
  const g = new THREE.BoxGeometry(0.04, 0.04, 0.04);
  const m = new THREE.MeshStandardMaterial({ color: 0x2ee584, roughness: 0.35, metalness: 0.15, transparent: true, opacity: 0.92 });
  const mesh = new THREE.Mesh(g, m);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({ color: 0x7df0bd, transparent: true, opacity: 0.6 }));
  mesh.add(edges);
  return mesh;
})();
parts.push({ id: 'builtin-cube', name: '标定立方', fmt: 'BUILT-IN', tris: 12, meta: '40mm 正方体', build: () => builtinTemplate.clone(true) });

// 真实电池模组（A11）：与 sim 场景共用同一套 STL 资产（统一模型坐标，加载即自动拼装）。
// 就绪后插到列表顶部；未就绪时列表里还没有它，不会出现"点了没反应"。
void loadBattery().then(({ group, tris }) => {
  group.quaternion.set(0.5, 0.5, 0.5, 0.5); // 循环换轴：模型 y（堆叠轴）立为竖直，否则预览侧放
  parts.unshift({
    id: 'battery-module', name: '方形铝壳电池模组', fmt: 'BUILT-IN', tris, meta: '220×190×180 mm · 真实模型',
    build: () => group.clone(), // 模板复用：clone 共享几何体/材质不重复上 GPU；preview 会改 position，模板本体不动
  });
  renderList();
}).catch((err) => console.error('电池模组加载失败', err));

/* ===== 列表渲染 ===== */

const listEl = document.getElementById('part-list')!;
const metaEl = document.getElementById('part-meta')!;

function renderList(): void {
  // 内置件显示人话描述（meta 覆盖），导入件显示三角形数（3D 重量参考）
  const sub = (p: Part) => p.meta ?? `${p.tris.toLocaleString()} 三角形`;
  listEl.innerHTML = parts.map((p) => `
    <li class="part-item${p.id === selectedId ? ' active' : ''}" data-id="${p.id}" role="button" tabindex="0">
      <span class="p-fmt"></span>
      <div class="p-info"><b></b><small>${sub(p)}</small></div>
    </li>`).join('');
  // 文件名来自本地导入，不可信进 innerHTML——用 DOM API 填充
  listEl.querySelectorAll('.part-item').forEach((liEl) => {
    const li = liEl as HTMLElement;
    const p = parts.find((x) => x.id === li.dataset.id)!;
    li.querySelector('.p-fmt')!.textContent = p.fmt;
    li.querySelector('.p-info b')!.textContent = p.name;
  });
  const sel = parts.find((p) => p.id === selectedId);
  metaEl.textContent = sel ? `${sel.name} · ${sel.fmt} · ${sub(sel)}` : '';
}

function select(id: string): void {
  const p = parts.find((x) => x.id === id);
  if (!p) return;
  selectedId = id;
  preview(p.build());
  renderList();
}

listEl.addEventListener('click', (e) => {
  const li = (e.target as HTMLElement).closest('.part-item') as HTMLElement | null;
  if (li) select(li.dataset.id!);
});
listEl.addEventListener('keydown', (e) => {
  const li = (e.target as HTMLElement).closest('.part-item') as HTMLElement | null;
  if (li && (e.key === 'Enter' || e.key === ' ')) { e.preventDefault(); select(li.dataset.id!); }
});

/* ===== 本地导入（STL/GLB/GLTF/OBJ） ===== */

const fileInput = document.getElementById('import-file') as HTMLInputElement;
document.getElementById('import-btn')!.addEventListener('click', () => fileInput.click());

fileInput.addEventListener('change', async () => {
  const files = [...(fileInput.files ?? [])];
  fileInput.value = '';
  // 同批文件建 blob URL 映射：.gltf 的外部 .bin/贴图、OBJ 的 MTL 都能就地解析
  const blobMap = new Map(files.map((f) => [f.name, URL.createObjectURL(f)]));
  for (const f of files) {
    if (f.size > 200 * 1024 * 1024) {
      toast(`跳过 ${f.name}：超过 200MB（主线程解析会冻结页面）`);
      continue;
    }
    try {
      const obj = await parseFile(f, blobMap);
      const tris = countTris(obj);
      const part: Part = {
        id: `imp-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`,
        name: f.name.replace(/\.[^.]+$/, ''),
        fmt: f.name.split('.').pop()!.toUpperCase(),
        tris,
        build: () => obj.clone(true),
      };
      parts.push(part);
      select(part.id);
      toast(`已导入 ${part.name}（${tris.toLocaleString()} 三角形）`);
    } catch (err) {
      console.error(err);
      toast(`导入失败：${f.name}（${String((err as Error)?.message ?? err).slice(0, 60)}）`);
    }
  }
  // 本批解析完毕（GLTFLoader resolve 时贴图已读入内存），释放 blob URL
  for (const url of blobMap.values()) URL.revokeObjectURL(url);
});

async function parseFile(f: File, blobMap: Map<string, string>): Promise<THREE.Object3D> {
  const buf = await f.arrayBuffer();
  const ext = f.name.split('.').pop()!.toLowerCase();
  if (ext === 'stl') {
    const { STLLoader } = await import('three/examples/jsm/loaders/STLLoader.js');
    const geo = new STLLoader().parse(buf);
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, importMaterial());
    normalizeUnits(mesh);
    return mesh;
  }
  if (ext === 'obj') {
    const { OBJLoader } = await import('three/examples/jsm/loaders/OBJLoader.js');
    const obj = new OBJLoader().parse(new TextDecoder().decode(buf));
    obj.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.isMesh) {
        // OBJ 无 MTL 时是白色 Phong——统一成 HUD 色板的导入材质，与 STL 预览一致
        m.material = importMaterial();
      }
    });
    normalizeUnits(obj);
    return obj;
  }
  if (ext === 'glb' || ext === 'gltf') {
    const { GLTFLoader } = await import('three/examples/jsm/loaders/GLTFLoader.js');
    // 外部资源（.bin/贴图）重写到同批文件的 blob URL（Blender "glTF Separate" 导出可导入）
    const mgr = new THREE.LoadingManager();
    mgr.setURLModifier((url) => {
      const base = url.split('/').pop()!.split('?')[0];
      return blobMap.get(base) ?? url;
    });
    const gltf = await new Promise<{ scene: THREE.Object3D }>((res, rej) =>
      new GLTFLoader(mgr).parse(buf, '', (g) => res(g as unknown as { scene: THREE.Object3D }), rej));
    normalizeUnits(gltf.scene);
    return gltf.scene;
  }
  throw new Error('不支持的格式（STEP 请先导出 GLB/STL）');
}

function importMaterial(): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color: 0x9fd8ff, roughness: 0.45, metalness: 0.25, transparent: true, opacity: 0.95 });
}

/** CAD 导出常见毫米单位：整树包围盒 >10 判为 mm → 只缩根节点（×0.001）。
 *  逐几何缩放是错的：Blender mm 场景导出 glTF 常见"几何 mm + 根 scale 0.001"，
 *  逐几何再缩会二次缩放（1e-6）导致模型不可见。 */
function normalizeUnits(obj: THREE.Object3D): void {
  obj.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(obj);
  if (box.isEmpty()) return;
  const s = box.getSize(new THREE.Vector3());
  if (Math.max(s.x, s.y, s.z) > 10) obj.scale.multiplyScalar(0.001);
}

function countTris(obj: THREE.Object3D): number {
  let n = 0;
  obj.traverse((o) => {
    const m = o as THREE.Mesh;
    if (m.isMesh && m.geometry?.attributes?.position) {
      const g = m.geometry as THREE.BufferGeometry;
      n += (g.index ? g.index.count : g.attributes.position.count) / 3; // GLB 带索引，按 index 数
    }
  });
  return Math.round(n);
}

/* ===== 预留：投放到仿真场景 ===== */

document.getElementById('apply-btn')!.addEventListener('click', () => {
  const sel = parts.find((p) => p.id === selectedId);
  if (!sel) { toast('请先在列表中选择零件'); return; }
  toast(`已记录「${sel.name}」待投放 · 仿真场景放置位随场景定稿开放（挂账 P4-场景融合）`);
});

/* ===== 初始 ===== */

select('builtin-cube');
