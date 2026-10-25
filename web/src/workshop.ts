/**
 * 零件库（R5，2026-10-25）：老师工作流第①步——导入电池零件。
 * 本地文件解析（不上传）：STL / GLB / GLTF / OBJ → three.js 预览 + 列表管理。
 * 「投放到仿真工作台」为预留接口：场景尺寸定稿后接通 sim 场景放置位。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { mountNav, toast } from './nav';
import './theme.css';
import './parts.css';

mountNav('workshop');

/* ================= 零件数据 ================= */

interface Part {
  id: string;
  name: string;
  fmt: string;
  tris: number;
  build: () => THREE.Object3D;   // 惰性构建（导入件缓存解析结果）
}

const parts: Part[] = [];
let selectedId: string | null = null;

/* ================= 预览视口 ================= */

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
  modelGroup.clear();
  modelGroup.add(obj);
  const box = new THREE.Box3().setFromObject(obj);
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

/* ================= 内置零件 ================= */

function builtinCube(): THREE.Object3D {
  const g = new THREE.BoxGeometry(0.04, 0.04, 0.04);
  const m = new THREE.MeshStandardMaterial({ color: 0x2ee584, roughness: 0.35, metalness: 0.15, transparent: true, opacity: 0.92 });
  const mesh = new THREE.Mesh(g, m);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({ color: 0x7df0bd, transparent: true, opacity: 0.6 }));
  mesh.add(edges);
  return mesh;
}
parts.push({ id: 'builtin-cube', name: '占位电池样件', fmt: 'BUILT-IN', tris: 12, build: builtinCube });

/* ================= 列表渲染 ================= */

const listEl = document.getElementById('part-list')!;
const metaEl = document.getElementById('part-meta')!;

function renderList(): void {
  listEl.innerHTML = parts.map((p) => `
    <li class="part-item${p.id === selectedId ? ' active' : ''}" data-id="${p.id}" role="button" tabindex="0">
      <span class="p-fmt">${p.fmt}</span>
      <div class="p-info"><b>${p.name}</b><small>${p.tris.toLocaleString()} 三角形</small></div>
    </li>`).join('');
  const sel = parts.find((p) => p.id === selectedId);
  metaEl.textContent = sel ? `${sel.name} · ${sel.fmt} · ${sel.tris.toLocaleString()} 三角形` : '';
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

/* ================= 本地导入（STL/GLB/GLTF/OBJ） ================= */

const fileInput = document.getElementById('import-file') as HTMLInputElement;
document.getElementById('import-btn')!.addEventListener('click', () => fileInput.click());

fileInput.addEventListener('change', async () => {
  const files = [...(fileInput.files ?? [])];
  fileInput.value = '';
  for (const f of files) {
    try {
      const obj = await parseFile(f);
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
});

async function parseFile(f: File): Promise<THREE.Object3D> {
  const buf = await f.arrayBuffer();
  const ext = f.name.split('.').pop()!.toLowerCase();
  if (ext === 'stl') {
    const { STLLoader } = await import('three/examples/jsm/loaders/STLLoader.js');
    const geo = new STLLoader().parse(buf);
    geo.computeVertexNormals();
    normalizeUnits(geo);
    return new THREE.Mesh(geo, importMaterial());
  }
  if (ext === 'obj') {
    const { OBJLoader } = await import('three/examples/jsm/loaders/OBJLoader.js');
    const obj = new OBJLoader().parse(new TextDecoder().decode(buf));
    obj.traverse((o) => { if ((o as THREE.Mesh).isMesh) normalizeUnits((o as THREE.Mesh).geometry as THREE.BufferGeometry); });
    return obj;
  }
  if (ext === 'glb' || ext === 'gltf') {
    const { GLTFLoader } = await import('three/examples/jsm/loaders/GLTFLoader.js');
    const gltf = await new Promise<{ scene: THREE.Object3D }>((res, rej) =>
      new GLTFLoader().parse(buf, '', (g) => res(g as unknown as { scene: THREE.Object3D }), rej));
    gltf.scene.traverse((o) => { if ((o as THREE.Mesh).isMesh) normalizeUnits((o as THREE.Mesh).geometry as THREE.BufferGeometry); });
    return gltf.scene;
  }
  throw new Error('不支持的格式（STEP 请先导出 GLB/STL）');
}

function importMaterial(): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color: 0x9fd8ff, roughness: 0.45, metalness: 0.25, transparent: true, opacity: 0.95 });
}

/** CAD 导出常见毫米单位（包围盒 >10 即视为 mm → 缩放为 m）；已按米的模型不动 */
function normalizeUnits(geo: THREE.BufferGeometry): void {
  geo.computeBoundingBox();
  const s = geo.boundingBox!.getSize(new THREE.Vector3());
  const maxDim = Math.max(s.x, s.y, s.z);
  if (maxDim > 10) geo.scale(0.001, 0.001, 0.001);
}

function countTris(obj: THREE.Object3D): number {
  let n = 0;
  obj.traverse((o) => {
    const m = o as THREE.Mesh;
    if (m.isMesh && m.geometry?.attributes?.position) n += m.geometry.attributes.position.count / 3;
  });
  return Math.round(n);
}

/* ================= 预留：投放到仿真场景 ================= */

document.getElementById('apply-btn')!.addEventListener('click', () => {
  const sel = parts.find((p) => p.id === selectedId);
  if (!sel) { toast('请先在列表中选择零件'); return; }
  toast(`已记录「${sel.name}」待投放 · 仿真场景放置位随场景定稿开放（挂账 P4-场景融合）`);
});

/* ================= 初始 ================= */

select('builtin-cube');
