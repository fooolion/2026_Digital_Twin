/**
 * 电池模组共享加载器（A11）：sim 场景（XML 引用）、零件库、总览视口三处共用同一套 STL 资产。
 * 传输：优先构建期预压 .gz（约 800KB→300KB）+ 原生解压，失败回退原文件（与 sim.ts 同策略）。
 * 返回按统一模型坐标组装好的模板组——单一拥有者可直接改 transform，
 * 多处复用（零件库反复选中）请 clone（clone 共享几何体/材质，不重复上 GPU）。
 * Rev: A11 · 2026-10-28
 */
import * as THREE from 'three';

const BASE = `${import.meta.env.BASE_URL}models/meshes/battery`;
const FILES = ['battery-lower', 'battery-upper-1', 'battery-upper-2', 'battery-upper-3'] as const;

/** 与 sim.ts 同策略：优先 .gz + DecompressionStream，失败回退原文件 */
async function fetchBytes(url: string): Promise<ArrayBuffer> {
  try {
    const gz = await fetch(`${url}.gz`);
    if (gz.ok) {
      const stream = gz.body!.pipeThrough(new DecompressionStream('gzip'));
      return await new Response(stream).arrayBuffer();
    }
  } catch { /* 解压/网络失败回退原文件 */ }
  const r = await fetch(url);
  if (!r.ok) throw new Error(`HTTP ${r.status}: ${url}`);
  return r.arrayBuffer();
}

export interface BatteryModel {
  group: THREE.Group; // 模板组：统一模型坐标、无姿态（姿态由调用方设置）
  tris: number;       // 总三角形数（零件库列表展示用）
}

let cached: Promise<BatteryModel> | null = null;

/** 加载并组装电池模组（模块级缓存：同页多处调用只取一次） */
export function loadBattery(): Promise<BatteryModel> {
  cached ??= (async () => {
    const { STLLoader } = await import('three/examples/jsm/loaders/STLLoader.js');
    const geos = await Promise.all(FILES.map(async (f) => {
      const geo = new STLLoader().parse(await fetchBytes(`${BASE}/${f}.stl`));
      geo.scale(0.001, 0.001, 0.001); // 自家资产已知 mm 单位 → m（烘进几何体，相对位置随缩放保持）
      return geo;
    }));
    // STL 自带面法线（CAD 平直边缘），不做 computeVertexNormals 以免圆滑化
    const mats = [
      new THREE.MeshStandardMaterial({ color: 0x9aa2ab, roughness: 0.42, metalness: 0.55 }),
      new THREE.MeshStandardMaterial({ color: 0x4c5560, roughness: 0.5, metalness: 0.45 }),
      new THREE.MeshStandardMaterial({ color: 0x4c5560, roughness: 0.5, metalness: 0.45 }),
      new THREE.MeshStandardMaterial({ color: 0x4c5560, roughness: 0.5, metalness: 0.45 }),
    ];
    const group = new THREE.Group();
    geos.forEach((geo, i) => group.add(new THREE.Mesh(geo, mats[i])));
    const tris = geos.reduce((s, g) => s + Math.round(g.attributes.position.count / 3), 0);
    return { group, tris };
  })();
  return cached;
}
