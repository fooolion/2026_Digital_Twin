# -*- coding: utf-8 -*-
"""三源模型几何一致性检测：官方 STEP ↔ DM URDF ↔ MuJoCo(MJCF) ↔ Unity Arm9.17.prefab。

检测内容
  A. 运动学链：6 个关节在零位/静止位下的世界坐标 —— URDF FK vs MuJoCo FK vs Unity 层级，
     用「刚体拟合 + 允许镜像拟合」的残差判断三套几何是否一致、是否互为镜像。
  B. 连杆长度：相邻关节间距（位姿无关的刚体尺寸），URDF vs Unity 逐段对比。
  C. 关节原点：MJCF 与 URDF 的关节原点逐项对照（MJCF 是照抄 URDF 的，应一致）。
  D. 整体包围盒：官方两个版本 STEP 的包围盒 vs MuJoCo 碰撞网格零位包围盒（粗校总体尺寸）。

用法（项目根目录）：
    mujoco_sim/.rebot/Scripts/python.exe tools/compare_models.py
"""
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

URDF = 'reBot-DevArm/Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf'
MJCF = 'mujoco_sim/models/ReBot_Arm_DM.xml'
PREFAB = 'MujocoUnityRenderer/Assets/Prefabs/Arm9.17.prefab'
MESH_DIR = 'reBot-DevArm/Rebot_Arm_description/DM/meshes'
STEP_0425 = 'reBot-DevArm/hardware/reBot_B601_DM/reBot_B601_DM_v1.1_20260425.step'
STEP_0625 = 'reBot-DevArm/hardware/reBot_B601_DM/reBot_B601_DM_v1.1_20260625.step'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


# ---------- 基础工具 ----------

def rpy_to_mat(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp,     cp * sr,          cp * cr]])


def compose(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def kabsch(P, Q, allow_reflect=False):
    """刚体拟合：找 R,t 使 Q ≈ R@P + t（P/Q 为 n×3 行向量点集）。返回 (残差RMS, R, t)。"""
    pm, qm = P.mean(0), Q.mean(0)
    H = (P - pm).T @ (Q - qm)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U @ Vt))
    D = np.diag([1.0, 1.0, d]) if not allow_reflect else np.diag([1.0, 1.0, np.sign(d) if d != 0 else 1.0])
    R = Vt.T @ D @ U.T
    t = qm - R @ pm
    resid = Q - (R @ P.T).T - t
    rms = float(np.sqrt((resid ** 2).sum(1).mean()))
    return rms, R, t


def stl_bbox(path):
    """读 STL（先按二进制解析，失败再按 ASCII），返回 (min, max) 包围盒。"""
    with open(path, 'rb') as f:
        raw = f.read()
    if len(raw) >= 84:                                    # 先试二进制：80头 + 4数量 + 50×n
        n = struct.unpack('<I', raw[80:84])[0]
        if n > 0 and 84 + 50 * n == len(raw):
            arr = np.frombuffer(raw[84:84 + 50 * n], dtype=np.uint8).reshape(n, 50)
            tri = arr[:, 12:48].copy().view('<f4').reshape(n * 3, 3)   # 跳过法线取 3 顶点
            if np.isfinite(tri).all() and np.abs(tri).max() < 1e4:
                return tri.min(0), tri.max(0)
    v = np.array([[float(x) for x in m.split()] for m in
                  re.findall(rb'vertex\s+([-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+)', raw)],
                 dtype=float)
    if len(v) == 0:
        raise ValueError(f'无法识别的 STL: {path}')
    return v.min(0), v.max(0)


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 + z1 * x2 + y1 * w2 - x1 * z2,
                     w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2])


def qrot(q, v):
    x, y, z, w = q
    t = 2.0 * np.cross([x, y, z], v)
    return v + w * np.array(t) + np.cross([x, y, z], t)


# ---------- 1. URDF（官方 DM 描述）----------

def parse_urdf(path):
    root = ET.parse(path).getroot()
    jinfo, jfrom_parent = {}, {}
    meshes = {}                                          # link -> [stl 相对路径]
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()])
        rpy = np.array([float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()])
        name = j.get('name')
        info = dict(parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                    xyz=xyz, R=rpy_to_mat(rpy), type=j.get('type'))
        jinfo[name] = info
        jfrom_parent[info['parent']] = (name, info)
    for link in root.findall('link'):
        lname = link.get('name')
        for col in link.findall('collision'):
            gm = col.find('geometry/mesh')
            if gm is not None:
                meshes.setdefault(lname, []).append(gm.get('filename'))
    # 沿链正向累加，得到零位下各 link 的世界变换
    T = np.eye(4)
    linkT = {'base_link': T.copy()}
    cur = 'base_link'
    while cur in jfrom_parent:
        name, info = jfrom_parent[cur]
        T = T @ compose(info['R'], info['xyz'])
        linkT[info['child']] = T.copy()
        cur = info['child']
    return root, linkT, meshes


# ---------- 2. Unity prefab 层级 ----------

def parse_prefab(path):
    txt = open(path, encoding='utf-8').read()
    blocks = re.findall(r'^--- !u!(\d+) &(\d+)\n(.*?)(?=^--- |\Z)', txt, re.M | re.S)
    go, tr = {}, {}
    for typ, fid, body in blocks:
        if typ == '1':
            nm = re.search(r'm_Name: (.*)', body)
            if nm:
                go[fid] = nm.group(1).strip()
        elif typ == '4':
            g = re.search(r'm_GameObject: \{fileID: (\d+)\}', body)
            f = re.search(r'm_Father: \{fileID: (\d+)\}', body)
            p3 = re.search(r'm_LocalPosition: \{x: ([\d.eE+-]+), y: ([\d.eE+-]+), z: ([\d.eE+-]+)\}', body)
            r4 = re.search(r'm_LocalRotation: \{x: ([\d.eE+-]+), y: ([\d.eE+-]+), z: ([\d.eE+-]+), w: ([\d.eE+-]+)\}', body)
            if g:
                tr[fid] = dict(g=g.group(1), f=f.group(1) if f else None,
                               p=np.array([float(x) for x in p3.groups()]) if p3 else np.zeros(3),
                               r=np.array([float(x) for x in r4.groups()]) if r4 else np.array([0, 0, 0, 1.0]))
    t2n = {t: go.get(d['g'], '?') for t, d in tr.items()}
    names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']

    def world_xform(t):
        """沿父链累积，返回该节点的 (世界坐标, 世界旋转四元数)。"""
        chain, cur = [], t
        while cur and cur in tr:
            chain.append(cur)
            cur = tr[cur]['f']
        chain.reverse()
        pos = np.zeros(3)
        q = np.array([0.0, 0.0, 0.0, 1.0])
        for c in chain:
            qn = tr[c]['r'] / np.linalg.norm(tr[c]['r'])
            pos = pos + qrot(q, tr[c]['p'])
            q = qmul(q, qn)
        return pos, q

    out = {}
    for n in names:
        for t, d in tr.items():
            if t2n.get(t) == n:
                out[n], _ = world_xform(t)
                break
    return out


# ---------- 3. STEP 包围盒 ----------

def step_bbox(path):
    """STEP 包围盒：只用 VERTEX_POINT 引用的点（真实拓扑顶点），
    避免被 B 样条控制点（可能离实体很远）污染。返回 (min, max, 顶点数, 总点数)。"""
    txt = open(path, encoding='utf-8', errors='ignore').read()
    pts = {}
    for m in re.finditer(r"#(\d+)\s*=\s*CARTESIAN_POINT\('[^']*',\s*\(([^)]+)\)\)", txt):
        pts[m.group(1)] = np.array([float(v) for v in m.group(2).split(',')])
    vids = set(re.findall(r"VERTEX_POINT\('[^']*',\s*#(\d+)\)", txt))
    v = np.array([pts[i] for i in vids if i in pts])
    return v.min(0), v.max(0), len(v), len(pts)


# ---------- 主流程 ----------

def main():
    # A. URDF 关节链
    urdf_root, linkT, urdf_meshes = parse_urdf(URDF)
    urdf_j = np.array([linkT[f'link{i}'][:3, 3] for i in range(1, 7)])
    print('== A1. DM URDF 关节零位世界坐标（Z 上系）==')
    for i, p in enumerate(urdf_j, 1):
        print(f'  J{i}: ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})')

    # B. MuJoCo 零位关节锚点（直接用 mujoco，验证 MJCF 与 URDF 一致）
    m = mujoco.MjModel.from_xml_path(MJCF)
    d = mujoco.MjData(m)
    d.qpos[:] = 0
    mujoco.mj_forward(m, d)
    mj_j = np.array([d.xanchor[m.joint(n).id] for n in JOINTS])
    print('\n== A2. MJCF 与 URDF 关节原点逐项对照（应一致）==')
    for i, n in enumerate(JOINTS, 1):
        o = re.search(rf'<body name="link{i}" pos="([^"]+)"', open(MJCF, encoding='utf-8').read())
        mj_pos = np.array([float(v) for v in o.group(1).split()])
        uo = None
        for j in urdf_root.findall('joint'):
            if j.get('name') == n:
                uo = np.array([float(v) for v in j.find('origin').get('xyz').split()])
        print(f'  {n}: MJCF={mj_pos}  URDF={uo}  差={np.abs(mj_pos-uo).max():.2e} m')

    # C. Unity 关节链
    uj_map = parse_prefab(PREFAB)
    uj = np.array([uj_map[f'Joint_{i}'] for i in range(1, 7)])
    print('\n== B. Unity Arm9.17 关节静止位世界坐标（Y 上系）==')
    for i, p in enumerate(uj, 1):
        print(f'  J{i}: ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})')

    # D. 连杆长度（位姿无关的刚体尺寸）
    print('\n== C. 连杆长度（相邻关节间距，米）URDF vs Unity ==')
    d_urdf = np.linalg.norm(np.diff(urdf_j, axis=0), axis=1)
    d_u = np.linalg.norm(np.diff(uj, axis=0), axis=1)
    for i, (a, b) in enumerate(zip(d_urdf, d_u), 1):
        print(f'  J{i}->J{i+1}: URDF={a:.4f}  Unity={b:.4f}  差={abs(a-b)*1000:.1f} mm')
    print(f'  J6->TCP : URDF/MuJoCo≈0.1554(gripper_base 面到 TCP 另计，Unity 无对应点，略)')

    # E. 刚体/镜像拟合：三套点集是否同一几何
    print('\n== D. 几何一致性拟合（残差 RMS，越接近 0 越一致）==')
    r_proper, _, _ = kabsch(uj, urdf_j)
    r_reflect, _, _ = kabsch(uj, urdf_j, allow_reflect=True)
    print(f'  Unity vs URDF : 刚体拟合 RMS={r_proper*1000:.1f} mm | 允许镜像 RMS={r_reflect*1000:.1f} mm'
          f'  => {"几何一致(含镜像)" if r_reflect*1000 < 5 else "几何不一致"}')
    r2, _, _ = kabsch(uj, mj_j)
    r2f, _, _ = kabsch(uj, mj_j, allow_reflect=True)
    print(f'  Unity vs MuJoCo: 刚体拟合 RMS={r2*1000:.1f} mm | 允许镜像 RMS={r2f*1000:.1f} mm')

    # F. STEP 包围盒（官方两版本）+ MuJoCo 碰撞网格包围盒
    print('\n== E. 整体包围盒(米) ==')
    for tag, path in [('官方 20260425 (实体臂)', STEP_0425), ('官方 20260625', STEP_0625)]:
        mn, mx, nv, nall = step_bbox(path)
        print(f'  {tag}: 尺寸 X={mx[0]-mn[0]:.3f} Y={mx[1]-mn[1]:.3f} Z={mx[2]-mn[2]:.3f}'
              f'  (范围 X[{mn[0]:.2f},{mx[0]:.2f}] Y[{mn[1]:.2f},{mx[1]:.2f}] Z[{mn[2]:.2f},{mx[2]:.2f}],'
              f' 顶点 {nv}/{nall})')
    # MuJoCo 零位碰撞网格包围盒（URDF 碰撞网格 = 同一套 STL，经 URDF FK 变换）
    allmin, allmax = np.full(3, np.inf), np.full(3, -np.inf)
    for link, T in linkT.items():
        for rel in urdf_meshes.get(link, []):
            fn = MESH_DIR + '/' + rel.replace('../meshes/', '')
            mn, mx = stl_bbox(fn)
            pts = (T[:3, :3] @ np.column_stack([mn, mx]).T).T + T[:3, 3]
            allmin = np.minimum(allmin, pts.min(0))
            allmax = np.maximum(allmax, pts.max(0))
    print(f'  URDF 碰撞网格零位包围盒: 尺寸 X={allmax[0]-allmin[0]:.3f} Y={allmax[1]-allmin[1]:.3f}'
          f' Z={allmax[2]-allmin[2]:.3f}  (Z 范围 [{allmin[2]:.3f},{allmax[2]:.3f}])')


if __name__ == '__main__':
    main()