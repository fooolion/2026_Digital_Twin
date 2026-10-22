# -*- coding: utf-8 -*-
"""官方 STEP(20260425) 装配结构解析 → 判定尺寸误差在 MuJoCo/URDF 侧还是 Unity 侧。

原理：装配里相邻零件的配合变换原点通常落在关节轴上。把 STEP 装配树每个部件的
世界放置解出来后：
  A. 与 Unity(Arm9.17 静止位, 同一姿态) 直接比坐标 → 判 Unity 是否忠实于官方 CAD；
  B. 用配对出的 6 个轴点算连杆长度(位姿无关)，与 URDF/MuJoCo 比 → 判 MuJoCo 侧误差。

用法: mujoco_sim/.rebot/Scripts/python.exe tools/step_assembly_joints.py
"""
import re
import sys
import numpy as np

sys.path.insert(0, 'tools')
from compare_models import URDF, MJCF, PREFAB, STEP_0425, parse_urdf, parse_prefab

S_ = r"(?:'[^']*'|\$)"                                 # 字符串参数（含 $ 占位）


def T_from_axes(pt, zdir, xdir):
    z = np.asarray(zdir, float)
    z = z / np.linalg.norm(z)
    if xdir is None:
        xdir = np.cross(np.array([1., 0, 0]) if abs(z[0]) < 0.9 else np.array([0., 0, 1]), z)
    x = np.asarray(xdir, float)
    x = x - z * x.dot(z)
    n = np.linalg.norm(x)
    if n < 1e-9:
        xdir = np.cross(np.array([1., 0, 0]) if abs(z[0]) < 0.9 else np.array([0., 0, 1]), z)
        x = np.asarray(xdir, float) - z * x.dot(z)
        n = np.linalg.norm(x)
    x = x / n
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = pt
    return T


def inv(T):
    R, p = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def parse_step(path):
    txt = open(path, encoding='utf-8', errors='ignore').read()
    pt = {int(i): np.array([float(v) for v in c.split(',')])
          for i, c in re.findall(rf"#(\d+)\s*=\s*CARTESIAN_POINT\({S_}\s*,\s*\(([^)]+)\)", txt)}
    dr = {int(i): np.array([float(v) for v in c.split(',')])
          for i, c in re.findall(rf"#(\d+)\s*=\s*DIRECTION\({S_}\s*,\s*\(([^)]+)\)", txt)}
    ax = {}
    for m in re.finditer(rf"#(\d+)\s*=\s*AXIS2_PLACEMENT_3D\({S_}\s*,\s*#(\d+)(?:\s*,\s*#(\d+))?(?:\s*,\s*#(\d+))?\)", txt):
        i, p, z, x = m.groups()
        ax[int(i)] = T_from_axes(pt[int(p)], dr.get(int(z)) if z else None, dr.get(int(x)) if x else None)
    prod = {int(i): n for i, n in re.findall(rf"#(\d+)\s*=\s*PRODUCT\('([^']*)'", txt)}
    pdf = {int(i): int(p) for i, p in
           re.findall(rf"#(\d+)\s*=\s*PRODUCT_DEFINITION_FORMATION\({S_}\s*,\s*{S_}\s*,\s*#(\d+)\)", txt)}
    pd2prod = {}
    for i, fm in re.findall(rf"#(\d+)\s*=\s*PRODUCT_DEFINITION\({S_}\s*,\s*{S_}\s*,\s*#(\d+),\s*#\d+\)", txt):
        if int(fm) in pdf:
            pd2prod[int(i)] = prod.get(pdf[int(fm)], f'?pd{i}')
    nauo = {int(i): (int(a), int(b)) for i, a, b in re.findall(
        rf"#(\d+)\s*=\s*NEXT_ASSEMBLY_USAGE_OCCURRENCE\({S_}\s*,\s*{S_}\s*,\s*{S_}\s*,\s*#(\d+)\s*,\s*#(\d+)", txt)}
    pds2def = {int(i): int(d) for i, d in re.findall(
        rf"#(\d+)\s*=\s*PRODUCT_DEFINITION_SHAPE\({S_}\s*,\s*{S_}\s*,\s*#(\d+)\)", txt)}
    pd2sr = {}
    for i, d, sr in re.findall(
            rf"#(\d+)\s*=\s*SHAPE_DEFINITION_REPRESENTATION\((?:{S_}\s*,\s*{S_}\s*,)?\s*#(\d+)\s*,\s*#(\d+)\)", txt):
        d_ = pds2def.get(int(d))
        if d_ in pd2prod:
            pd2sr[d_] = int(sr)
    sr2pd = {v: k for k, v in pd2sr.items()}
    idt = {int(i): (int(a), int(b)) for i, a, b in re.findall(
        rf"#(\d+)\s*=\s*ITEM_DEFINED_TRANSFORMATION\({S_}\s*,\s*{S_}\s*,\s*#(\d+)\s*,\s*#(\d+)\)", txt)}
    rr = {}                                             # id -> (repA, repB, ax1, ax2)
    for m in re.finditer(r"#(\d+)\s*=\s*\(([^;]*?REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION.*?)\)\s*;", txt, re.S):
        i, block = int(m.group(1)), m.group(2)
        b = re.search(rf"REPRESENTATION_RELATIONSHIP\([^)]*?#(\d+)\s*,\s*#(\d+)\s*\)", block)
        w = re.search(rf"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*(?:ITEM_DEFINED_TRANSFORMATION"
                      rf"\({S_}\s*,\s*{S_}\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)|#(\d+))", block)
        if not (b and w):
            continue
        a1, a2, idref = w.groups()
        if idref is not None:
            a1, a2 = idt[int(idref)]
        rr[i] = (int(b.group(1)), int(b.group(2)), int(a1), int(a2))
    cdsr = [(int(r_), int(p_)) for _, r_, p_ in re.findall(
        rf"#(\d+)\s*=\s*CONTEXT_DEPENDENT_SHAPE_REPRESENTATION\(\s*#(\d+)\s*,\s*#(\d+)\)", txt)]
    return dict(ax=ax, pd2prod=pd2prod, nauo=nauo, sr2pd=sr2pd, rr=rr,
                cdsr=cdsr, pds2def=pds2def, is_mm='MILLI' in txt)


def build_edges(S, flip):
    edges = {}
    for rrid, pdsid in S['cdsr']:
        d = S['pds2def'].get(pdsid)
        if d not in S['nauo'] or rrid not in S['rr']:
            continue
        pp, cp = S['nauo'][d]
        ra, rb, a1, a2 = S['rr'][rrid]
        T = S['ax'][a1] @ inv(S['ax'][a2])
        pa, pb = S['sr2pd'].get(ra), S['sr2pd'].get(rb)
        if (pa, pb) == (cp, pp):                         # 约定 B：repA 属于子件
            T = inv(T)
        if flip:                                         # 约定探测用：整体取反
            T = inv(T)
        pn, cn = S['pd2prod'].get(pp, f'?pd{pp}'), S['pd2prod'].get(cp, f'?pd{cp}')
        edges.setdefault(pn, []).append((cn, T))
    return edges


def dfs(edges, top):
    sys.setrecursionlimit(50000)
    inst = {}

    def walk(name, Tw):
        inst.setdefault(name, []).append(Tw)
        for cn, Te in edges.get(name, []):
            walk(cn, Tw @ Te)

    walk(top, np.eye(4))
    return inst


def run(S, flip):
    edges = build_edges(S, flip)
    children = {c for lst in edges.values() for c, _ in lst}
    tops = [p for p in edges if p not in children]
    if not tops:
        tops = [max(edges, key=lambda k: len(edges[k]))]
    return dfs(edges, tops[0]), tops[0]


def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    S = parse_step(STEP_0425)
    print(f"== 1. STEP 装配解析 ==  单位: {'mm' if S['is_mm'] else 'm'}"
          f"  实例边: {len(S['cdsr'])}  变换: {len(S['rr'])}  部件名: {len(S['pd2prod'])}")
    scale = 0.001 if S['is_mm'] else 1.0
    _, linkT, _ = parse_urdf(URDF)
    uj = np.array([linkT[f'link{i}'][:3, 3] for i in range(1, 7)])
    pj = parse_prefab(PREFAB)
    uj2 = np.array([pj[f'Joint_{i}'] for i in range(1, 7)])

    best = None
    for flip in (False, True):
        inst, top = run(S, flip)
        origins = np.array([(T[0, 3], T[2, 3], -T[1, 3]) for Ts in inst.values()
                            for T in Ts[:1]]) * scale      # CAD(x,y,z) → Unity(x,z,-y)
        uni_max = max(float(np.linalg.norm(origins - p, axis=1).min()) * 1000 for p in uj2)
        print(f'  变换约定{"B(取反)" if flip else "A(标准)"}: 顶层 {top}, 部件 {len(inst)},'
              f' Unity 关节最近装配原点最大 {uni_max:.1f} mm')
        if best is None or uni_max < best[0]:
            best = (uni_max, flip, inst, top)
    uni_max, flip, inst, top = best

    print(f'\n== 2. 部件世界原点（约定{"B" if flip else "A"}，Unity 系 mm，按 Z 降序，只列每种首个实例）==')
    rows = sorted(((T[2, 3] / scale, T[0, 3] / scale, T[1, 3] / scale, n, len(Ts))
                   for n, Ts in inst.items() for T in Ts[:1]), reverse=True)
    for z, x, y, n, c in rows:
        print(f'  y={z:+8.1f}  x={x:+8.1f} z={y:+8.1f}  {n} ×{c}')

    names = [n for n, Ts in inst.items() for _ in Ts[:1]]
    origins = np.array([(T[0, 3], T[2, 3], -T[1, 3]) for Ts in inst.values()
                        for T in Ts[:1]]) * scale
    print('\n== 3. Unity 关节 ↔ STEP 部件原点配对（同一静止位，直接比）==')
    step_chain = []
    for i, p in enumerate(uj2, 1):
        d = np.linalg.norm(origins - p, axis=1)
        k = int(d.argmin())
        step_chain.append(origins[k])
        print(f'  Unity J{i} {np.round(p, 4)} ← 「{names[k]}」 距离 {d[k]*1000:.1f} mm')

    sc = np.array(step_chain)                            # Unity 系(米)
    d_step = np.linalg.norm(np.diff(sc, axis=0), axis=1) * 1000
    d_urdf = np.linalg.norm(np.diff(uj, axis=0), axis=1) * 1000
    d_uni = np.linalg.norm(np.diff(uj2, axis=0), axis=1) * 1000
    mjc = [re.search(rf'<body name="link{i}" pos="([^"]+)"', open(MJCF, encoding='utf-8').read()).group(1)
           for i in range(1, 7)]
    d_mj = np.linalg.norm(np.diff(np.array([[float(v) for v in t.split()] for t in mjc]),
                                  axis=0), axis=1) * 1000
    print('\n== 4. 连杆长度 mm（位姿无关，判误差侧）==')
    print('  段        STEP官方   URDF   MuJoCo   Unity   URDF-STEP  MuJoCo-STEP  Unity-STEP')
    for i in range(5):
        print(f'  J{i+1}->J{i+2}   {d_step[i]:7.1f}  {d_urdf[i]:6.1f}  {d_mj[i]:6.1f}  {d_uni[i]:6.1f}'
              f'   {d_urdf[i]-d_step[i]:+8.1f}   {d_mj[i]-d_step[i]:+8.1f}     {d_uni[i]-d_step[i]:+8.1f}')
    e_u, e_m = float(np.abs(d_uni - d_step).max()), float(np.abs(d_mj - d_step).max())
    print(f'\n  Unity↔STEP 直接坐标最大偏差: {uni_max:.1f} mm → Unity 侧{"忠实" if uni_max < 3 else "有偏差"}')
    print(f'  连杆长度最大偏差: URDF/MuJoCo {e_m:.1f} mm vs Unity {e_u:.1f} mm →'
          f' 误差在 {"MuJoCo/URDF 侧" if e_m > e_u else "Unity 侧"}')


if __name__ == '__main__':
    main()
