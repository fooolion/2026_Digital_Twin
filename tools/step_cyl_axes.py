# -*- coding: utf-8 -*-
"""官方 STEP(20260425) 圆柱面(轴承孔/轴)同轴聚类 → 找 6 个关节轴。

判据：真实关节处，相邻两个零件各有一个同轴圆柱面(孔/轴)，所以「多个不同零件
共享同一根轴心线」的圆柱簇就是关节。这是与姿态、坐标系约定完全无关的刚体几何。

用法: mujoco_sim/.rebot/Scripts/python.exe tools/step_cyl_axes.py
"""
import re
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, 'tools')
from step_assembly_joints import parse_step, build_edges, dfs, STEP_0425


def parse_entities(path):
    txt = open(path, encoding='utf-8', errors='ignore').read()
    pt, dr, ax, cyl, refs = {}, {}, {}, {}, {}
    for pc in re.split(r'(?=#\d+\s*=)', txt):
        m = re.match(r'#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*?)\)\s*;', pc, re.S)
        if not m:
            continue
        i, name, body = int(m.group(1)), m.group(2), m.group(3)
        refs[i] = [int(x) for x in re.findall(r'#(\d+)', body)]
        if name == 'CYLINDRICAL_SURFACE':
            mm = re.search(r'#(\d+)\s*,\s*([-\d.eE+]+)\s*$', body.strip())
            if mm:
                cyl[i] = (int(mm.group(1)), float(mm.group(2)))
        elif name == 'CARTESIAN_POINT':
            mm = re.search(r'\(\s*([-\d.eE+,\s]+)\s*\)', body)
            if mm:
                pt[i] = np.array([float(v) for v in mm.group(1).split(',')])
        elif name == 'DIRECTION':
            mm = re.search(r'\(\s*([-\d.eE+,\s]+)\s*\)', body)
            if mm:
                dr[i] = np.array([float(v) for v in mm.group(1).split(',')])
        elif name == 'AXIS2_PLACEMENT_3D':
            ax[i] = re.findall(r'#(\d+)', body)
    return pt, dr, ax, cyl, refs, txt


def name2cylinders(S, refs, cyl, txt):
    """产品名 -> 该件几何里的圆柱 id 列表。
    路线：SRR/RR+SRR(无变换) 把同产品的各形状表示连成连通分量；分量里含定义级
    SR(已知产品) 的，其 ABSR 的 items 闭包里的圆柱都归该产品。"""
    # 1) 收集表示之间的成对关系
    pairs = [(int(a), int(b)) for a, b in
             re.findall(r"=\s*SHAPE_REPRESENTATION_RELATIONSHIP\((?:'[^']*'\s*,\s*'[^']*'\s*,)?\s*#(\d+)\s*,\s*#(\d+)\)", txt)]
    for m in re.finditer(r"#\d+\s*=\s*\(([^;]*?REPRESENTATION_RELATIONSHIP[^;]*?)\)\s*;", txt, re.S):
        blk = m.group(1)
        if 'WITH_TRANSFORMATION' in blk:
            continue
        b = re.search(r"REPRESENTATION_RELATIONSHIP\((?:'[^']*'\s*,\s*'[^']*'\s*,)?\s*#(\d+)\s*,\s*#(\d+)\)", blk)
        if b:
            pairs.append((int(b.group(1)), int(b.group(2))))
    # 2) 并查集
    par = {}

    def find(x):
        par.setdefault(x, x)
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x

    for a, b in pairs:
        par[find(a)] = find(b)
    # 3) 定义级 SR 所在分量 -> 产品名
    comp2name = {}
    for sr, pd in S['sr2pd'].items():
        nm = S['pd2prod'].get(pd)
        if nm:
            comp2name[find(sr)] = nm
    # 4) 所有形状表示实体(几何根) -> 闭包内圆柱 -> 归产品
    sr_ids = [int(i) for i in re.findall(r"#(\d+)\s*=\s*[A-Z0-9_]*SHAPE_REPRESENTATION\s*\(", txt)]
    out = defaultdict(list)
    visited = set()
    for sr in sr_ids:
        root = find(sr) if sr in par else None
        nm = comp2name.get(root)
        if nm is None:
            continue
        stack, seen = list(refs.get(sr, [])), set()
        while stack:
            e = stack.pop()
            if e in seen:
                continue
            seen.add(e)
            if e in cyl:
                out[nm].append(e)
            stack.extend(refs.get(e, []))
        visited |= seen
    return out


def world_axes(inst, n2c, cyl, pt, dr, ax, rmin=3.0, rmax=30.0):
    """返回 [(零件名, 半径, 轴上点, 单位方向), ...]，世界=CAD 装配系(mm)。"""
    rows = []
    for nm, ids in n2c.items():
        for T in inst.get(nm, []):
            T = np.asarray(T)
            for cid in ids:
                apl, r = cyl[cid]
                if not (rmin <= r <= rmax):
                    continue
                a = ax.get(apl)
                if not a:
                    continue
                p = pt.get(int(a[0]))
                z = dr.get(int(a[1])) if len(a) > 1 else None
                if p is None or z is None:
                    continue
                o = T[:3, :3] @ p + T[:3, 3]
                d = T[:3, :3] @ z
                n = np.linalg.norm(d)
                if n < 1e-9:
                    continue
                rows.append((nm, r, o, d / n))
    return rows


def cluster(rows):
    """按 (规范方向, 垂轴偏移 0.5mm 栅格) 聚类同轴圆柱。"""
    groups = defaultdict(list)
    for nm, r, o, d in rows:
        key = d.copy()
        k = np.argmax(np.abs(key) > 1e-6)
        if key[k] < 0:
            key = -key
        groups[tuple(np.round(key, 3))].append((nm, r, o, d))
    out = []
    for dk, g in groups.items():
        d = np.array(dk)
        d = d / np.linalg.norm(d)
        u = np.cross(d, [1, 0, 0] if abs(d[0]) < 0.9 else [0, 1, 0])
        u /= np.linalg.norm(u)
        v = np.cross(d, u)
        cells = defaultdict(list)
        for nm, r, o, _ in g:
            cells[(round(float(o @ u) / 0.5), round(float(o @ v) / 0.5))].append((nm, r, o))
        for cell, members in cells.items():
            names = {m[0] for m in members}
            os_ = np.array([m[2] for m in members])
            out.append((len(names), sorted({round(m[1], 1) for m in members}),
                        os_.mean(0), d, sorted(names)))
    out.sort(key=lambda t: -t[0])
    return out


def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    pt, dr, ax, cyl, refs, txt = parse_entities(STEP_0425)
    print(f'实体: 点 {len(pt)} 方向 {len(dr)} 轴系 {len(ax)} 圆柱 {len(cyl)}')
    S = parse_step(STEP_0425)
    n2c = name2cylinders(S, refs, cyl, txt)
    print(f'含圆柱的产品: {len(n2c)}')
    best = None
    for flip in (False, True):
        edges = build_edges(S, flip)
        children = {c for l in edges.values() for c, _ in l}
        top = [p for p in edges if p not in children][0]
        inst = dfs(edges, top)
        rows = world_axes(inst, n2c, cyl, pt, dr, ax)
        cl = [c for c in cluster(rows) if c[0] >= 2]
        print(f'约定{"B(取反)" if flip else "A(标准)"}: 圆柱轴 {len(rows)} 条, ≥2 件同轴簇 {len(cl)} 个,'
              f' 最大簇 {cl[0][0] if cl else 0} 件')
        if best is None or len(cl) > len(best[0]):
            best = (cl, flip, inst, top)
    cl, flip, inst, top = best
    print(f'\n== 采用约定{"B" if flip else "A"}（簇按不同零件数降序, 坐标=CAD装配系 mm）==')
    for n_, rs, o, d, names in cl[:45]:
        print(f'  {n_}件 r={rs} 点({o[0]:+8.1f},{o[1]:+8.1f},{o[2]:+8.1f})'
              f' 向({d[0]:+.2f},{d[1]:+.2f},{d[2]:+.2f})  {",".join(names[:4])}{"…" if len(names) > 4 else ""}')


if __name__ == '__main__':
    main()
