# -*- coding: utf-8 -*-
"""
把官方 DM 视觉网格减面为总览页低模 → web/public/models/visual_lo/
分级目标：主体件 ≤5000 三角，硬件件(hardware)≤1200 三角，小件原样。
用法: mujoco_sim/.rebot/Scripts/python.exe tools/make_visual_lo.py
"""
import os
import io
import sys

import numpy as np
import trimesh

SRC = r"D:\Study\Zju\Competition\2026_Digital_Twin\reBot-DevArm\Rebot_Arm_description\DM\meshes\visual"
DST = r"D:\Study\Zju\Competition\2026_Digital_Twin\web\public\models\visual_lo"
MAIN_FACES = 5000     # 主体外形件目标面数
HW_FACES = 1200       # hardware_black 螺丝/紧固件（视觉上就是小凸点）
TOL = 0.95            # 已低于目标*0.95 的件跳过（避免重复抖动减面）

os.makedirs(DST, exist_ok=True)
total_src = total_dst = 0
report = []

for name in sorted(os.listdir(SRC)):
    if not name.lower().endswith(".stl"):
        continue
    path = os.path.join(SRC, name)
    mesh = trimesh.load(path, force="mesh", process=False)
    n0 = len(mesh.faces)
    target = HW_FACES if "hardware" in name else MAIN_FACES
    if n0 > target * 1.0 / TOL:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=target)
        except Exception:  # noqa: BLE001
            # 某些 trimesh 版本无 fast-simplification 后端，退回 open3d 风格显式调用
            import fast_simplification
            v, f = fast_simplification.simplify(
                mesh.vertices.astype(np.float64), mesh.faces.astype(np.int64),
                target_count=target)
            mesh = trimesh.Trimesh(vertices=v, faces=f, process=False)
        if len(mesh.faces) > target * 1.3:
            try:
                import fast_simplification
                v, f = fast_simplification.simplify(
                    mesh.vertices.astype(np.float64), mesh.faces.astype(np.int64),
                    target_count=target)
                mesh = trimesh.Trimesh(vertices=v, faces=f, process=False)
            except Exception as e:  # noqa: BLE001
                print(f"  ! {name}: 二次减面失败 {e}，保留 {len(mesh.faces)} 面")
    out = os.path.join(DST, name)
    mesh.export(out)
    s0, s1 = os.path.getsize(path), os.path.getsize(out)
    total_src += s0
    total_dst += s1
    report.append((name, n0, len(mesh.faces), s0, s1))

for name, n0, n1, s0, s1 in report:
    print(f"{name:52s} {n0:7d}->{n1:6d} faces  {s0//1024:5d}->{s1//1024:4d} KB")
print(f"TOTAL {total_src//1024} KB -> {total_dst//1024} KB ({total_dst*100//total_src}%)")
