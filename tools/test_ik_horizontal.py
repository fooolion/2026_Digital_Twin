# -*- coding: utf-8 -*-
"""solve_ik 两级优先（位置硬 + 倾角尽力而为）的验证脚本（手动跑，不进主链路）。

合同（2026-10-09 方案 v2，用户拍板）：位置高精度（≤0.1mm 级）任何情况下优先；
倾角在位置不动的前提下尽力压低（物理下限由腕部结构决定，孤立小片区内可达水平）。

  1) 姿态残差雅可比 FD 核对（上游 du/dq + 求解器实际用的 J3 两层）
  2) 热启动拖拽（真实用法）：任意方向拖动，断言位置全程 <1mm（一级目标），
     倾角尽力而为只报告；相邻步不跳变
  3) 冷启动（清空帧间状态）：位置 <1mm；可行片区目标倾角应能压到 <3°
  4) 随机目标 20 组：位置全部 <1mm（位置处处可达）；倾角尽力而为，
     只做合理性上界断言（<89°）

用法（仓库根目录）:
    PYTHONUTF8=1 mujoco_sim/.rebot/Scripts/python.exe tools/test_ik_horizontal.py
退出码 0 = 全部通过。
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)                                   # sim_bridge 的模型路径是仓库根相对
sys.path.insert(0, os.path.join(ROOT, "mujoco_sim"))

import numpy as np
import mujoco
from sim_bridge import solve_ik, MODEL

# 已验证的可行构型（随机采样最优，倾角 0.41°）
Q_STAR = np.radians([146.2, -4.6, -53.5, 58.6, -88.0, 80.1])


def set_arm(model, data, joint_idx, q):
    for j, v in zip(joint_idx, q):
        data.qpos[model.jnt_qposadr[j]] = float(v)
    mujoco.mj_forward(model, data)


def tilt_deg(data, sid):
    u = data.site_xmat[sid].reshape(3, 3)[:, 1]
    return float(np.degrees(np.arctan2(np.hypot(u[0], u[1]), abs(u[2]))))


def main():
    model = mujoco.MjModel.from_xml_path(MODEL)
    sid = model.site("tcp").id
    joint_idx = [model.joint(f"joint{i}").id for i in range(1, 7)]
    ik_data = mujoco.MjData(model)
    meas = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    failures = []

    # ---- 1) 姿态雅可比（上游 du/dq）：有限差分对照 ----
    lohi = [model.jnt_range[j] for j in joint_idx]
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    for trial in range(4):
        set_arm(model, ik_data, joint_idx, [rng.uniform(lo, hi) for lo, hi in lohi])
        mujoco.mj_jacSite(model, ik_data, jacp, jacr, sid)
        u0 = ik_data.site_xmat[sid].reshape(3, 3)[:, 1].copy()
        err_max = 0.0
        for j in joint_idx:
            dadr, qadr = model.jnt_dofadr[j], model.jnt_qposadr[j]
            ana = np.cross(jacr[:, dadr], u0)
            eps = 1e-7
            ik_data.qpos[qadr] += eps
            mujoco.mj_forward(model, ik_data)
            fd = (ik_data.site_xmat[sid].reshape(3, 3)[:, 1] - u0) / eps
            ik_data.qpos[qadr] -= eps
            err_max = max(err_max, float(np.abs(ana - fd).max()))
        if err_max > 1e-4:
            failures.append(f"雅可比核对 trial{trial}: |解析-差分|={err_max:.2e}")
    print("[1] 姿态雅可比 FD 核对完成")

    # ---- 1b) 残差雅可比 J3（求解器实际用的 2×6）解析 vs 差分 ----
    def e_res(qq):
        set_arm(model, ik_data, joint_idx, qq)
        u = ik_data.site_xmat[sid].reshape(3, 3)[:, 1]
        s = np.hypot(u[0], u[1])
        if s < 1e-12:
            return np.zeros(2)
        a = np.arctan2(s, abs(u[2]))
        yv = np.array([0.0, 0.0, 1.0 if u[2] >= 0 else -1.0])
        return a * (np.cross(u, yv) / s)[:2]

    def J3_formula(qq):                        # 与 solve_ik.build 完全相同的公式
        set_arm(model, ik_data, joint_idx, qq)
        mujoco.mj_jacSite(model, ik_data, jacp, jacr, sid)
        u = ik_data.site_xmat[sid].reshape(3, 3)[:, 1]
        s = np.hypot(u[0], u[1])
        yv = np.array([0.0, 0.0, 1.0 if u[2] >= 0 else -1.0])
        a = np.arctan2(s, abs(u[2]))
        ah = np.cross(u, yv) / s
        Jws = jacr[:, [model.jnt_dofadr[j] for j in joint_idx]]
        dw = np.cross(np.cross(Jws.T, u), yv)
        dA = (dw - np.outer(dw @ ah, ah)) / s
        return (np.outer(ah, -(Jws.T @ ah)) + a * dA.T)[:2]

    for trial in range(3):
        q = [rng.uniform(lo, hi) for lo, hi in lohi]
        J = J3_formula(q)
        Jfd = np.zeros((2, 6))
        eps = 1e-7
        for k in range(6):
            qp, qm = list(q), list(q)
            qp[k] += eps
            qm[k] -= eps
            Jfd[:, k] = (e_res(qp) - e_res(qm)) / (2 * eps)
        err_max = float(np.abs(J - Jfd).max())
        if err_max > 1e-5:
            failures.append(f"J3 残差雅可比核对 trial{trial}: |解析-差分|={err_max:.2e}")
    print("[1b] 残差雅可比 J3 FD 核对完成")

    # ---- 2) 热启动拖拽（真实用法）：位置全程 <1mm 是一级目标，倾角尽力而为 ----
    set_arm(model, meas, joint_idx, Q_STAR)
    tcp0 = meas.site_xpos[sid].copy()
    a1 = tcp0 + np.array([0.04, 0.03, -0.03])
    set_arm(model, ik_data, joint_idx, Q_STAR)     # 桥接的初始姿态
    max_jump = 0.0
    worst = (0.0, 0.0)
    for i in range(16):
        target = tcp0 + (a1 - tcp0) * i / 15
        q_prev = np.array([ik_data.qpos[model.jnt_qposadr[j]] for j in joint_idx])
        solve_ik(model, ik_data, target, joint_idx)
        meas.qpos[:] = ik_data.qpos[:]
        mujoco.mj_forward(model, meas)
        perr = float(np.linalg.norm(target - meas.site_xpos[sid]))
        t = tilt_deg(meas, sid)
        worst = (max(worst[0], perr), max(worst[1], t))
        max_jump = max(max_jump, float(np.abs(
            np.array([ik_data.qpos[model.jnt_qposadr[j]] for j in joint_idx]) - q_prev).max()))
        if perr > 1e-3:
            failures.append(f"热拖拽 step{i}: 位置 {perr*1000:.2f}mm > 1mm（一级目标失守）")
    print(f"[2] 热启动拖拽 15 步：位置最差 {worst[0]*1000:.3f}mm，最差倾角 {worst[1]:.2f}°（尽力而为），"
          f"最大单步跳变 {np.degrees(max_jump):.2f}°（报告值：合同允许为压倾角翻分支，伺服平滑）")

    # 静态稳定性：同一目标连续解 5 帧，解必须逐步一致（抓求解器抖动/振荡翻腕）
    static_jump = 0.0
    for _ in range(5):
        q_before = np.array([ik_data.qpos[model.jnt_qposadr[j]] for j in joint_idx])
        solve_ik(model, ik_data, target, joint_idx)
        q_after = np.array([ik_data.qpos[model.jnt_qposadr[j]] for j in joint_idx])
        static_jump = max(static_jump, float(np.abs(q_after - q_before).max()))
    print(f"[2b] 静态目标连续 5 帧：最大解漂移 {np.degrees(static_jump):.4f}°")
    if np.degrees(static_jump) > 0.01:
        failures.append(f"静态目标解漂移 {np.degrees(static_jump):.3f}°（应完全稳定）")

    # ---- 3) 真·冷启动（清空帧间状态）：位置 <1mm；可行片区目标倾角 <3° ----
    solve_ik.memo = None
    solve_ik.elites = []
    ik_data.qpos[:] = 0.0
    t0 = time.time()
    solve_ik(model, ik_data, tcp0, joint_idx)
    dt = time.time() - t0
    meas.qpos[:] = ik_data.qpos[:]
    mujoco.mj_forward(model, meas)
    perr = float(np.linalg.norm(tcp0 - meas.site_xpos[sid]))
    t = tilt_deg(meas, sid)
    print(f"[3] 零位冷启动到可行目标：位置 {perr*1000:.3f}mm，倾角 {t:.3f}°（耗时 {dt*1000:.0f}ms）")
    if perr > 1e-3 or t > 3.0:
        failures.append(f"冷启动: 位置 {perr*1000:.2f}mm 倾角 {t:.2f}°")

    # ---- 4) 随机目标 20 组：位置全部 <1mm（一级目标处处可达），倾角尽力而为 ----
    tilts = []
    for trial in range(20):
        target = np.array([rng.uniform(0.15, 0.45),
                           rng.uniform(-0.25, 0.25),
                           rng.uniform(0.05, 0.45)])
        ik_data.qpos[:] = 0.0
        solve_ik(model, ik_data, target, joint_idx)
        meas.qpos[:] = ik_data.qpos[:]
        mujoco.mj_forward(model, meas)
        perr = float(np.linalg.norm(target - meas.site_xpos[sid]))
        t = tilt_deg(meas, sid)
        tilts.append(t)
        if perr > 1e-3:
            failures.append(f"随机目标{trial} {target.round(3)}: 位置 {perr*1000:.2f}mm > 1mm")
        if t > 89.0:
            failures.append(f"随机目标{trial} {target.round(3)}: 倾角 {t:.2f}° > 89°（尽力而为失效）")
    tilts = np.array(tilts)
    print(f"[4] 随机目标 20 组：位置全部 <1mm，倾角 中位 {np.median(tilts):.1f}° / "
          f"最小 {tilts.min():.1f}° / 最大 {tilts.max():.1f}°（尽力而为）")

    print()
    if failures:
        print(f"FAIL（{len(failures)} 项）：")
        for f in failures:
            print("  -", f)
        sys.exit(1)
    print("PASS：全部检查通过")


if __name__ == "__main__":
    main()
