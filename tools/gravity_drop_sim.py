# -*- coding: utf-8 -*-
"""仿真侧自由坠落对照：给定姿态零速释放，跑 N 秒纯重力（控制力矩=0），
输出各关节 Δ°，与真机 `test` 命令的同窗口 Δ° 对比 → 标定 damping。

判读：某关节 仿真Δ > 真机Δ ⇒ 仿真掉得太快 ⇒ 调大该轴 damping；反之调小。

用法:
    mujoco_sim/.rebot/Scripts/python.exe tools/gravity_drop_sim.py --angles 0 -50 -40 0 0 0
    mujoco_sim/.rebot/Scripts/python.exe tools/gravity_drop_sim.py --angles 0 -50 -40 0 0 0 --seconds 3
    mujoco_sim/.rebot/Scripts/python.exe tools/gravity_drop_sim.py --angles 0 -50 -40 75 0 0 \
        --hold joint2,joint4 --seconds 1.0 --series   # 冻结使能锁位的轴 + 时序输出（时序对表用）
"""
import argparse
import os
import sys

import numpy as np
import mujoco

# MuJoCo 的 C 库打不开含非 ASCII 的绝对路径，锚定项目根后用相对路径加载（与 safety_monitor.py 一致）
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(_ROOT)

MJCF = 'mujoco_sim/models/ReBot_Arm_DM.xml'
J = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def main():
    """解析姿态/冻结轴/时长参数，跑自由坠落仿真并输出各关节 Δ° 时序。"""
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    ap = argparse.ArgumentParser()
    ap.add_argument('--angles', nargs=6, type=float, required=True, help='J1..J6 初始角度(度)')
    ap.add_argument('--seconds', type=float, default=3.0, help='坠落窗口时长(秒), 默认 3, 与真机 test 窗口一致')
    ap.add_argument('--hold', default='', help='冻结的关节名, 逗号分隔(如 joint2,joint4)——对应真机使能锁位的轴')
    ap.add_argument('--series', action='store_true', help='每 0.2s 打印一次各关节角度(时序对表用)')
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(MJCF)
    d = mujoco.MjData(m)
    d.qpos[:] = 0
    for i, a in enumerate(args.angles):
        d.qpos[m.joint(J[i]).qposadr[0]] = np.radians(a)
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)

    q0 = d.qpos.copy()
    hold_dofs = [m.joint(n).dofadr[0] for n in args.hold.split(',') if n]
    n = int(args.seconds / m.opt.timestep)
    sample_every = max(1, int(0.2 / m.opt.timestep))
    for k in range(n):
        d.ctrl[:] = 0                                     # 无控制力矩: 纯重力+摩擦+damping
        for dofi in hold_dofs:
            d.qvel[dofi] = 0.0                            # 冻结对应真机使能锁位的轴
        mujoco.mj_step(m, d)
        if args.series and k % sample_every == 0:
            degs = [round(float(np.degrees(d.qpos[m.joint(nm).qposadr[0]])), 2) for nm in J]
            print(f'  t={k * m.opt.timestep:4.1f}s  {degs}')

    print(f'姿态 {args.angles}  自由坠落 {args.seconds:.0f}s (ctrl=0):')
    for i, name in enumerate(J):
        adr = m.jnt_qposadr[m.joint(name).id]
        dd = np.degrees(d.qpos[adr] - q0[adr])
        print(f'  {name}: {dd:+8.2f}°  (|Δ|={abs(dd):.2f}°)')


if __name__ == '__main__':
    main()
