# -*- coding: utf-8 -*-
"""ReBot Arm B601-DM 的 MuJoCo 运行器。

用法（在项目根目录 D:\\Study\\Zju\\Competition\\2026_Digital_Twin 下运行）:

    # 1. 看关节信息 + 正运动学示例
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py

    # 2. 交互式查看器（启动时 7 关节全部断电：只受重力+摩擦，可拖动，松手停/被拉掉）
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py --view

    # 3. 交互式查看器（启动时 7 关节全部锁位：像通电舵机，拖动会弹回）
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py --hold
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py --hold --angles 0 -60 -60 30 0 0

    # 4. 指定 6 轴角度(度)，打印法兰/TCP 的世界坐标 + 各关节重力负载（静态正运动学 FK，不仿真）
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py --angles 0 -90 -45 0 0 0

    # 5. 位置伺服（无头）：真实物理仿真驱动机械臂走到目标角度并打印结果
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py --move 0 -90 -90 45 20 0
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/rebot_run.py --move 0 -90 -90 45 20 0 --gripper 0.015

交互式查看器的终端指令（与真机 CLI tools/cli_arm_controller.py 同款语法）:
    off <关节 1-7>     断电该关节（只受重力+摩擦，可拖动）
    on  <关节 1-7>     锁在当前角度（on = 全部锁当前姿态）
    test <关节> [秒]   定量坠落测试：卸力 N 秒报 Δ° 与时间序列，自动恢复（对照真机同款命令）
    pose <J1..J6>      设置 6 轴角度（锁位的关节伺服过去，断电的关节瞬移）
    list               打印 7 关节模式与角度
    help               指令速查
    q                  退出并清力矩

说明：
    - 关节状态可任意混合：例如 J2/J3 锁位 + J4 断电 = 在仿真里复现真机的单轴坠落测试。
    - 断电 = 该关节力矩清零（只受重力+摩擦）；锁位 = 计算力矩伺服（mj_inverse，见 rebot_control.py）。
    - 夹爪开度单位米：0=闭合，0.05=全开；夹 30mm 电池约开 0.015。
    - 运行时在【终端】输入指令（点一下终端窗口打字，回车生效），3D 窗口看效果。
"""
import argparse
import math
import os
import queue
import sys
import threading
import time

import numpy as np
import mujoco

from rebot_control import arm_servo, gripper_servo, GEAR_RADIUS

# 路径锚定：无论从哪个目录启动都切到项目根（模型/输出全用相对路径，与 tools/ 同一约定）
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MODEL = "mujoco_sim/models/ReBot_Arm_DM.xml"
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)] + ["gripper_gear"]


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def clip_targets(model: mujoco.MjModel, targets_deg, margin_deg: float = 2.0) -> np.ndarray:
    """把 6 个目标角(度)限幅到关节限位内（留 margin_deg 度余量，避免顶在软限位上）。"""
    out = []
    for j in range(1, 7):
        lo, hi = model.jnt_range[model.joint(f"joint{j}").id]
        margin = math.radians(margin_deg)
        out.append(float(np.clip(math.radians(targets_deg[j - 1]), lo + margin, hi - margin)))
    return np.asarray(out)


def print_joints(model: mujoco.MjModel) -> None:
    # 独立自由度 = 位置变量数 − 等式约束 − 定长腱（两条齿轮-齿条腱各钉住 1 个相对自由度）
    independent = model.nq - model.neq - model.ntendon
    print(f"模型: ReBot_Arm_DM  |  关节定义 {model.njnt} 个（6 臂 + 1 齿轮 + 2 夹指）")
    print(f"  其中 2 条齿轮-齿条啮合腱 -> 独立自由度 {independent} = 6 臂 + 1 齿轮")
    print("-" * 62)
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        typ = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(int(model.jnt_type[i]), "?")
        lo, hi = model.jnt_range[i]          # 内部单位始终是弧度
        unit = "rad" if typ == "hinge" else "m"
        print(f"  {name:14s}  type={typ:5s}  范围 {lo:+.2f} ~ {hi:+.2f} {unit}")
    print("-" * 62)


def forward_kinematics(model: mujoco.MjModel, data: mujoco.MjData,
                       angles_deg, gripper: float) -> None:
    """给定 6 轴角度(度) + 夹爪开度(米)，打印法兰与 TCP 的世界坐标（静态，不仿真）。"""
    data.qpos[:] = 0.0
    for j, name in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = deg2rad(angles_deg[j])
    # 齿轮角 = 开度 / 齿轮半径；两指由齿轮-齿条啮合约束自动反向平移
    data.qpos[model.jnt_qposadr[model.joint("gripper_gear").id]] = gripper / GEAR_RADIUS
    data.qpos[model.jnt_qposadr[model.joint("finger_left").id]] = gripper
    data.qpos[model.jnt_qposadr[model.joint("finger_right").id]] = -gripper

    mujoco.mj_forward(model, data)

    flange = data.site_xpos[model.site("flange").id]      # 腕末端机械法兰
    tcp = data.site_xpos[model.site("tcp").id]            # 夹持中心（两指尖之间）
    load = [data.qfrc_bias[model.jnt_dofadr[model.joint(f"joint{j}").id]] for j in range(1, 7)]
    print(f"关节角(度): {angles_deg}")
    print(f"夹爪开度:   {gripper:.3f} m (0=闭, 0.05=全开; 夹30mm电池约0.015)")
    print(f"法兰坐标(X,Y,Z): [{flange[0]:.4f}, {flange[1]:.4f}, {flange[2]:.4f}] m")
    print(f"TCP 坐标(X,Y,Z): [{tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f}] m  (距基座 {float(np.linalg.norm(tcp)):.4f} m)")
    print(f"重力负载(N·m) J1..J6: {[round(l, 2) for l in load]}  (校准关节 frictionloss 用：真机稳得住=>frictionloss>此值；会被拉掉=>小于此值)")


def position_control(model: mujoco.MjModel, data: mujoco.MjData,
                     targets_deg, gripper: float, duration: float = 4.0) -> None:
    """计算力矩伺服：真实驱动机械臂走到目标角度（无头，跑 duration 秒后打印结果）。"""
    motor_idx = [model.actuator(f"motor{i}").id for i in range(1, 7)]
    joint_idx = [model.joint(f"joint{i}").id for i in range(1, 7)]
    f_motor = model.actuator("gripper_gear_motor").id
    f_joint = model.joint("gripper_gear").id

    targets = clip_targets(model, targets_deg)   # 目标角限幅（留 2° 余量）
    dt = model.opt.timestep
    steps = int(duration / dt)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    for _ in range(steps):
        arm_servo(model, data, targets, joint_idx, motor_idx)
        gripper_servo(model, data, gripper, f_motor, f_joint)
        mujoco.mj_step(model, data)

    achieved = [np.degrees(data.qpos[model.jnt_qposadr[j]]) for j in joint_idx]
    eff_targets = [np.degrees(t) for t in targets]
    errs = [abs(achieved[i] - eff_targets[i]) for i in range(6)]
    tcp = data.site_xpos[model.site("tcp").id]
    print(f"位置伺服运行 {duration:.0f} 秒后结果:")
    print(f"  目标角度(度): {[round(t, 2) for t in eff_targets]}  (原始输入: {targets_deg})")
    print(f"  实际角度(度): {[round(a, 2) for a in achieved]}")
    print(f"  各轴误差(度): {[round(e, 2) for e in errs]}")
    print(f"  夹爪开度: {data.qpos[model.jnt_qposadr[f_joint]] * GEAR_RADIUS:.4f} m (目标 {gripper:.3f})")
    print(f"  最终 TCP: [{tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f}] m")


def viewer_interactive(model: mujoco.MjModel, data: mujoco.MjData,
                       start_hold: bool, angles_deg=None, gripper: float = 0.0) -> None:
    """交互式查看器：7 个关节（6 臂 + 夹爪齿轮）各自可独立"锁位/断电"，
    终端指令语法与真机 CLI（tools/cli_arm_controller.py）完全一致。

    start_hold : True=启动时全部锁位（对应 --hold），False=全部断电自由（对应 --view）。
    angles_deg : 初始 6 轴角(度)，None=零位。
    gripper    : 夹爪锁位目标开度(米)。

    语义映射：断电 = 该关节力矩清零（只受重力+摩擦，鼠标可拖动）；
              锁位 = 计算力矩伺服锁当前角（拖动弹回，像通电舵机）。
    """
    from mujoco import viewer

    N = 7
    motor_idx = [model.actuator(f"motor{i}").id for i in range(1, 7)] + \
                [model.actuator("gripper_gear_motor").id]
    qadr = [model.jnt_qposadr[model.joint(n).id] for n in JOINT_NAMES]
    joint_ids = [model.joint(n).id for n in JOINT_NAMES]

    if angles_deg is not None:
        for j in range(6):
            data.qpos[qadr[j]] = deg2rad(angles_deg[j])

    holding = [start_hold] * N                            # True=伺服锁位, False=断电自由
    targets = [data.qpos[qadr[i]] for i in range(N)]      # 锁位目标（弧度）
    testing = [None] * N                                  # 坠落测试状态: dict(t0,dur,a0,series,last)
    grip_target = gripper                                 # 夹爪锁位目标开度(米)
    sample_dt = 0.2                                       # 序列采样周期(秒)

    cmd_q = queue.Queue()
    stop = [False]

    def stdin_worker():
        """后台线程：读终端指令行，投递给主仿真循环执行。"""
        while True:
            try:
                line = input()
            except EOFError:
                cmd_q.put(None)
                return
            cmd_q.put(line.strip())

    threading.Thread(target=stdin_worker, daemon=True).start()

    def jname(i):
        return f"J{i+1}" if i < 6 else "夹爪"

    def do_cmd(line):
        nonlocal grip_target
        parts = line.split()
        if not parts:
            return
        c = parts[0].lower()

        if c in ("q", "quit", "exit"):
            stop[0] = True
        elif c == "list":
            st = "  ".join(f"{jname(i)}:{'锁' if holding[i] else '断'}:"
                           f"{np.degrees(data.qpos[qadr[i]]):+.1f}°" for i in range(6))
            print(f"\n  [状态] {st} | 爪:{'锁' if holding[6] else '断'}"
                  f":{np.degrees(data.qpos[qadr[6]]):+.1f}°")
        elif c in ("off", "on"):
            if len(parts) < 2:
                if c == "on":                             # on = 全部锁当前姿态
                    for i in range(N):
                        holding[i] = True
                        testing[i] = None
                        targets[i] = data.qpos[qadr[i]]
                    grip_target = data.qpos[qadr[6]] * GEAR_RADIUS   # 夹爪锁当前开度
                    print("[+] 全部关节已锁在当前角度")
                else:
                    print("[!] 格式: off <关节 1-7>")
                return
            try:
                j = int(parts[1]) - 1
            except ValueError:
                print("[!] 关节编号须为数字 1-7")
                return
            if not (0 <= j < N):
                print("[!] 关节编号错误，可选 1 ~ 7")
                return
            if c == "off":
                holding[j] = False
                testing[j] = None
                print(f"[+] {jname(j)} 已断电（只受重力+摩擦，可拖动；on {j+1} 恢复）")
            else:
                holding[j] = True
                testing[j] = None
                targets[j] = data.qpos[qadr[j]]
                if j == 6:
                    grip_target = data.qpos[qadr[6]] * GEAR_RADIUS   # 锁在当前开度(米)
                    print(f"[+] 夹爪已锁在开度 {grip_target:.3f} m")
                else:
                    print(f"[+] {jname(j)} 已锁在 {np.degrees(data.qpos[qadr[j]]):+.1f}°")
        elif c == "test":
            if len(parts) < 2:
                print("[!] 格式: test <关节 1-7> [秒]")
                return
            try:
                j = int(parts[1]) - 1
                sec = float(parts[2]) if len(parts) > 2 else 3.0
            except ValueError:
                print("[!] 参数须为数字")
                return
            if not (0 <= j < N):
                print("[!] 关节编号错误，可选 1 ~ 7")
                return
            testing[j] = dict(t0=time.time(), dur=sec, a0=data.qpos[qadr[j]],
                              series=[], last=0.0)
            holding[j] = False
            print(f"[~] {jname(j)} 卸力 {sec:.0f}s，观察坠落（到点自动恢复锁位）...")
        elif c == "pose":
            if len(parts) != 7:
                print("[!] 格式: pose <J1> <J2> <J3> <J4> <J5> <J6>")
                return
            vals = [deg2rad(float(v)) for v in parts[1:7]]
            for j in range(6):
                data.qpos[qadr[j]] = vals[j]
                if holding[j]:
                    targets[j] = vals[j]
                testing[j] = None                               # pose 瞬移会使坠落测试的 a0 失效
            mujoco.mj_forward(model, data)
            print("[+] 姿态已设置（断电关节为瞬移，锁位关节伺服到位）")
        elif c == "help":
            print("  off/on/test <关节 1-7> [秒] | pose <6角> | on(全锁) | list | q")
        else:
            print(f"[?] 未识别指令: {line}（off/on/test <关节> | pose | list | q）")

    handle = viewer.launch_passive(model, data)
    print("交互式查看器就绪：在【终端】输入指令（点一下终端窗口打字），3D 窗口看效果。")
    print("  指令: off/on/test <关节 1-7> [秒] | pose <6角> | list | help | q   （同真机 CLI 语法）")

    try:
        while handle.is_running() and not stop[0]:
            # 1) 处理终端命令
            try:
                while True:
                    line = cmd_q.get_nowait()
                    if line is None:
                        stop[0] = True
                        break
                    try:
                        do_cmd(line)
                    except ValueError as e:
                        print(f"\n[!] 指令参数错误（已忽略）: {e}")
            except queue.Empty:
                pass
            if stop[0] or not handle.is_running():
                break

            # 2) 坠落测试采样/到点恢复
            now = time.time()
            for i in range(N):
                t = testing[i]
                if t is None:
                    continue
                if now - t["last"] >= sample_dt:
                    t["last"] = now
                    t["series"].append(round(float(np.degrees(data.qpos[qadr[i]])), 1))
                if now - t["t0"] >= t["dur"]:
                    a1 = float(np.degrees(data.qpos[qadr[i]]))
                    drop = abs(a1 - float(np.degrees(t["a0"])))
                    verdict = "会掉 ✗" if drop > 2.0 else "稳住 ✓"
                    holding[i] = True
                    targets[i] = data.qpos[qadr[i]]
                    if i == 6:
                        grip_target = data.qpos[qadr[6]] * GEAR_RADIUS
                    testing[i] = None
                    print(f"\n[+] {jname(i)} 卸力 {t['dur']:.0f}s: "
                          f"{float(np.degrees(t['a0'])):+.1f}° → {a1:+.1f}° (Δ={drop:.1f}°) ⇒ {verdict}")
                    print("    序列: " + " ".join(str(x) for x in t["series"]))

            # 3) 控制力：锁位关节伺服到目标；断电关节力矩清零
            eff = [data.qpos[qadr[i]] if not holding[i] else targets[i] for i in range(6)]
            arm_servo(model, data, np.asarray(eff), joint_ids[:6], motor_idx[:6])
            if holding[6]:
                gripper_servo(model, data, grip_target, motor_idx[6], joint_ids[6])
            else:
                data.ctrl[motor_idx[6]] = 0.0
            for i in range(6):
                if not holding[i]:
                    data.ctrl[motor_idx[i]] = 0.0

            mujoco.mj_step(model, data)
            handle.sync()
    finally:
        data.ctrl[:] = 0.0
        handle.close()
        print("[*] 查看器已关闭，力矩已清零。")


def main() -> None:
    parser = argparse.ArgumentParser(description="ReBot Arm B601-DM MuJoCo 运行器")
    parser.add_argument("--view", action="store_true",
                        help="交互式查看器：启动时 7 关节全部断电（重力+摩擦，可拖动）")
    parser.add_argument("--angles", type=float, nargs="*", default=None,
                        help="6 个关节角(度)：--angles 单独用时只算静态正运动学；配 --view/--hold 时设初始姿态")
    parser.add_argument("--move", type=float, nargs="*", default=None,
                        help="6 个目标关节角(度)，跑真实物理仿真驱动到位（无头）")
    parser.add_argument("--gripper", type=float, default=0.0,
                        help="夹爪开度(米)，0 闭 ~ 0.05 全开")
    parser.add_argument("--duration", type=float, default=4.0,
                        help="--move 的位置伺服运行时长(秒)，默认 4")
    parser.add_argument("--hold", action="store_true",
                        help="交互式查看器：启动时 7 关节全部锁位（可配 --angles；运行中 off/on 逐轴切换）")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    print_joints(model)

    if args.hold or args.view:
        if args.angles is not None and len(args.angles) != 6:
            print("错误：--angles 需要正好 6 个角度（对应 joint1~joint6）")
            sys.exit(1)
        viewer_interactive(model, data, start_hold=args.hold,
                           angles_deg=args.angles, gripper=args.gripper)
        return

    if args.move is not None:
        if len(args.move) != 6:
            print("错误：--move 需要正好 6 个目标角（对应 joint1~joint6）")
            sys.exit(1)
        position_control(model, data, args.move, args.gripper, args.duration)
    elif args.angles is not None:
        if len(args.angles) != 6:
            print("错误：--angles 需要正好 6 个角度（对应 joint1~joint6）")
            sys.exit(1)
        forward_kinematics(model, data, args.angles, args.gripper)
    else:
        print("\n(未指定 --angles/--move，演示一个示例姿态)")
        forward_kinematics(model, data, [0.0, -60.0, -60.0, 30.0, 0.0, 0.0], 0.0)


if __name__ == "__main__":
    main()
