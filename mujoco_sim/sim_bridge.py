# -*- coding: utf-8 -*-
"""MuJoCo <-> Unity 双向桥接（MuJoCo 侧）。

下行（Unity -> MuJoCo）：读 target.json 里的目标点（夹爪 TCP 要到的位置）+ 夹爪开度，
    用两级优先（词典序）逆解求 6 关节角：位置高精度到达（≤0.1mm）硬优先，
    指垫倾角在位置不动的前提下尽力压低（见 solve_ik 文档串），
    计算力矩驱动机械臂到位、夹爪开合。
上行（MuJoCo -> Unity）：把 6 关节角 + 夹爪开度 + 电机力矩反馈写到 latest_state.json。

用法（项目根目录 D:\\Study\\Zju\\Competition\\2026_Digital_Twin）:
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/sim_bridge.py            # 一直跑直到 Ctrl+C
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/sim_bridge.py --view      # 同时打开 MuJoCo 查看器
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/sim_bridge.py --duration 30

目标文件 mujoco_sim/output/target.json（Unity 侧写）:
    {"active": true, "x": 0.30, "y": 0.0, "z": 0.35, "gripper": 0.015}
    - x/y/z 是 MuJoCo 世界坐标（Z 轴向上，米）。
    - gripper 是夹爪开度（米）：0=闭合，0.05=全开；缺省 0（保持闭合）。夹30mm电池约 0.015。
    有 active=false 或缺文件时走演示轨迹（夹爪慢开合演示）。

停止：Ctrl+C
输出：mujoco_sim/output/latest_state.json
      （6 关节角 + 电机力矩/摩擦/负载反馈，直接写入，失败自动重试）
"""
import argparse
import json
import math
import os
import stat
import time

import numpy as np
import mujoco

from rebot_control import arm_servo, gripper_servo, GEAR_RADIUS

# 路径锚定：无论从哪个目录启动都切到项目根（模型/输出全用相对路径，与 tools/ 同一约定）
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MODEL = "mujoco_sim/models/ReBot_Arm_DM.xml"
OUT = "mujoco_sim/output/latest_state.json"
TARGET = "mujoco_sim/output/target.json"


def write_state(path: str, t: float, angles_deg, gripper: float,
                motor_torque, motor_friction, motor_load) -> None:
    """把状态写到 JSON（直接覆盖写）。失败时清只读属性 + 重试，不因占用/只读崩掉。

    除关节角外，还写 6 个电机的力矩反馈（单位 N·m，有符号）：
      motor_torque   电机施加力矩
      motor_friction 摩擦力矩（库仑 + 粘性）
      motor_load     负载力矩（重力 + 科里奥利/离心）
    """
    data = {
        "t": round(t, 4),
        "joints_deg": [round(float(a), 4) for a in angles_deg],
        "gripper_m": round(float(gripper), 4),
        "motor_torque": [round(float(x), 3) for x in motor_torque],
        "motor_friction": [round(float(x), 3) for x in motor_friction],
        "motor_load": [round(float(x), 3) for x in motor_load],
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    text = json.dumps(data)

    # 原子写：先写临时文件再 os.replace，Unity 读端不会看到写了一半的 JSON
    tmp = path + ".tmp"
    for _ in range(10):
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(text)
            os.replace(tmp, path)
            return
        except PermissionError:
            # 可能是文件被设成只读、或被别的进程短暂占用：清只读属性后重试
            try:
                os.chmod(path, stat.S_IWRITE)
            except OSError:
                pass
            time.sleep(0.01)
    print(f"[警告] 写入 {path} 反复失败（文件被占用？），跳过本次")


def read_target(path: str):
    """读目标点文件，返回 (是否激活, target_pos 或 None, 夹爪开度 0.0~0.05)。

    target_pos 为 MuJoCo 世界坐标(米)；夹爪开度单位米(0=闭, 0.05=全开)，缺省 0。
    """
    if not os.path.exists(path):
        return False, None, 0.0
    try:
        with open(path, encoding="utf-8") as f:
            t = json.load(f)
        if not t.get("active", False):
            return False, None, 0.0
        pos = np.array([float(t["x"]), float(t["y"]), float(t["z"])])
        if not (np.isfinite(pos).all() and np.linalg.norm(pos) < 2.0):
            return False, None, 0.0       # NaN/Inf 或工作空间外的离谱值：回落演示轨迹
        gripper = float(t.get("gripper", 0.0))
        if not math.isfinite(gripper):
            gripper = 0.0
        gripper = max(0.0, min(0.05, gripper))   # 限位在夹爪行程内
        return True, pos, gripper
    except Exception:
        return False, None, 0.0


def solve_ik(model, ik_data, target_pos, joint_idx, site_name="tcp", ori_gain=1.0):
    """两级优先（词典序）逆解：求 6 个臂关节的目标角度（弧度）。

    输入/约定：
        在独立的 ik_data 副本上迭代（不动主仿真的 data），初始姿态照抄当前 data。
        只动 6 个臂关节，夹爪（齿轮 + 两指，3 个自由度）不参与。
        target_pos 为 MuJoCo 世界坐标（Z 轴向上，米）。

    两级优先（词典序，2026-10-09 方案 v2，用户拍板）：
        ① 位置硬约束：目标点高精度到达（≤0.1mm），任何情况下优先——抓取前提；
        ② 倾角尽力而为：位置不动的前提下，尽量压低"夹指内平面与底座面的夹角" α
          （指垫法向 = tcp 局部 Y，两指沿它滑动闭合；α → 0 即指垫水平）。

    姿态残差 = 倾角 α × 水平倾转轴 â（把指垫法向转回竖直所需旋转矢量的水平分量；
    绕竖直轴的偏航是冗余方向，不约束）。注意不能用 (u_y,-u_x) 做残差：其模长
    sin(α) 在 α=90° 处梯度为零，指垫竖直姿态下求解器会"失明"（实测验证）。
    ori_gain = 倾角行权重（位置优先由接受准则保证，与权重无关；默认 1 = 同权）。

    算法：
        1. 词典序自适应 LM：位置改善即接受（守邻域内倾角不许明显恶化）；位置
           守住（≤0.1mm+松弛）且倾角改善才接受；否则加阻尼重试，λ 到顶收敛退出。
        2. 位置零空间精修 tilt_nullspace_polish：dq = -(I-Jp⁺Jp)·∇α 一阶不动
           TCP，与位置回拉交替，只接受"位置回到容差且倾角改善"的轮次。
        3. 纯位置收尾：把倾角步的位置松弛修回 ≤0.1mm。

    起点搜索顺序（倾角低于愿望线 3° 即提前返回）：热启动（当前姿态）→ 帧间
    记忆（solve_ik.memo，免每帧重采样）→ 跨帧精英库（solve_ik.elites）→ 按
    目标哈希播种的分轮随机预筛（≤2 万点，摸到低倾角片区提前收工，限时 0.12s。
    实测基准：热启动命中帧 ~2ms、memo 命中帧 ~7ms、全量 2 万点 ~0.86s——终审）。

    实测（tools/test_ik_horizontal.py）：拖拽 15 步位置最差 0.097mm；冷启动
    0.006mm/0.013°；随机 20 目标位置全部 <1mm、倾角中位 0.0°/最大 2.0°。
    行为说明：为压倾角允许翻解分支（伺服一阶惯性 ~0.5s 平滑，翻后稳定不振荡）。
    """
    sid = model.site(site_name).id

    dof_ads = [model.jnt_dofadr[j] for j in joint_idx]   # 6 个臂关节在 qvel/qfrc 里的地址
    qpos_ads = [model.jnt_qposadr[j] for j in joint_idx]  # 6 个臂关节在 qpos 里的地址
    lo_hi = [model.jnt_range[j] for j in joint_idx]

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    Z = np.array([0.0, 0.0, 1.0])
    LAM_MIN, LAM_MAX = 1e-6, 1e3
    max_step = np.radians(10)      # 信赖域：单步关节增量上限
    POS_TOL = 1e-4                 # 一级目标：位置精度 0.1mm（硬要求）
    HOLD_TOL = 5e-4                # 倾角步期间允许的位置松弛（随后由位置步修回）
    TILT_WANT = np.radians(3)      # 倾角愿望线：低于它就不再触发精英/采样搜索

    def build(gain):
        """forward + 计算 5×6 雅可比与误差向量（姿态部分按 gain 加权），返回 (A, err, alpha, pos_norm)。"""
        mujoco.mj_forward(model, ik_data)
        mujoco.mj_jacSite(model, ik_data, jacp, jacr, sid)
        pos_err = target_pos - ik_data.site_xpos[sid]
        u = ik_data.site_xmat[sid].reshape(3, 3)[:, 1]   # 夹指闭合方向（tcp 局部 Y）的世界方向
        s_t = np.hypot(u[0], u[1])
        alpha = np.arctan2(s_t, abs(u[2]))        # 倾角 ∈ [0, π/2]，u 朝上/朝下等价
        Jori = np.zeros((2, 6))
        ori_err = np.zeros(2)
        if s_t > 1e-9:
            y_eq = Z if u[2] >= 0 else -Z         # 就近的竖直方向
            a_h = np.cross(u, y_eq) / s_t         # 水平倾转轴 â：绕它转 α 恰把 u 转到 y_eq
            Jws = jacr[:, dof_ads]                # (3,6) 各关节角速度列 ω_k
            dw = np.cross(np.cross(Jws.T, u), y_eq)          # dw_k = (ω_k×u)×y_eq
            dA = (dw - np.outer(dw @ a_h, a_h)) / s_t        # dâ/dq_k
            J3 = np.outer(a_h, -(Jws.T @ a_h)) + alpha * dA.T  # de_k = â·dα_k + α·dâ_k
            Jori = gain * J3[:2]
            ori_err = gain * alpha * a_h[:2]
        A = np.vstack([jacp[:, dof_ads], Jori])   # 5×6
        err = np.concatenate([pos_err, ori_err])
        return A, err, alpha, float(np.linalg.norm(pos_err))

    def lm_run(gain, max_iters):
        """词典序自适应 LM：位置硬优先（≤0.1mm），位置不动时降倾角。返回 (pos_norm, alpha)。

        接受准则（严格优先级，不是加权折中）：
          - 位置能改善 → 接受（倾角不许明显恶化，防"位置步白赚倾角变差"）；
          - 位置已守住（≤POS_TOL+松弛）且倾角改善 → 接受（倾角步允许位置小幅松弛，
            下一轮位置步会修回）；
          - 都不满足 → 加阻尼重试，λ 到顶仍不行则收敛退出。
        gain=0 时退化为纯位置 LM（用于收尾修复）。
        """
        lam = 1e-3
        A, err, alpha, posn = build(gain)
        for _ in range(max_iters):
            if posn < POS_TOL and alpha < 2e-3:
                break                                     # 已水平，无事可做
            q0 = ik_data.qpos[qpos_ads].copy()
            accepted = False
            for _try in range(8):
                dq = A.T @ np.linalg.solve(A @ A.T + lam * lam * np.eye(5), err)
                worst = float(np.abs(dq).max())
                if worst > max_step:
                    dq *= max_step / worst        # 等比缩到信赖域内，方向不变
                for k in range(6):
                    lo, hi = lo_hi[k]
                    ik_data.qpos[qpos_ads[k]] = float(np.clip(q0[k] + dq[k], lo, hi))
                A2, err2, alpha2, posn2 = build(gain)
                if posn2 < posn - 1e-7:
                    # 位置改善：逼近阶段（还离目标远）倾角自由；守邻域内才保护倾角
                    take = (alpha2 <= alpha + 1e-3) if posn <= POS_TOL + HOLD_TOL else True
                elif posn2 <= POS_TOL + HOLD_TOL and alpha2 < alpha - 1e-5:
                    take = True                           # 位置守住，倾角改善
                else:
                    take = False
                if take:
                    A, err, alpha, posn = A2, err2, alpha2, posn2
                    lam = max(lam * 0.5, LAM_MIN)
                    accepted = True
                    break                                 # 接受后进入下一轮外层迭代
                lam *= 4.0                                # 试步变差：加阻尼重来
                ik_data.qpos[qpos_ads] = q0               # 先还原，保证状态与误差对应
                if lam > LAM_MAX:
                    break
            if not accepted:
                break                                     # 加大阻尼也走不动，收敛（或卡住）
        return posn, alpha

    def tilt_nullspace_polish(max_rounds=40):
        """位置零空间内的倾角下降（流形上精修）：零空间投影梯度步 + 位置回拉，交替。

        零空间投影 dq = -(I - Jp⁺Jp)·∇α 数学上保证一阶不动 TCP，绕开"位置步与
        倾角步互相打架"。只接受位置回到容差且倾角改善的轮次，不改善即停并回滚。
        返回 (posn, alpha)。
        """
        best_a = None
        best_q = None
        for _ in range(max_rounds):
            A, err, alpha, posn = build(ori_gain)
            if best_a is None:
                best_a, best_q = alpha, ik_data.qpos[qpos_ads].copy()
            Jp = A[:3, :]                                 # 位置雅可比 (3,6)
            Nproj = np.eye(6) - Jp.T @ np.linalg.solve(Jp @ Jp.T + 1e-4 * np.eye(3), Jp)
            dq = -(Nproj @ (A[3:, :].T @ err[3:]))        # 零空间内的倾角下降步
            worst = float(np.abs(dq).max())
            if worst < 1e-9:
                break
            if worst > max_step:
                dq *= max_step / worst
            q0 = ik_data.qpos[qpos_ads].copy()
            for k in range(6):
                lo, hi = lo_hi[k]
                ik_data.qpos[qpos_ads[k]] = float(np.clip(q0[k] + dq[k], lo, hi))
            posn, alpha = lm_run(gain=0.0, max_iters=60)  # 位置回拉（只接受位置下降）
            if posn <= POS_TOL * 2 and alpha < best_a - 1e-4:
                best_a, best_q = alpha, ik_data.qpos[qpos_ads].copy()
            else:
                ik_data.qpos[qpos_ads] = best_q           # 这轮没赚：回滚到历史最优
                mujoco.mj_forward(model, ik_data)
                break
        lm_run(gain=0.0, max_iters=60)                    # 收尾：位置修到 ≤0.1mm
        mujoco.mj_forward(model, ik_data)
        u = ik_data.site_xmat[sid].reshape(3, 3)[:, 1]
        alpha = float(np.arctan2(np.hypot(u[0], u[1]), abs(u[2])))
        posn = float(np.linalg.norm(target_pos - ik_data.site_xpos[sid]))
        return posn, alpha

    # 严格优先级（词典序）：①位置硬约束（≤0.1mm——目标点必须到，这是抓取的前提）；
    # ②位置不动时尽力压低指垫倾角（尽力而为，物理下限由腕部结构与限位决定）。
    # 第 1 个起点 = 当前姿态（拖拽热启动，倾角低于愿望线即毫秒级返回）。
    best = None

    def remember(score, q):
        """把本次结果并入跨帧精英库（去近似重、按分保留最优 8 个）。"""
        solve_ik.elites = [e for e in solve_ik.elites if np.abs(q - e[1]).max() > 0.03]
        solve_ik.elites.append((float(score), q))
        solve_ik.elites.sort(key=lambda t: t[0])
        del solve_ik.elites[8:]

    def attempt(qs):
        """对种子 qs 跑词典序 LM + 零空间精修。

        返回 (ok, alpha)：ok = 位置达标（<1mm）；alpha = 最终指垫倾角（弧度）。
        无论 ok 与否都更新 best（倾角优先、同倾角取近分支）并记入精英库。
        """
        nonlocal best
        ik_data.qpos[qpos_ads] = qs
        posn, alpha = lm_run(gain=ori_gain, max_iters=150)
        if posn > POS_TOL:
            posn, alpha = lm_run(gain=0.0, max_iters=60)   # 收尾：修回倾角步的位置松弛
        posn, alpha = tilt_nullspace_polish()              # 流形上精修倾角（位置不动）
        if posn > POS_TOL:
            posn, alpha = lm_run(gain=0.0, max_iters=60)
        score = alpha + 0.02 * posn
        remember(score, ik_data.qpos[qpos_ads].copy())
        # best 选择：位置不达标的候选用 10·posn 罚到后面；位置达标时按（倾角, 关节
        # 距离）字典序——倾角差 >1° 倾角优先，倾角相当选近分支（防隔帧翻腕）。
        jm = float(np.abs(ik_data.qpos[qpos_ads] - q_seed_arr).max())
        eff = alpha if posn < 1e-3 else alpha + 10.0
        if (best is None or eff < best[2] - np.radians(1)
                or (eff <= best[2] + np.radians(1) and jm < best[3])):
            best = (score, ik_data.qpos[qpos_ads].copy(), eff, jm)
        return posn < 1e-3, alpha

    q_seed = ik_data.qpos[qpos_ads].copy()
    q_seed_arr = np.asarray(q_seed)              # attempt 的闭包引用，必须在首调前赋值
    warm_ok, warm_alpha = attempt(q_seed)
    if warm_ok and warm_alpha < TILT_WANT:
        solve_ik.memo = None                     # 位置达标且倾角够低，无需帧间记忆
        return np.array([ik_data.qpos[a] for a in qpos_ads])

    # 帧间记忆：热启动 attempt 未达愿望线时（该点倾角下限高于愿望线，或位置本身
    # 不可达），没有记忆的话下面重采样每帧都要跑（实测全量 2 万点 ~0.86s），50Hz
    # 循环直接掉到个位数。规则：目标相对记忆锚点单帧移动 <1.5cm（覆盖手速
    # 0.5m/s@60Hz 的快速拖拽）且累计漂移 <5cm，就直接沿用上帧解（位置已达标，
    # 倾角为该点可达值）；漂移超限或热启动结果更好则重新采样（限时 0.12s）。
    memo = solve_ik.memo
    if memo is not None:
        mt, mq, ms, mdrift = memo
        step = float(np.linalg.norm(target_pos - mt))
        if step < 1.5e-2 and mdrift + step < 5e-2 and ms <= best[0]:
            solve_ik.memo = (np.array(target_pos), mq, ms, mdrift + step)
            ik_data.qpos[qpos_ads] = mq
            mujoco.mj_forward(model, ik_data)
            return np.array([ik_data.qpos[a] for a in qpos_ads])

    # 跨帧精英库：历史各次求解留下的位置达标构型（按倾角分保留最优 8 个）。冷
    # 启动时先试它们——上一帧成功的片区附近再拖拽，通常一个精英就中，免掉整段
    # 采样。连续性守卫：达标但离当前姿态任一关节差 >90° 的解（别解分支，如 J6
    # 绕半圈）先记作备胎继续找；整轮没有更近的才用，避免拖拽中臂当场翻腕。
    def jump_big():
        return float(np.abs(ik_data.qpos[qpos_ads] - q_seed_arr).max()) > np.radians(90)

    far_fallback = None
    for _, eq in list(solve_ik.elites):
        ok, al = attempt(eq)
        if ok and al < TILT_WANT:
            solve_ik.memo = None
            if jump_big():
                if far_fallback is None:
                    far_fallback = ik_data.qpos[qpos_ads].copy()
                continue
            return np.array([ik_data.qpos[a] for a in qpos_ads])

    # 冷启动兜底采样：倾角下降是局部方法，冷启动种子的零空间盆地决定能压到多低，
    # 低倾角盆地需靠随机采样探明（纯局部下降实测停在 80~90°）。分轮撒点：每轮
    # 2000 点换一批种子（按目标量化哈希播种——同一目标稳定、不同目标探索不同
    # 区域），pool 里出现"倾角 <2.9° 且距目标 <5cm"的原始样本就提前收工，最多
    # 10 轮共 2 万点（实测 2 万点可命中 tcp0 片区，2000 点不够——审查发现）。
    # 命中后解进精英库跨帧复用，这段成本对同一目标只付一次。
    # 预算与跳过（审查实测教训）：采样只算运动学（mj_kinematics，比 mj_forward
    # 便宜数倍），且限时 0.12s——拖拽中 memo 漂移每打穿一次预算就付一次，不设限
    # 会每 5cm 卡 ~0.9s；已有 far_fallback（精英库里的可行备胎）时整段跳过，
    # 审查实测此时再撒 2 万点对最终解零贡献。
    if far_fallback is None:
        lo_arr = np.array([lo for lo, _ in lo_hi])
        hi_arr = np.array([hi for _, hi in lo_hi])
        base_seed = int(np.abs(np.sum(np.round(np.asarray(target_pos) * 1000.0)
                                      * np.array([1, 997, 1000003]))))
        pool = []
        deadline = time.time() + 0.12
        kin = mujoco.mj_kinematics
        for round_i in range(10):
            rng = np.random.default_rng(base_seed + round_i)
            for _ in range(2000):
                qs = rng.uniform(lo_arr, hi_arr)
                ik_data.qpos[qpos_ads] = qs
                kin(model, ik_data)
                u = ik_data.site_xmat[sid].reshape(3, 3)[:, 1]
                a = float(np.arctan2(np.hypot(u[0], u[1]), abs(u[2])))
                p = float(np.linalg.norm(target_pos - ik_data.site_xpos[sid]))
                pool.append((a + 0.5 * p, a, p, qs))
            near = min(pool, key=lambda c: c[0])
            if near[1] < 0.05 and near[2] < 0.05:
                break                                 # 已摸到可行片区，不必再撒
            if time.time() > deadline:
                break                                 # 本帧预算用完，余下覆盖交给下次打穿
        pool.sort(key=lambda c: c[0])
        picked = [c[3] for c in pool[:4]]
        picked.append(min(pool, key=lambda c: c[1])[3])   # 倾角最小的专项候选
        picked.append(min(pool, key=lambda c: c[2])[3])   # 位置最近的专项候选
        for qs in picked:
            ok, al = attempt(qs)
            if ok and al < TILT_WANT:
                solve_ik.memo = None
                if jump_big():
                    if far_fallback is None:
                        far_fallback = ik_data.qpos[qpos_ads].copy()
                    continue
                return np.array([ik_data.qpos[a] for a in qpos_ads])

    if far_fallback is not None:                 # 只有远分支的达标解：用备胎
        ik_data.qpos[qpos_ads] = far_fallback
        mujoco.mj_forward(model, ik_data)
        return np.array([ik_data.qpos[a] for a in qpos_ads])

    solve_ik.memo = (np.array(target_pos), best[1].copy(), best[0], 0.0)
    ik_data.qpos[qpos_ads] = best[1]              # 全部起点都没到愿望线：回放倾角最优
    mujoco.mj_forward(model, ik_data)
    return np.array([ik_data.qpos[a] for a in qpos_ads])


solve_ik.memo = None     # 帧间记忆 (目标点, 折中解, 评分, 累计漂移)，逻辑见 solve_ik 尾部
solve_ik.elites = []     # 跨帧精英库 [(评分, 关节角)]，最多 8 个


def motor_feedback(model, data, joint_idx, motor_idx):
    """计算并返回 6 个电机的力矩反馈（单位 N·m，有符号，沿 +q 方向）。

    三个返回列表：
      motor_torque   电机施加力矩（data.actuator_force）
      motor_friction 摩擦力矩（库仑 frictionloss + 粘性 damping，运动时与速度反向）
      motor_load     负载力矩（data.qfrc_bias：重力 + 科里奥利/离心）

    摩擦直接按模型里的关节摩擦公式算（MuJoCo 不单独暴露这一项）：
      friction = frictionloss * sign(qvel) + damping * qvel
    """
    motor_torque = []
    motor_friction = []
    motor_load = []
    for i in range(6):
        dadr = model.jnt_dofadr[joint_idx[i]]
        v = data.qvel[dadr]
        motor_torque.append(float(data.actuator_force[motor_idx[i]]))
        motor_friction.append(float(-(model.dof_damping[dadr] * v
                                      + model.dof_frictionloss[dadr] * np.sign(v))))
        motor_load.append(float(data.qfrc_bias[dadr]))
    return motor_torque, motor_friction, motor_load


def demo_target(sim_t: float) -> np.ndarray:
    """演示/校准轨迹：6 个臂关节**轮流**单独摆动（每个占一个时间片，其余保持不变）。

    这样每个关节动的时候其它关节是静止的，方便逐个核对 Unity 侧的轴向与转向对不对。
    顺序 J1→J6，每段 seg 秒循环；各关节用各自的中心角和摆幅（都在限位内）。
    """
    seg = 3.5                                        # 每个关节摆动时长(秒)
    centers = [0.0, -60.0, -60.0, 0.0, 0.0, 0.0]     # 各关节中心角(度)；整体是个伸出去的姿态，腕部/夹爪看得清
    amps = [60.0, 40.0, 40.0, 25.0, 40.0, 90.0]      # 各关节摆幅(度)

    idx = int(sim_t / seg) % 6                       # 当前轮到哪个关节
    phase = (sim_t % seg) / seg * 2 * math.pi        # 段内相位
    targets = np.array([math.radians(c) for c in centers])
    targets[idx] = math.radians(centers[idx] + amps[idx] * math.sin(phase))
    return targets


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo<->Unity 桥接")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="运行秒数；0 = 一直跑直到 Ctrl+C")
    parser.add_argument("--view", action="store_true",
                        help="同时打开 MuJoCo 交互查看器（方便和 Unity 对比）")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    ik_data = mujoco.MjData(model)   # 供 IK 使用，不参与主仿真
    dt = model.opt.timestep
    # 约 50 Hz 写一次 / 重解一次 IK（每 0.02s）
    write_every = max(1, int(round(0.02 / dt)))

    wn, zeta = 8.0, 1.0           # 位置伺服手感（计算力矩控制，见 rebot_control.py）
    motor_idx = [model.actuator(f"motor{i}").id for i in range(1, 7)]
    joint_idx = [model.joint(f"joint{i}").id for i in range(1, 7)]
    f_motor = model.actuator("gripper_gear_motor").id
    f_joint = model.joint("gripper_gear").id

    viewer_handle = None
    if args.view:
        from mujoco import viewer
        viewer_handle = viewer.launch_passive(model, data)
        print("MuJoCo 查看器已打开")

    print(f"MuJoCo 桥接已启动，输出: {OUT}")
    print(f"目标点文件: {TARGET}（active=true 时走拖拽 IK，否则走演示轨迹）")
    print("（Ctrl+C 停止）")

    step = 0
    last_target_angles = None
    try:
        while True:
            sim_t = step * dt

            # 决定本步的目标角度：拖拽 IK 模式 or 演示轨迹
            active, target_pos, grip_cmd = read_target(TARGET)
            if active:
                if step % write_every == 0:
                    # 以实时姿态为初值求 IK，返回 6 个臂关节的逆解角度。
                    # 求解器任何异常都不许杀掉桥接（实机演示时 Unity 侧输入不可控）：
                    # 失败帧沿用上一帧目标角（首帧则保持当前姿态）
                    ik_data.qpos[:] = data.qpos[:]
                    try:
                        last_target_angles = solve_ik(model, ik_data, target_pos, joint_idx)
                    except Exception as exc:
                        print(f"[警告] solve_ik 异常（{exc!r}），本帧沿用上一目标角")
                if last_target_angles is None:
                    # 刚进入拖拽模式、还没解出时，先保持当前姿态，避免跳变
                    last_target_angles = np.array(
                        [data.qpos[model.jnt_qposadr[j]] for j in joint_idx])
                targets = last_target_angles
                gripper_target = grip_cmd            # 夹爪开度来自 Unity 指令
            else:
                last_target_angles = None
                targets = demo_target(sim_t)
                # 演示：夹爪全行程慢开合（0~0.05m），方便核对齿轮轴向/方向
                gripper_target = 0.025 + 0.025 * math.sin(2 * math.pi * 0.12 * sim_t)

            # 计算力矩伺服，驱到 targets；夹爪由齿轮电机经齿条驱动开合
            arm_servo(model, data, targets, joint_idx, motor_idx, wn, zeta)
            gripper_servo(model, data, gripper_target, f_motor, f_joint)

            mujoco.mj_step(model, data)
            step += 1

            if step % write_every == 0:
                angles = [np.degrees(data.qpos[model.jnt_qposadr[j]]) for j in joint_idx]
                actual_gripper = data.qpos[model.jnt_qposadr[f_joint]] * GEAR_RADIUS
                motor_torque, motor_friction, motor_load = motor_feedback(
                    model, data, joint_idx, motor_idx)
                write_state(OUT, sim_t, angles, actual_gripper,
                            motor_torque, motor_friction, motor_load)
                if viewer_handle is not None:
                    viewer_handle.sync()   # 把最新 state 推给查看器，否则它不动

            if args.duration > 0 and sim_t >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if viewer_handle is not None:
            viewer_handle.close()
    print("桥接已停止。")


if __name__ == "__main__":
    main()