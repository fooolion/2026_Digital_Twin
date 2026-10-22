# -*- coding: utf-8 -*-
"""机械臂关节伺服控制（计算力矩控制 / computed torque）。

给 rebot_run.py 和 sim_bridge.py 共用的控制器，避免两处各写一套、越改越不一致。

核心思路 —— 为什么不用"重力补偿 + 统一 PD"了：
    机械臂几个关节是互相耦合的：动肩肘会拽动腕部（惯性矩阵 M 的非对角项），
    还有科里奥利力。旧方法只抵消重力（qfrc_bias 的一部分），没抵消这些耦合，
    结果腕关节（惯量极小、增益却和肩肘一样大）在所有姿态都会高频颤振
    （约 8 rad/s 的极限环），误差看着不大其实是抖出来的假象。

    计算力矩控制：直接调用 MuJoCo 的逆动力学 mj_inverse，精确算出
    "要实现想要的关节加速度，需要给每个关节多大力矩"（重力/科里奥利/耦合
    全部一起抵消）。之后每个关节就像独立的 弹簧-阻尼 系统：
        q'' = kp * (target - q)  -  kd * q'
    其中 kp = wn^2，kd = 2*zeta*wn。这样各关节平滑、不颤振、没静差。

注意：
    - mj_inverse 不感知碰撞接触（只解无约束的逆动力学）。所以当机械臂真的
      卡到障碍/自碰撞时，它会照常要求力矩，臂顶在障碍上到不了目标——这属
      于"目标不可达"，不是控制器毛病。
    - 电机力矩仍受模型里 ctrllimited/ctrlrange 上限约束，超了会被 MuJoCo 截断。
"""
import mujoco


def arm_servo(model: mujoco.MjModel, data: mujoco.MjData,
              targets, joint_idx, motor_idx,
              wn: float = 8.0, zeta: float = 1.0) -> list:
    """6 个臂关节的计算力矩伺服：把电机力矩写进 data.ctrl，返回 6 个力矩(N·m)。

    参数
    ----
    targets    : 6 个臂关节的目标角（弧度，np.array 或 list，长度 6）
    joint_idx  : 6 个臂关节的 id 列表（joint1~joint6）
    motor_idx  : 6 个臂电机的 id 列表（motor1~motor6）
    wn         : 期望闭环自然频率(rad/s)，越大响应越快；默认 8（约 1.3 Hz）
    zeta       : 阻尼比，1 = 临界阻尼（快且不振荡）；默认 1

    说明：wn/zeta 是"伺服手感"旋钮——想让臂更快就调大 wn，若发现抖就调小 wn。
    """
    kp = wn * wn
    kd = 2.0 * zeta * wn

    # 先把所有自由度的期望加速度清零（含两个夹指），再只填 6 个臂关节
    data.qacc[:] = 0.0
    for i in range(6):
        da = model.jnt_dofadr[joint_idx[i]]
        qa = model.jnt_qposadr[joint_idx[i]]
        data.qacc[da] = kp * (targets[i] - data.qpos[qa]) - kd * data.qvel[da]

    # 逆动力学：把想要的加速度换算成各关节需要的力矩
    mujoco.mj_inverse(model, data)

    torques = []
    for i in range(6):
        da = model.jnt_dofadr[joint_idx[i]]
        data.ctrl[motor_idx[i]] = data.qfrc_inverse[da]
        torques.append(float(data.qfrc_inverse[da]))
    return torques


GEAR_RADIUS = 0.008   # 夹爪齿轮节圆半径(米)：模数1、16齿，齿条位移 = r × 齿轮角(rad)


def gripper_servo(model: mujoco.MjModel, data: mujoco.MjData,
                  gripper: float, gear_motor: int, gear_joint: int,
                  kp: float = 3.0, kd: float = 0.5) -> float:
    """夹爪齿轮 PD 伺服：把目标开度(米)换算成齿轮角，驱动齿轮电机。

    两侧夹爪不再单独驱动——它们靠模型里的"齿轮-齿条啮合"约束(定长腱)
    自动反向平移（finger_left = +r·θ，finger_right = -r·θ），对应实机
    中心电机→小齿轮→两侧齿条→平动开合。

    gripper : 目标开度（米），0 = 闭合，0.05 = 全开
    返回值  : 齿轮电机力矩(N·m)
    """
    gd = model.jnt_dofadr[gear_joint]
    gq = model.jnt_qposadr[gear_joint]
    target = gripper / GEAR_RADIUS      # 开度(米) -> 齿轮角(rad)
    err = target - data.qpos[gq]
    tau = kp * err - kd * data.qvel[gd]
    data.ctrl[gear_motor] = tau
    return float(tau)