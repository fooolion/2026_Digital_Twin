#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
达妙机械臂独立硬件遥测服务 (只读安全监听模式)
功能：
1. 循环查询 7 轴达妙电机的物理位置 (rad)、速度 (rad/s)、受到的真实力矩 (N·m)
2. 保持电机失能，支持安全手扳/负载检测
3. 对外通过 UDP 广播及 JSON 文件同步给 MuJoCo/Unity 消费端

本机修改 (2026-10-02, 李郑阳侧)：
- 串口默认 COM8（队长机为 COM13）；可用命令行第一个参数覆盖：python test_force.py COM13
- JSON 路径锚定到项目根（原写法依赖当前工作目录）
- gripper_m 单位修正：原样写入的是电机弧度，现按 GEAR_RADIUS=0.008 换算成米（假设电机零位=全闭合）
- 默认**不写** latest_state.json（该文件是 sim_bridge.py → Unity 渲染链路的通道，新服务不默认抢占）；
  确需覆写（独立力面板模式，不同时跑 sim_bridge）时显式加 --json
"""

import os
import sys
import time
import json
import socket
import motorbridge

# 硬件配置
SERIAL_PORT = "COM8"          # 队长机为 COM13；本机实测 COM8
BAUD_RATE = 921600
GEAR_RADIUS = 0.008           # 夹爪齿轮节圆半径(米)，与 mujoco_sim/rebot_control.py 的 GEAR_RADIUS 保持一致（勿单边修改）

# 数据分发配置
UDP_BROADCAST_IP = "127.0.0.1"
UDP_BROADCAST_PORT = 9004  # 与既有遥测端口保持一致
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_OUTPUT_PATH = os.path.join(_ROOT, "mujoco_sim", "output", "latest_state.json")
WRITE_JSON = "--json" in sys.argv   # 默认不写（防抢写渲染链路通道）；显式 --json 才覆写

# 7 轴拓扑配置 (J1~J3: DM4340P, J4~J6+Gripper: DM4310)
ARM_JOINTS = [
    {"name": "J1", "id": 0x01, "mst_id": 0x11, "model": "4340P"},
    {"name": "J2", "id": 0x02, "mst_id": 0x12, "model": "4340P"},
    {"name": "J3", "id": 0x03, "mst_id": 0x13, "model": "4340P"},
    {"name": "J4", "id": 0x04, "mst_id": 0x14, "model": "4310"},
    {"name": "J5", "id": 0x05, "mst_id": 0x15, "model": "4310"},
    {"name": "J6", "id": 0x06, "mst_id": 0x16, "model": "4310"},
    {"name": "Gripper", "id": 0x07, "mst_id": 0x17, "model": "4310"},
]

def extract_torque(state_obj):
    """安全解析不同底层版本的力矩属性字段"""
    if state_obj is None:
        return 0.0
    for field in ("tau", "torq", "torque", "t"):
        if hasattr(state_obj, field):
            return float(getattr(state_obj, field, 0.0))
    return 0.0

def main():
    """遥测主循环：50Hz 轮询 7 轴 → UDP 9004 广播 + (加 --json 时)覆写 latest_state.json。"""
    # 串口号 = 第一个不以 - 开头的命令行参数（兼容 --json 与位置参数任意顺序）
    port = next((a for a in sys.argv[1:] if not a.startswith("-")), SERIAL_PORT)
    print(f"[*] 正在初始化达妙 CAN 网关 ({port} @ {BAUD_RATE})...")
    try:
        ctrl = motorbridge.Controller.from_dm_serial(serial_port=port, baud=BAUD_RATE)
    except Exception as e:
        print(f"[!] 串口连接失败: {e}。请确认 DM 调试工具是否已关闭。")
        sys.exit(1)

    motors = []
    for j in ARM_JOINTS:
        m = ctrl.add_damiao_motor(j["id"], j["mst_id"], j["model"])
        motors.append(m)

    # 确保保存路径存在
    os.makedirs(os.path.dirname(JSON_OUTPUT_PATH), exist_ok=True)
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("[*] 正在确保所有电机处于空闲失能状态（安全监测）...")
    for m in motors:
        try:
            m.disable()
        except Exception:
            pass

    print("[+] 遥测服务启动成功！按 Ctrl+C 退出。")
    print("[*] 等待 7 轴全部收到首帧反馈（收到前不发送，避免全 0 包）...")
    loop_period = 0.02  # 50 Hz 轮询
    last_print = 0.0
    last_v = [None] * 7                    # 每轴上一帧有效 (pos, vel, tau)；丢帧时沿用旧值

    try:
        while True:
            t0 = time.time()

            # 轮询请求反馈
            for m in motors:
                if hasattr(m, "request_feedback"):
                    m.request_feedback()
            ctrl.poll_feedback_once()

            positions = []
            velocities = []
            torques = []

            ready = True
            for i, m in enumerate(motors):
                st = m.get_state()
                if st is not None:
                    last_v[i] = (round(st.pos, 4), round(st.vel, 4), round(extract_torque(st), 4))
                if last_v[i] is None:      # 该轴从未收到反馈 → 整包不发（0 不是合法的"未知"值）
                    ready = False
                    break
                positions.append(last_v[i][0])
                velocities.append(last_v[i][1])
                torques.append(last_v[i][2])
            if not ready:
                time.sleep(0.05)
                continue

            # 1. 组装并发送 UDP 遥测包
            telemetry = {
                "timestamp": round(time.time(), 3),
                "q_act": positions[:6],
                "v_act": velocities[:6],
                "tau_act": torques[:6],       # 6 轴电机承受真实力矩 (N·m)
                "gripper_pos": positions[6],
                "gripper_tau": torques[6]
            }
            tx_sock.sendto(json.dumps(telemetry).encode("utf-8"), (UDP_BROADCAST_IP, UDP_BROADCAST_PORT))

            # 2. 覆写 latest_state.json（符合系统数据规范；默认跳过——那是 sim_bridge.py →
            #    Unity 渲染链路的通道，确需覆写时加 --json，且不要与 sim_bridge 同时跑）
            if WRITE_JSON:
                state_data = {
                    "t": telemetry["timestamp"],
                    "joints_deg": [round(p * 57.29578, 2) for p in positions[:6]],
                    "gripper_m": round(-positions[6] * GEAR_RADIUS, 4),   # 取负：实机负角=张开，
                                                                          # 对齐仿真口径 正=张开（见 rebot_control.py gripper_servo）
                    "motor_torque": torques[:6],
                    "motor_friction": [0.0] * 6,
                    "motor_load": torques[:6]
                }
                tmp_file = JSON_OUTPUT_PATH + ".tmp"
                with open(tmp_file, "w") as f:
                    json.dump(state_data, f)
                os.replace(tmp_file, JSON_OUTPUT_PATH)

            # 3. 终端 5Hz 看板
            if time.time() - last_print > 0.2:
                status_str = " | ".join([f"J{i+1}:{torques[i]:+5.2f}" for i in range(6)])
                print(f"\r[50Hz 遥测] 真实受力(N·m) -> {status_str} | 爪:{torques[6]:+5.2f}", end="", flush=True)
                last_print = time.time()

            cost = time.time() - t0
            if cost < loop_period:
                time.sleep(loop_period - cost)

    except KeyboardInterrupt:
        print("\n[*] 正在关闭服务...")
    finally:
        for closer in (lambda: ctrl.close(), lambda: tx_sock.close()):
            try:
                closer()
            except Exception:
                pass
        print("[+] 资源已释放。")

if __name__ == "__main__":
    main()