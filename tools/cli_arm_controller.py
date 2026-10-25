# -*- coding: utf-8 -*-
"""达妙机械臂真机 CLI 控制终端（reBot B601-DM，7 轴 = 6 关节 + 夹爪）。

用途
  - 日常单轴运动、整臂摆姿态；
  - 摩擦标定：卸力观察/定量坠落测试，给 MuJoCo 的 frictionloss 对表提供真机数据。

依赖（mujoco_sim/.rebot 环境）
    pip install motorbridge pyserial    # motorbridge = Seeed 官方电机 SDK（PyPI）

运行（机械臂 24V 上电、USB-CAN 接好后）
    python tools/cli_arm_controller.py

交互指令
    <关节 1-7> <角度>          单轴平滑运动，角度单位度（如 2 -10）
    pose <J1>..<J6>            整臂依次驱到 6 个角度（对齐仿真 --angles 姿态用）
    list                       打印 7 轴状态表
    off <关节>                 卸力单轴（可手扳，观察稳住/会掉；确认后执行）
    on <关节>                  恢复使能并锁在当前角度
    tau [秒]                   7 轴扭矩速读（N·m），T_CLAMP 夹紧标定用（默认刷一次）
    test <关节> [秒]           定量坠落测试：卸力 N 秒报 Δ°，Δ>2° 判"会掉"，自动恢复
    q                          退出并逐轴卸力（不要直接点终端 ×，会让电机保持使能）

安全
  - 每个目标角都先过 JOINTS_CONFIG 的软限位，超限拒绝执行；
  - off / test 需要二次确认，执行时该轴会失去力矩，请扶住臂身。
"""
import math
import sys
import time

import motorbridge

PORT = "COM8"            # USB-CAN 适配器串口号（设备管理器里的"USB 串行设备"；插拔后可能变号）
BAUDRATE = 921600        # 串口波特率 bit/s，与适配器/电机固件一致

# 7 轴配置：can_id/mst_id = 电机 CAN 地址/应答地址（hex）；min_deg/max_deg = 安全软限位（度）；
# vlim = 运动限速（rad/s）。改限位/限速只动这张表。
JOINTS_CONFIG = [
    {"id": 1, "name": "J1_Base",     "can_id": 0x01, "mst_id": 0x11, "model": "4340P", "min_deg": -165.0, "max_deg": 165.0, "vlim": 1.2},
    {"id": 2, "name": "J2_Shoulder", "can_id": 0x02, "mst_id": 0x12, "model": "4340P", "min_deg": -115.0, "max_deg":  60.0, "vlim": 1.0},
    {"id": 3, "name": "J3_Elbow",    "can_id": 0x03, "mst_id": 0x13, "model": "4340P", "min_deg": -140.0, "max_deg":  45.0, "vlim": 1.0},
    {"id": 4, "name": "J4_Wrist1",   "can_id": 0x04, "mst_id": 0x14, "model": "4310",  "min_deg": -170.0, "max_deg": 170.0, "vlim": 1.5},
    {"id": 5, "name": "J5_Wrist2",   "can_id": 0x05, "mst_id": 0x15, "model": "4310",  "min_deg": -115.0, "max_deg": 115.0, "vlim": 1.5},
    {"id": 6, "name": "J6_Wrist3",   "can_id": 0x06, "mst_id": 0x16, "model": "4310",  "min_deg": -180.0, "max_deg": 180.0, "vlim": 1.5},
    {"id": 7, "name": "J7_Gripper",  "can_id": 0x07, "mst_id": 0x17, "model": "4310",  "min_deg": -343.0, "max_deg":   0.0, "vlim": 1.0},
]
# J7 夹爪方向（2026-10-02 真机实测）：0°=闭合（初始姿态），负角度=张开，全开约 -343°（实测近似值）。


def _cfg(joint_id):
    """按关节编号(1-7)取配置项。"""
    return next(c for c in JOINTS_CONFIG if c["id"] == joint_id)


def check_port(port):
    """启动前检查串口是否存在，不存在就列出可用口并退出。

    port: 串口号字符串，如 "COM8"。
    没有 pyserial 时静默跳过（不阻塞启动）。
    """
    try:
        import serial.tools.list_ports
    except ImportError:
        return
    names = [p.device for p in serial.tools.list_ports.comports()]
    if port not in names:
        print(f"[!] 找不到串口 {port}，当前可用: {names or '无'}")
        print("[*] 打开设备管理器确认 USB-CAN 的端口号，改本文件顶部 PORT 后重试。")
        sys.exit(1)


class CliArmController:
    """真机臂的连接管理、运动控制与摩擦标定，交互入口见 run_repl()。"""

    def __init__(self):
        """建立 CAN 网关，注册 7 台电机，自检在线状态并切入 POS_VEL 模式。"""
        check_port(PORT)
        print(f"[*] 正在建立 CAN 通信网关 ({PORT} @ {BAUDRATE})...")
        self.ctrl = motorbridge.Controller.from_dm_serial(serial_port=PORT, baud=BAUDRATE)
        self.motors = {}                                 # 关节编号(1-7) -> 电机对象

        print("[*] 注册 7 台达妙电机节点并切换至 POS_VEL 模式...")
        for j in JOINTS_CONFIG:
            self.motors[j["id"]] = self.ctrl.add_damiao_motor(j["can_id"], j["mst_id"], j["model"])

        # 清总线：连发 15 轮反馈请求，把上次会话残留的帧排干
        for _ in range(15):
            for m in self.motors.values():
                if hasattr(m, "request_feedback"):
                    m.request_feedback()
            self.ctrl.poll_feedback_once()
            time.sleep(0.01)

        # 总在线自检：逐台要一次反馈，区分"总线不通"和"拒绝写入"两类故障
        silent = []
        for j in JOINTS_CONFIG:
            m = self.motors[j["id"]]
            if hasattr(m, "request_feedback"):
                m.request_feedback()
            self.ctrl.poll_feedback_once()
            time.sleep(0.02)
            if m.get_state() is None:
                silent.append(j["name"])
        if silent:
            raise RuntimeError(
                f"以下电机在总线上拿不到任何反馈: {', '.join(silent)}\n"
                f"[!] 检查：① 机械臂 24V 电源是否上电 ② 该关节 CAN 线插头是否松动"
                f"（底座 J1 最常见） ③ USB-CAN 适配器是否被其他程序占用。排除后重新运行。")

        # 切 POS_VEL 模式：上次没卸力退出时电机常有残留状态，重试通常能过
        for j in JOINTS_CONFIG:
            m = self.motors[j["id"]]
            last = None
            for _ in range(3):
                try:
                    m.ensure_mode(motorbridge.MODE_POS_VEL)
                    last = None
                    break
                except Exception as e:
                    last = e
                    self.ctrl.poll_feedback_once()
                    time.sleep(0.3)
            if last is not None:
                raise RuntimeError(
                    f"{j['name']} 设置工作模式失败（重试 3 次）: {last}\n"
                    f"[!] 处理：给机械臂断电 10 秒再上电，关掉其他占用端口的终端，重新运行。")

    # ---------- 状态查询 ----------

    def get_joint_actual_deg(self, joint_id):
        """读某关节当前实际角度（反馈丢失自动重试 3 次）。

        joint_id: 关节编号 1-7。返回角度，单位度。
        连续 3 次拿不到反馈时抛 RuntimeError——绝不静默返回 0.0：
        该值会被用作绝对位置指令的起点（move_joint）和锁位目标（hold），
        反馈丢失时装作 0° 会让臂以限速全速甩向 0 位（真机安全红线）。
        """
        m = self.motors[joint_id]
        st = None
        for _ in range(3):
            if hasattr(m, "request_feedback"):
                m.request_feedback()
            self.ctrl.poll_feedback_once()
            st = m.get_state()
            if st is not None:
                break
            time.sleep(0.05)
        if st is None:
            raise RuntimeError(f"{_cfg(joint_id)['name']} 反馈丢失（重试 3 次）——中止本次操作，"
                               f"绝不以 0° 兜底下发绝对位置指令")
        rad = getattr(st, "pos", getattr(st, "position", 0.0))
        return math.degrees(rad)

    def print_status_table(self):
        """打印 7 轴状态表（当前角度 / 软限位 / 限速）。"""
        print("\n" + "=" * 80)
        print(f"{'编号':<4} | {'关节名称':<12} | {'当前实际角度':<14} | {'安全限制范围 (Min ~ Max)':<24} | {'限速 (rad/s)'}")
        print("-" * 80)
        for j in JOINTS_CONFIG:
            try:
                curr_deg = self.get_joint_actual_deg(j["id"])
                curr_str = f"{curr_deg:.2f}°"
            except RuntimeError:
                curr_str = "无反馈"
            lim_str = f"[{j['min_deg']:>6.1f}° ~ {j['max_deg']:>6.1f}°]"
            print(f"  {j['id']:<3} | {j['name']:<12} | {curr_str:>12}    | {lim_str:<24} | {j['vlim']:>6.1f}")
        print("=" * 80)

    # ---------- 运动 ----------

    def move_joint(self, joint_id, target_deg, duration=2.5):
        """单轴平滑运动到目标角度（先过软限位，五次多项式 S 曲线防冲击）。

        joint_id:   关节编号 1-7；
        target_deg: 目标角度，单位度；
        duration:   行程时间，单位秒（默认 2.5）。
        返回 True=已执行 / False=被软限位拒绝。
        """
        cfg = _cfg(joint_id)
        if not (cfg["min_deg"] <= target_deg <= cfg["max_deg"]):
            print(f"\n[!] 拒绝执行：目标角度 {target_deg:.2f}° 超出限制范围"
                  f" [{cfg['min_deg']:.1f}°, {cfg['max_deg']:.1f}°]！")
            return False

        motor = self.motors[joint_id]
        start_rad = math.radians(self.get_joint_actual_deg(joint_id))
        target_rad = math.radians(target_deg)

        print(f"\n[*] 正在使能电机 {cfg['name']} (ID: 0x{cfg['can_id']:02X})...")
        motor.enable()
        time.sleep(0.1)

        # 五次多项式 10t³-15t⁴+6t⁵：起止速度/加速度都为 0，缓启缓停
        dt = 0.02                                         # 指令下发周期，单位秒（50 Hz）
        steps = int(duration / dt)
        print(f"[*] 开始平滑驱动至 {target_deg:.2f}° (行程耗时 {duration:.1f}s)...")
        for step in range(steps + 1):
            t = step / steps
            s = 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5
            motor.send_pos_vel(start_rad + s * (target_rad - start_rad), cfg["vlim"])
            self.ctrl.poll_feedback_once()
            time.sleep(dt)

        # 重力下位置环有零点几度的稳态误差（肩/肘明显），补发逼近到位
        time.sleep(0.2)
        for _ in range(3):
            try:
                end_deg = self.get_joint_actual_deg(joint_id)
            except RuntimeError:
                end_deg = None
            if end_deg is None or abs(end_deg - target_deg) <= 0.3:
                break
            motor.send_pos_vel(target_rad, cfg["vlim"])
            time.sleep(0.4)
        try:
            end_deg = self.get_joint_actual_deg(joint_id)
            end_str = f"{end_deg:.2f}°"
        except RuntimeError:
            end_str = "无反馈（运动已下发，读回失败）"
        print(f"[+] 运动完成！最终反馈角度: {end_str} (目标: {target_deg:.2f}°)")
        return True

    def print_torques(self, duration=0.0):
        """打印 7 轴当前电机扭矩（N·m）。

        duration: 持续刷新时长，单位秒；0 = 只打印一次。
        用途：T_CLAMP 夹紧标定——闭合夹爪夹住物体后读 Gripper 扭矩。
        """
        t0 = time.time()
        while True:
            vals = []
            for j in JOINTS_CONFIG:
                m = self.motors[j["id"]]
                if hasattr(m, "request_feedback"):
                    m.request_feedback()
                self.ctrl.poll_feedback_once()
                st = m.get_state()
                tau = 0.0
                if st is not None:
                    for f in ("tau", "torq", "torque", "t"):
                        if hasattr(st, f):
                            tau = float(getattr(st, f, 0.0))
                            break
                vals.append(tau)
            print("\r" + " | ".join(f"{j['name'].split('_')[0]}:{v:+6.2f}"
                                    for j, v in zip(JOINTS_CONFIG, vals)), end="", flush=True)
            if duration <= 0 or time.time() - t0 >= duration:
                print()
                break
            time.sleep(0.2)

    def move_pose(self, deg6, duration=2.5):
        """J1..J6 依次驱到 6 个给定角度（把真机摆到仿真 --angles 的同款姿态）。

        deg6:     长度 6 的角度列表，单位度，对应 J1~J6；
        duration: 每轴行程时间，单位秒。
        """
        for jid, deg in zip(range(1, 7), deg6):
            cfg = _cfg(jid)
            if cfg["min_deg"] <= deg <= cfg["max_deg"]:
                self.move_joint(jid, deg, duration)
            else:
                print(f"[!] 跳过 J{jid}: {deg}° 超出安全范围 [{cfg['min_deg']:.1f}°, {cfg['max_deg']:.1f}°]")

    # ---------- 摩擦标定（配合 MuJoCo frictionloss 对表） ----------

    def limp(self, joint_id):
        """卸力单轴（交互命令 off）：该轴失去力矩，受重力可能坠落，可手扳换姿态。"""
        cfg = _cfg(joint_id)
        print(f"[!] 即将卸力 {cfg['name']}：该轴会失去力矩，受重力可能坠落，请扶住臂身！")
        if input("    确认请输入 y: ").strip().lower() != "y":
            print("[*] 已取消。")
            return
        self.motors[joint_id].disable()
        print(f"[+] {cfg['name']} 已卸力。观察：稳住 or 会掉；手扳可换姿态；输 'on {joint_id}' 恢复。")

    def hold(self, joint_id):
        """重新使能单轴并锁在当前角度（交互命令 on）。

        读不到反馈时抛 RuntimeError 中止（保持卸力状态是安全的），
        绝不锁向 0° 兜底。
        """
        cfg = _cfg(joint_id)
        deg = self.get_joint_actual_deg(joint_id)
        m = self.motors[joint_id]
        m.enable()
        time.sleep(0.1)
        try:
            m.ensure_mode(motorbridge.MODE_POS_VEL)
        except Exception:
            time.sleep(0.2)
            m.ensure_mode(motorbridge.MODE_POS_VEL)   # 首调常超时（见构造函数），重试一次
        m.send_pos_vel(math.radians(deg), cfg["vlim"])
        print(f"[+] {cfg['name']} 已恢复使能，锁在 {deg:.2f}°。")

    def drop_test(self, joint_id, seconds=3.0):
        """定量坠落测试（交互命令 test）：卸力 N 秒后报告角度变化并自动恢复。

        joint_id: 关节编号 1-7；
        seconds:  卸力时长，单位秒（默认 3）。
        判据：|Δ| > 2° 视为"会掉"（frictionloss 应 > 此姿态重力负载），否则"稳住"（应 <）。
        返回角度变化量的绝对值，单位度；用户取消时返回 None。
        """
        cfg = _cfg(joint_id)
        if input(f"[!] {cfg['name']} 将卸力 {seconds:.0f}s（可能坠落），请做好保护，确认 y: "
                 ).strip().lower() != "y":
            print("[*] 已取消。")
            return None
        a0 = self.get_joint_actual_deg(joint_id)
        self.motors[joint_id].disable()
        t0 = time.time()
        series = []                                       # (秒, 角度) 时间序列，供坠落速度对表
        last = a0                                         # 丢帧时沿用上一采样，序列保持连续
        while time.time() - t0 < seconds:
            t = time.time() - t0
            try:
                last = self.get_joint_actual_deg(joint_id)
            except RuntimeError:
                pass
            series.append((t, last))
            time.sleep(0.2)
        a1 = last
        self.hold(joint_id)                               # 无论采样是否丢帧，恢复使能必被执行
        drop = abs(a1 - a0)
        verdict = "会掉 ✗（frictionloss 应 > 此姿态重力负载）" if drop > 2.0 \
            else "稳住 ✓（frictionloss 应 < 此姿态重力负载）"
        print(f"[+] {cfg['name']} 卸力 {seconds:.0f}s: {a0:.2f}° → {a1:.2f}° (Δ={drop:.2f}°) ⇒ {verdict}")
        print("    序列: " + " ".join(f"{t:.1f}s:{a:.1f}" for t, a in series))
        return drop

    # ---------- 交互终端 ----------

    def run_repl(self):
        """命令行交互主循环；输入 q 走 shutdown() 安全退出。"""
        print("\n[+] 达妙机械臂 CLI 交互控制终端就绪")
        print("[*] 运动指令: <关节 1-7> <目标角度>  |  pose <J1..J6 六角度>  |  list 刷新状态")
        print("[*] 摩擦标定: off <关节> 卸力 | on <关节> 恢复锁位 | test <关节> [秒] 定量坠落测试")
        print("[*] 输入 'q' 退出并断电卸力")
        self.print_status_table()

        try:
            while True:
                try:
                    raw = input("\n请输入控制指令 (例如: 1 45.0 / pose 0 -50 -40 0 0 0 / test 3): ").strip()
                    if not raw:
                        continue
                    if raw.lower() in ("q", "quit", "exit"):
                        break
                    if raw.lower() in ("list", "status", "ls"):
                        self.print_status_table()
                        continue

                    parts = raw.split()
                    cmd = parts[0].lower()

                    if cmd in ("off", "on", "test"):          # 摩擦标定三件套
                        if len(parts) < 2:
                            print(f"[!] 格式: {cmd} <关节 1-7>" + (" [秒]" if cmd == "test" else ""))
                            continue
                        jid = int(parts[1])
                        if jid not in range(1, 8):
                            print("[!] 关节编号错误，可选范围为 1 ~ 7")
                            continue
                        if cmd == "off":
                            self.limp(jid)
                        elif cmd == "on":
                            self.hold(jid)
                        else:
                            self.drop_test(jid, float(parts[2]) if len(parts) > 2 else 3.0)
                        continue

                    if cmd == "tau":                          # 7 轴扭矩速读（T_CLAMP 标定用）
                        self.print_torques(float(parts[1]) if len(parts) > 1 else 0.0)
                        continue

                    if cmd == "pose":                         # 整臂姿态
                        if len(parts) != 7:
                            print("[!] 格式: pose <J1> <J2> <J3> <J4> <J5> <J6>（共 6 个角度）")
                            continue
                        self.move_pose([float(v) for v in parts[1:7]])
                        continue

                    if len(parts) != 2:                       # 默认：单轴运动 <关节> <角度>
                        print("[!] 格式错误！可用: <关节 1-7> <角度> | pose 六角度 | off/on/test <关节>")
                        continue
                    jid, t_deg = int(parts[0]), float(parts[1])
                    if jid not in range(1, 8):
                        print("[!] 关节编号错误，可选范围为 1 ~ 7")
                        continue
                    cfg = _cfg(jid)
                    print(f"\n>>> 选中关节: {cfg['name']} | 允许安全范围: [{cfg['min_deg']:.1f}°, {cfg['max_deg']:.1f}°]")
                    self.move_joint(jid, t_deg)

                except ValueError:
                    print("[!] 输入非法：请输入有效数字！")
                except KeyboardInterrupt:
                    print("\n[!] 收到 Ctrl+C 中断信号！")
                    break
                except Exception as e:
                    # 通信丢失等任何异常：中止本条指令但不退出——退出必须走 q（安全卸力）。
                    # 若刚才卸力过，该轴仍处于卸力状态（安全），通信恢复后重试即可。
                    print(f"\n[!] 指令执行异常（通信丢失?）：{e}")
                    print("    已中止该指令。恢复后可重新输入（输 q 安全退出并卸力）。")
        finally:
            self.shutdown()      # q / Ctrl+C / 任何漏网异常：逐轴卸力必被执行

    def shutdown(self):
        """逐轴卸力并关闭 CAN 总线（退出前必须走这里；直接点终端 × 会跳过）。"""
        print("\n[*] 正在切断各轴力矩输出并安全关闭 CAN 物理总线...")
        for m in self.motors.values():
            try:
                m.disable()
            except Exception:
                pass
        if self.ctrl:
            self.ctrl.close()
        print("[+] 系统已安全卸力退出。")


if __name__ == "__main__":
    app = CliArmController()
    app.run_repl()
