#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 安全/夹紧判断服务（消费实机遥测，产出面板数据）。

数据流
  test_force.py（实机遥测, UDP 9004）
      → 本服务（姿态复现进 MuJoCo → 安全判断 + 夹爪夹紧判断）
          → mujoco_sim/output/safety_state.json（原子覆写，每包刷新）
          → UDP 9005 广播同一包（郑瑞洋综合面板二选一消费）

判断逻辑
  1. 姿态复现：实机关节角(rad, 符号映射 SIGNS) 写入 MuJoCo qpos → mj_forward
     → qfrc_bias = 该姿态下模型预期的重力/惯性力矩
  2. 过载判定：|实测扭矩| > LIMITS[i]（额定力矩的 50%）→ DANGER
  3. 异常力判定：|实测扭矩 − 模型预期| > MARGINS[i] → WARNING（外部接触/卡阻嫌疑）
  4. 夹紧判定：|夹爪电机扭矩| > T_CLAMP(0.8) → 已夹紧（2026-10-02 真机标定：空闭合 0.23 / 夹 103g 达 1.40）
  5. 断流判定：>1s 没收到遥测 → level=NO_DATA（面板显示断连）

输出契约（safety_state.json / UDP 9005，同一个 JSON 包）
  {"timestamp": float,          # 秒
   "linked": bool,              # 9004 遥测是否存活
   "level": "OK|WARNING|DANGER|NO_DATA",
   "safe": bool,                # level 为 OK/WARNING（NO_DATA/DANGER 时 False）
   "warnings": ["J3: ...", ...],
   "joints": [{"name":"J1","tau_real":N·m,"tau_model":N·m,"over_limit":bool,"anomaly":bool} ×6],
   "gripper": {"tau_real": N·m, "clamped": bool}}
  注：夹紧判定命中(clamped=True)时 level 会升为 WARNING——属设计行为（夹紧提示），非故障。

面板口径提示：快速手扳时 |实测−模型预期| 会瞬时超过 MARGINS → WARNING 闪现属**设计行为**
（异常力判定的本意就是提示接触/卡阻），面板建议"持续 >0.5s 才升级为醒目告警"。

用法
  # 正式（先启动 test_force.py；失能监听模式，默认不做"实测vs预期"偏差判定）
  mujoco_sim/.rebot/Scripts/python.exe tools/safety_monitor.py
  # 电机使能闭环联调时（tau_act 有真实输出力矩）加 --anomaly 打开偏差判定
  mujoco_sim/.rebot/Scripts/python.exe tools/safety_monitor.py --anomaly
  # 无硬件自检（本机 9104 端口合成数据跑通全链路，自动断言 PASS/FAIL）
  mujoco_sim/.rebot/Scripts/python.exe tools/safety_monitor.py --selftest

注意：本服务不碰硬件（不需要 motorbridge），任何机器可跑；
      需要硬件接入时改 test_force.py（队长的文件）。
"""
import argparse
import json
import os
import socket
import sys
import threading
import time

import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 注意：MuJoCo 的 C 库在 Windows 上打不开含非 ASCII 的绝对路径，
# 所以 chdir 到项目根后用相对路径加载（OUT_JSON 用绝对路径，Python open 无此问题）。
os.chdir(ROOT)
MJCF = os.path.join('mujoco_sim', 'models', 'ReBot_Arm_DM.xml')
OUT_JSON = os.path.join(ROOT, 'mujoco_sim', 'output', 'safety_state.json')

J = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

# ---- 可调参数 ----
SIGNS = [1.0] * 6          # 实机关节角(rad) → MuJoCo 关节角的符号映射（方向已单轴验证 = 全 +1；变了改这里）
TAU_SIGNS = [1.0] * 6      # 实机扭矩 → MuJoCo 力矩方向的符号映射（夹紧/过载按幅值判定，此映射只影响正负号显示）
LIMITS = [13.5, 13.5, 13.5, 3.5, 3.5, 3.5]   # 过载线：4340P/4310 额定力矩(27/7)的 50%
MARGINS = [3.0, 3.0, 3.0, 1.5, 1.5, 1.5]     # 异常力线：|实测 − 模型预期| 超此值 → 接触/卡阻
T_CLAMP = 0.8              # 夹紧线：|夹爪扭矩| > 此值 → 已夹紧。2026-10-02 真机实测标定：
                           # 空闭合保持 +0.23、夹 103g 已达 +1.40 → 阈值取两者中点 0.8
STALE_SEC = 1.0            # 遥测断流判定


class Judger:
    """加载一次 MuJoCo 模型，对每包遥测做姿态复现 + 三类判定。"""

    def __init__(self, check_anomaly=True):
        self.check_anomaly = check_anomaly
        self.m = mujoco.MjModel.from_xml_path(MJCF)
        self.d = mujoco.MjData(self.m)
        self.adr = [self.m.joint(n).qposadr[0] for n in J]
        self.dof = [self.m.joint(n).dofadr[0] for n in J]

    def judge(self, tel):
        """对一包遥测做姿态复现 + 过载/异常力/夹紧三类判定，返回输出契约 JSON 包。

        tel: 遥测 dict（test_force.py 的 UDP 包），q_act/gripper_pos 单位 rad，tau_act/gripper_tau 单位 N·m。
        """
        d = self.d
        d.qpos[:] = 0
        for i in range(6):
            d.qpos[self.adr[i]] = SIGNS[i] * float(tel['q_act'][i])
        mujoco.mj_forward(self.m, d)
        bias = [TAU_SIGNS[i] * float(d.qfrc_bias[self.dof[i]]) for i in range(6)]

        joints, warnings, danger = [], [], False
        for i in range(6):
            tr = TAU_SIGNS[i] * float(tel['tau_act'][i])
            over = abs(tr) > LIMITS[i]
            # 失能监听模式下电机不出力、tau≈0（重力被摩擦顶住），实测 vs 预期的偏差是正常
            # 物理现象——该判定只在电机使能（有输出力矩）时才有意义，用 --no-anomaly 关掉
            anomaly = self.check_anomaly and abs(tr - bias[i]) > MARGINS[i]
            if over:
                danger = True
                warnings.append(f'{NAMES[i]} 扭矩 {tr:+.2f} 超限值 {LIMITS[i]:.1f} N·m')
            elif anomaly:
                warnings.append(f'{NAMES[i]} 实测 {tr:+.2f} 偏离模型预期 {bias[i]:+.2f} 超 {MARGINS[i]:.1f} N·m')
            joints.append(dict(name=NAMES[i], tau_real=round(tr, 3), tau_model=round(bias[i], 3),
                               over_limit=over, anomaly=anomaly))

        gt = float(tel.get('gripper_tau', 0.0))
        clamped = abs(gt) > T_CLAMP
        if clamped:
            warnings.append(f'夹爪扭矩 {gt:+.2f} 超夹紧阈值 {T_CLAMP:.2f} N·m → 已夹紧')

        level = 'DANGER' if danger else ('WARNING' if warnings else 'OK')
        return {'timestamp': float(tel.get('timestamp', time.time())),
                'linked': True,
                'level': level,
                'safe': not danger,
                'warnings': warnings,
                'joints': joints,
                'gripper': {'tau_real': round(gt, 3), 'clamped': clamped}}


def write_out(pkg):
    """原子写结果文件。目标被占用（编辑器开着/杀毒扫描）时重试 3 次，仍失败则跳过本次
    并返回 False——安全服务不能因为文件被占用而死。"""
    tmp = OUT_JSON + '.%d.tmp' % os.getpid()
    text = json.dumps(pkg)
    last = None
    for _ in range(3):
        try:
            with open(tmp, 'w') as f:
                f.write(text)
            os.replace(tmp, OUT_JSON)
            return True
        except PermissionError as e:
            last = e
            time.sleep(0.05)
    print('\n[!] 写 safety_state.json 被占用（编辑器开着它？杀毒在扫？），跳过本次: %s' % last)
    return False


def no_data_pkg():
    joints = [dict(name=n, tau_real=0.0, tau_model=0.0, over_limit=False, anomaly=False)
              for n in NAMES]                                # 契约恒为 ×6，面板可安全索引
    return {'timestamp': round(time.time(), 3), 'linked': False, 'level': 'NO_DATA', 'safe': False,
            'warnings': ['遥测断流 >%.0fs' % STALE_SEC], 'joints': joints,
            'gripper': {'tau_real': 0.0, 'clamped': False}}


def serve(rx, judger, tx, pub_port, deadline=None):
    """主循环：收一包判一包发一包。deadline 不为 None 时到点返回（自检用）。"""
    last_pkg_time = 0.0
    warned_stale = False
    seen_levels = []
    t_start = time.time()
    while True:
        now = time.time()
        if deadline is not None and now - t_start > deadline:
            break
        got = False
        try:
            data, _ = rx.recvfrom(65536)
            got = True
        except socket.timeout:
            pass
        if got:
            try:
                tel = json.loads(data)
                pkg = judger.judge(tel)
            except (ValueError, KeyError, IndexError, TypeError):
                continue                                     # 坏包/缺字段：丢包保命，沿用上一包状态
            last_pkg_time = now
            warned_stale = False
        else:
            if now - last_pkg_time > STALE_SEC:
                pkg = no_data_pkg()
                if not warned_stale:
                    print('[!] 遥测断流 >%.0fs' % STALE_SEC)
                    warned_stale = True
            else:
                continue
        seen_levels.append(pkg['level'])
        try:
            write_out(pkg)
            tx.sendto(json.dumps(pkg).encode('utf-8'), ('127.0.0.1', pub_port))
        except OSError as e:
            print('\n[!] 输出失败（继续运行）: %s' % e)
        print('\r[%s] %s' % (pkg['level'], '; '.join(pkg['warnings'])[:90]), end='', flush=True)
    return seen_levels


def main():
    """解析命令行参数并启动判断主循环（正式监听 9004，或 --selftest 合成数据自检）。"""
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=9004, help='遥测监听端口（test_force.py 的发送口）')
    ap.add_argument('--pub', type=int, default=9005, help='判断结果 UDP 广播端口（面板消费）')
    ap.add_argument('--selftest', action='store_true', help='无硬件自检：合成数据跑通全链路（监听 9104）')
    ap.add_argument('--anomaly', action='store_true',
                    help='打开"实测偏离模型预期"判定（电机使能闭环联调时用；失能监听模式下 tau≈0 属正常'
                         '物理现象，此判定会常驻误报，故默认关闭）')
    a = ap.parse_args()

    judger = Judger(check_anomaly=a.anomaly)
    port = 9104 if a.selftest else a.port
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(('127.0.0.1', port))
    rx.settimeout(0.3)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if a.selftest:
        global OUT_JSON
        # 自检写独立文件：避免与正在运行的正式实例抢写同一个 safety_state.json
        OUT_JSON = OUT_JSON.replace('safety_state.json', 'safety_state_selftest.json')

        def feeder():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            time.sleep(0.4)
            ok_pkt = json.dumps({'timestamp': time.time(),
                                 'q_act': [0.0, -0.873, -0.698, 0.0, 0.0, 0.0],
                                 'v_act': [0.0] * 6,
                                 'tau_act': [0.0, 2.2, -7.6, -2.1, 0.0, 0.0],
                                 'gripper_pos': 0.0, 'gripper_tau': 0.05}).encode()
            bad_pkt = json.dumps({'timestamp': time.time(),
                                  'q_act': [0.0, -0.873, -0.698, 0.0, 0.0, 0.0],
                                  'v_act': [0.0] * 6,
                                  'tau_act': [0.0, 2.2, -30.0, -2.1, 0.0, 0.0],
                                  'gripper_pos': 0.3, 'gripper_tau': 1.0}).encode()
            s.sendto(ok_pkt, ('127.0.0.1', port))
            time.sleep(0.3)
            s.sendto(b'garbage-not-json', ('127.0.0.1', port))   # 坏包免疫自检：不应崩服务
            time.sleep(0.2)
            s.sendto(bad_pkt, ('127.0.0.1', port))
        threading.Thread(target=feeder, daemon=True).start()
        # 截止 1.5s：最后一包在 0.9s，断流判定在 1.9s——慢机器留余量又不被断流覆盖
        seen = serve(rx, judger, tx, a.pub, deadline=1.5)
        pkg = json.load(open(OUT_JSON, encoding='utf-8'))
        ok = ('OK' in seen and pkg['level'] == 'DANGER'
              and pkg['gripper']['clamped'] and not pkg['safe'])
        print('\n自检%s: 级别序列=%s, 最终=%s, 夹紧=%s'
              % ('通过 ✅' if ok else '失败 ❌', seen, pkg['level'], pkg['gripper']['clamped']))
        sys.exit(0 if ok else 1)

    print(f'[+] 安全判断服务: 监听 UDP {port} → 判断 → {OUT_JSON} + UDP {a.pub}。Ctrl+C 退出。')
    try:
        serve(rx, judger, tx, a.pub)
    except KeyboardInterrupt:
        print('\n[*] 退出。')
    finally:
        rx.close()
        tx.close()


if __name__ == '__main__':
    main()
