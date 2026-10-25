# -*- coding: utf-8 -*-
"""
PC 权威端桥接客户端（P5-D 参考实现 / 测试工具）
=================================================
连接伴生服务 WS 桥（role=pc，需 token），功能：
  1. 接收网页遥操作页下发的 input（打印 + 频率统计）——将来转发给 UDP 9003 链路
  2. 周期发 telemetry（模拟 6 关节角）——供监控页 ECharts 实时流验证（P5-E）
  3. --quiet 只统计不打印

用法：
  mujoco_sim/.rebot/Scripts/python.exe tools/pc_bridge_client.py --token <TOKEN>
  （TOKEN 见伴生服务启动打印，或环境变量 COMPANION_TOKEN 固定）

将来真机接入：把本文件的 telemetry 发送替换为 UDP 9004 遥测读取、input 接收
回调里转发 UDP 9003 控制下发（接口与队长《技术路线by罗.md》对齐）。
"""
import argparse
import asyncio
import json
import time

import websockets

WS_URL = "ws://127.0.0.1:8800/ws"
INPUT_WATCHDOG_S = 0.2   # >200ms 无 input → 下发零指令/停机（真机语义：失联即停）
RECONNECT_MAX_S = 8.0    # 指数退避重连上限


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--token", required=True, help="伴生服务桥 token（启动打印或 COMPANION_TOKEN）")
    ap.add_argument("--url", default=WS_URL)
    ap.add_argument("--no-telemetry", action="store_true", help="不发模拟遥测（只收 input）")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    n_input = 0
    t0 = time.time()
    last_report = t0
    last_input_ts = time.time()  # 看门狗基准
    stale_warned = False
    session = 0

    # 发现 7：schema 校验 + clamp（权威端最后一道闸——server 不校验 payload，真机前必须拦）
    def clamp_input(payload):
        if not isinstance(payload, dict):
            return None
        out = {}
        for k in ("lStick", "rStick"):
            v = payload.get(k)
            if not isinstance(v, dict) or "x" not in v or "y" not in v:
                return None
            try:
                out[k] = {"x": max(-1.0, min(1.0, float(v["x"]))), "y": max(-1.0, min(1.0, float(v["y"])))}
            except (TypeError, ValueError):
                return None
        for k in ("lGrip", "lX", "lY", "rA", "rB"):
            out[k] = bool(payload.get(k))
        out["armed"] = bool(payload.get("armed"))
        out["estop"] = bool(payload.get("estop"))
        out["seq"] = payload.get("seq")
        return out

    while True:  # 发现 3：断开自动重连（指数退避 2s→4s→8s…上限 8s）
        session += 1
        try:
            async with websockets.connect(args.url) as ws:
                await ws.send(json.dumps({"type": "hello", "role": "pc", "token": args.token}))
                welcome = json.loads(await ws.recv())
                if welcome.get("type") != "welcome":
                    print("连接被拒：", welcome)
                    return
                print(f"[pc-bridge] 会话#{session} 已连桥（clients={welcome.get('clients')}）——等待 input…")

                async def telemetry_loop():
                    if args.no_telemetry:
                        return
                    while True:
                        await ws.send(json.dumps({"type": "telemetry", "payload": {
                            "j": [round(20 * ((time.time() * 0.5 + i * 0.3) % 2 - 1), 2) for i in range(6)],
                        }}))
                        await asyncio.sleep(0.2)  # 5Hz 模拟遥测

                tele_task = asyncio.create_task(telemetry_loop())
                try:
                    while True:
                        # 看门狗修复：recv 带 50ms 超时驱动主循环——桥静默时（网页未 armed，
                        # 最常见状态）recv 永久阻塞，旧写法把看门狗/统计嵌在 async for 里
                        # 永远不可达。现在无论有没有消息，每 50ms 都走一次看门狗/统计。
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=0.05)
                        except asyncio.TimeoutError:
                            raw = None
                        if raw is not None:
                            try:
                                msg = json.loads(raw)
                            except (json.JSONDecodeError, TypeError, ValueError):
                                continue
                            if msg.get("type") == "state" and msg.get("src") == "input":
                                # 服务端 join 时会补发最近一帧缓存 input（最长 600s 前，
                                # 为遥操页预热设计）——陈旧帧不能当实时指令（真机前必须过滤）
                                ts = msg.get("ts")
                                if isinstance(ts, (int, float)) and time.time() - ts > 1.0:
                                    print("[!] 丢弃补发的陈旧 input（ts 距今 >1s）")
                                    continue
                                n_input += 1
                                last_input_ts = time.time()
                                payload = clamp_input(msg.get("payload", {}))
                                if payload is None:
                                    print("[!] input schema 异常，已丢弃")
                                    continue
                                # ↓↓↓ 真机接入点：clamp 后的 payload 转发 UDP 9003 ↓↓↓
                                if not args.quiet and payload.get("armed"):
                                    ls, rs = payload["lStick"], payload["rStick"]
                                    tag = "[E-STOP] " if payload["estop"] else ("" if payload["armed"] else "[终结帧] ")
                                    print(f"input  L=({ls['x']:.1f},{ls['y']:.1f}) R=({rs['x']:.1f},{rs['y']:.1f})"
                                          f" grip={payload['lGrip']} A={payload['rA']} B={payload['rB']} {tag}")
                        now = time.time()
                        # 输入看门狗——>200ms 无 input 视为失联，下发零指令（真机语义：失联即停）
                        if now - last_input_ts > INPUT_WATCHDOG_S and not stale_warned:
                            print(f"[看门狗] >{INPUT_WATCHDOG_S}s 无 input → 下发零指令/停机（真机接入点）")
                            stale_warned = True
                        if now - last_input_ts <= INPUT_WATCHDOG_S:
                            stale_warned = False
                        if now - last_report >= 5:
                            print(f"[统计] {n_input} 条 input，{n_input / (now - t0):.1f} Hz 平均")
                            last_report = now
                except websockets.exceptions.ConnectionClosed as e:
                    if e.code in (4001, 4002, 4003):
                        # 鉴权/角色被拒是配置问题：重连没有意义（服务端不发拒绝消息、直接关）
                        print(f"[pc-bridge] 桥拒绝连接 code={e.code}（token/角色不对）——"
                              f"检查 --token 或 COMPANION_TOKEN 后重试，退出")
                        return
                    print(f"[pc-bridge] 会话#{session} 桥关闭 code={e.code}")
                finally:
                    tele_task.cancel()
                    try:
                        await tele_task
                    except asyncio.CancelledError:
                        pass
        except websockets.exceptions.ConnectionClosed as e:
            # 握手阶段（hello/welcome）被关：服务端鉴权失败只关连接不发消息，
            # 4001~4003 是配置问题，重连无意义
            if e.code in (4001, 4002, 4003):
                print(f"[pc-bridge] 桥拒绝连接 code={e.code}（token/角色不对）——"
                      f"检查 --token 或 COMPANION_TOKEN 后重试，退出")
                return
            print(f"[pc-bridge] 会话#{session} 握手阶段连接关闭 code={e.code}")
        except OSError as e:
            print(f"[pc-bridge] 会话#{session} 连接失败：{e}")
        backoff = min(2 ** session, RECONNECT_MAX_S)
        print(f"[pc-bridge] {backoff:.0f}s 后重连…")
        await asyncio.sleep(backoff)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[pc-bridge] 退出")
