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

    async with websockets.connect(args.url) as ws:
        await ws.send(json.dumps({"type": "hello", "role": "pc", "token": args.token}))
        welcome = json.loads(await ws.recv())
        if welcome.get("type") != "welcome":
            print("连接被拒：", welcome)
            return
        print(f"[pc-bridge] 已连桥（clients={welcome.get('clients')}）——等待 input…")

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
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get("type") == "state" and msg.get("src") == "input":
                    n_input += 1
                    if not args.quiet:
                        p = msg.get("payload", {})
                        print(f"input  lStick=({p.get('lStick', {}).get('x', 0):.1f},{p.get('lStick', {}).get('y', 0):.1f})"
                              f" rStick=({p.get('rStick', {}).get('x', 0):.1f},{p.get('rStick', {}).get('y', 0):.1f})"
                              f" grip={p.get('lGrip')} A={p.get('rA')} B={p.get('rB')}")
                now = time.time()
                if now - last_report >= 5:
                    print(f"[统计] {n_input} 条 input，{n_input / (now - t0):.1f} Hz 平均")
                    last_report = now
        finally:
            tele_task.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[pc-bridge] 退出")
