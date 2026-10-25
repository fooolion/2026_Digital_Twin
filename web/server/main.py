# -*- coding: utf-8 -*-
"""
P5-A 伴生服务骨架（FastAPI · 2026-10-25）
==========================================
比赛 PC 本机运行的网页伴生服务，与前端同源（跨域/混合内容问题消失，教室断网可用）。

职责（骨架范围）：
  1. 静态托管 web/dist（先 npm run build）
  2. GET /api/health    健康检查（前端 companion.ts 探测此端点切换"实时/演示"双模）
  3. WS  /ws            桥接 hub（P5-B~E 的地基）：
       - 客户端角色：role=web（浏览器页）/ pc（PC 权威端，将来接 UDP 链路）
       - 协议（JSON）：{type, role?, payload?, ts?}
           web/pc → server: hello | input(遥操作输入) | telemetry(遥测)
           server → 客户端: welcome | state(最新状态广播，新连接即补发)
       - 内存保存最近一次 state（新客户端连上立刻拿到，监控页 P5-E 用）

运行：
  mujoco_sim/.rebot/Scripts/python.exe web/server/main.py        # http://0.0.0.0:8800
  （依赖：fastapi + uvicorn[standard]，已装 .rebot venv）

P5-B~E 扩展点（本骨架不动）：
  - 双摄像头 MJPEG：加 /cam/{n}.mjpg 端点（cv2 循环编码 JPEG 流）
  - PICO 大屏视频：OBS→MediaMTX→WebRTC，网页端直连 MediaMTX，不经本服务
  - PC 权威端接入：pc 角色客户端桥接 UDP 9003/9004
"""
import asyncio
import json
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

HOST, PORT = "0.0.0.0", 8800
DIST = Path(__file__).resolve().parent.parent / "dist"

app = FastAPI(title="伴生服务 · 报废电池拆解数字孪生", version="0.1.0")
# CORS：dev 模式下前端跑在 5173，探测/连接 8800 为跨源——本机演示服务放开无风险
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/api/health")
async def health() -> dict:
    return {
        "ok": True,
        "service": "companion",
        "version": "0.1.0",
        "uptime_s": round(time.time() - _T0, 1),
        "bridge_clients": len(_HUB.members),
        "dist": DIST.is_dir(),
    }


_T0 = time.time()


class Hub:
    """WS 桥：成员管理 + 最新状态缓存 + 广播。单进程内存态（骨架级，够用）。"""

    def __init__(self) -> None:
        self.members: dict[WebSocket, dict] = {}  # ws -> {role, joined_ts}
        self.latest: dict | None = None           # 最近一次 state 消息
        self._lock = asyncio.Lock()

    async def broadcast(self, message: dict, exclude: WebSocket | None = None) -> None:
        dead = []
        for ws in list(self.members):
            if ws is exclude:
                continue
            try:
                await ws.send_json(message)
            except Exception:  # noqa: BLE001 - 发送失败视为掉线，收尾清理
                dead.append(ws)
        for ws in dead:
            await self.leave(ws)

    async def join(self, ws: WebSocket, role: str) -> None:
        async with self._lock:
            self.members[ws] = {"role": role, "joined_ts": time.time()}
        await ws.send_json({"type": "welcome", "role": role, "clients": len(self.members)})
        if self.latest is not None:
            await ws.send_json(self.latest)  # 新客户端补发最近状态

    async def leave(self, ws: WebSocket) -> None:
        async with self._lock:
            self.members.pop(ws, None)

    async def handle(self, ws: WebSocket, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except (ValueError, TypeError):
            await ws.send_json({"type": "error", "detail": "invalid json"})
            return
        mtype = msg.get("type")
        if mtype in ("input", "telemetry"):
            # 桥核心语义：input/telemetry 即时广播给其他端，并缓存为最新 state
            state = {"type": "state", "src": mtype, "payload": msg.get("payload"), "ts": time.time()}
            self.latest = state
            await self.broadcast(state, exclude=ws)
        elif mtype == "ping":
            await ws.send_json({"type": "pong", "ts": time.time()})
        else:
            await ws.send_json({"type": "error", "detail": f"unknown type: {mtype}"})


_HUB = Hub()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    role = "web"
    try:
        # 首条消息应为 hello（带 role）；超时未说 hello 按默认 web 处理
        try:
            first = json.loads(await asyncio.wait_for(ws.receive_text(), timeout=5))
            if first.get("type") == "hello":
                role = str(first.get("role") or "web")
        except (asyncio.TimeoutError, ValueError):
            pass
        await _HUB.join(ws, role)
        while True:
            raw = await ws.receive_text()
            await _HUB.handle(ws, raw)
    except WebSocketDisconnect:
        pass
    finally:
        await _HUB.leave(ws)


# 静态托管必须最后挂载（否则吞掉 /api 与 /ws 路由）。
# 挂两路：带 /2026_Digital_Twin/ 前缀（dist 产物因 GitHub Pages base 而引用此前缀）
# 与根路径（局域网设备直开 http://本机IP:8800/ 即可访问）。
if DIST.is_dir():
    app.mount("/2026_Digital_Twin", StaticFiles(directory=str(DIST), html=True), name="dist-prefixed")
    app.mount("/", StaticFiles(directory=str(DIST), html=True), name="dist")
else:
    @app.get("/")
    async def no_dist() -> dict:
        return {"ok": False, "hint": "web/dist 不存在——先 cd web && npm run build"}


if __name__ == "__main__":
    print(f"[companion] http://{HOST}:{PORT}  (dist={'ok' if DIST.is_dir() else '缺失，请先 build'})")
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
