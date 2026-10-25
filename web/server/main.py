# -*- coding: utf-8 -*-
"""
P5-A 伴生服务骨架（FastAPI · 2026-10-25）
==========================================
比赛 PC 本机运行的网页伴生服务，与前端同源（跨域/混合内容问题消失，教室断网可用）。

职责（骨架范围）：
  1. 静态托管 web/dist（先 npm run build）
  2. GET /api/health    健康检查（前端 companion.ts 探测此端点切换"实时/演示"双模）
  2b. GET /cam/{0,1}.mjpg  双摄像头 MJPEG 流（P5-B；懒启动+共享编码+离线占位帧）
      GET /api/cams        摄像头在线状态
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
  - PICO 大屏视频：OBS→MediaMTX→WebRTC，网页端直连 MediaMTX，不经本服务
  - PC 权威端接入：pc 角色客户端桥接 UDP 9003/9004
"""
import asyncio
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

HOST, PORT = "0.0.0.0", 8800
DIST = Path(__file__).resolve().parent.parent / "dist"

# 摄像头映射（P5-B）：slot 0/1（网页端相机 A/B）→ cv2 设备索引。
# 本机实测：工业相机 = 1（侧墙）/ 2（后墙）；笔记本内置 0 仅测试用。
# 相机编号是"本机相关"的（换机器重新确认）；环境变量 COMPANION_CAMS="1,2" 可覆盖。
CAM_IDS = [int(x) for x in os.environ.get("COMPANION_CAMS", "1,2").split(",")]
CAM_FPS = 15          # MJPEG 帧率上限（1080p JPEG q70 下单流约 1-2 MB/s，局域网够用）
CAM_JPEG_QUALITY = 70

app = FastAPI(title="伴生服务 · 报废电池拆解数字孪生", version="0.1.0", docs_url=None, redoc_url=None, openapi_url=None)
# CORS：dev 模式下前端跑在 5173，探测/连接 8800 为跨源——本机演示服务放开无风险
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


_T0 = time.time()


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


class Hub:
    """WS 桥：成员管理 + 最新状态缓存 + 广播。单进程内存态（骨架级，够用）。"""

    def __init__(self) -> None:
        self.members: dict[WebSocket, dict] = {}  # ws -> {role, joined_ts}
        self.latest: dict | None = None           # 最近一次 state 消息
        self._lock = asyncio.Lock()

    async def broadcast(self, message: dict, exclude: WebSocket | None = None) -> None:
        # 逐客户端带超时并发发送：慢/死客户端不拖累广播循环（裸 await 会被一个卡住的
        # TCP 连接串行化整条广播——P5-E 接实时流后高频率下发会放大这个问题）
        async def _send(ws: WebSocket) -> None:
            try:
                await asyncio.wait_for(ws.send_json(message), timeout=3)
            except Exception:  # noqa: BLE001 - 超时/断开视为掉线
                await self.leave(ws)
                try:
                    await asyncio.wait_for(ws.close(), 1)  # m5：不发 close 帧，对端会误以为仍在线
                except Exception:  # noqa: BLE001
                    pass
        await asyncio.gather(*(_send(ws) for ws in list(self.members) if ws is not exclude))

    async def join(self, ws: WebSocket, role: str) -> None:
        async with self._lock:
            self.members[ws] = {"role": role, "joined_ts": time.time()}
        await ws.send_json({"type": "welcome", "role": role, "clients": len(self.members)})
        if self.latest is not None and time.time() - self.latest.get("ts", 0) < 600:
            await ws.send_json(self.latest)  # 补发最近状态（>10 分钟视为过期，不发）

    async def leave(self, ws: WebSocket) -> None:
        async with self._lock:
            self.members.pop(ws, None)

    async def handle(self, ws: WebSocket, raw: str) -> None:
        if len(raw) > 262_144:  # m4：256KB 上限（uvicorn 单帧默认 16MB，防 latest 缓存/广播放大）
            await ws.send_json({"type": "error", "detail": "payload too large"})
            return
        try:
            msg = json.loads(raw)
        except (ValueError, TypeError, RecursionError):
            await ws.send_json({"type": "error", "detail": "invalid json"})
            return
        if not isinstance(msg, dict):  # m1：'[1]'/'"x"'/'5' 合法 JSON 但非对象，.get 会 AttributeError
            await ws.send_json({"type": "error", "detail": "not an object"})
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


def _LAN_IP() -> str | None:
    """本机局域网地址（Origin 校验用：从本机任一网卡地址集合判定）。"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # 不发包，仅取路由源地址
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return None


class CamStream:
    """单相机共享读取器：后台 task 读帧+编码一次，多连接共享同一份 JPEG bytes。
    读不到帧（未接/被占）→ 生成占位帧（灰底文字），流不断。懒启动（首个请求才开相机）。"""

    def __init__(self, slot: int, device: int) -> None:
        self.slot, self.device = slot, device
        self.jpeg: bytes | None = None
        self.online = False
        self.event = asyncio.Event()
        self._task: asyncio.Task | None = None
        self._lock = asyncio.Lock()

    def _placeholder(self) -> bytes:
        frame = np.full((360, 640, 3), 14, dtype=np.uint8)
        cv2.putText(frame, f"Camera {chr(65 + self.slot)} offline (cv2:{self.device})",
                    (70, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (90, 160, 210), 2)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        return buf.tobytes() if ok else b""

    async def _run(self) -> None:
        interval = 1.0 / CAM_FPS
        loop = asyncio.get_running_loop()
        cap = await loop.run_in_executor(None, lambda: cv2.VideoCapture(self.device, cv2.CAP_DSHOW))
        try:
            while True:
                t0 = time.monotonic()
                ok, frame = await loop.run_in_executor(None, cap.read)
                if ok:
                    self.online = True
                    encoded, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, CAM_JPEG_QUALITY])
                    if encoded:
                        self.jpeg = buf.tobytes()
                else:
                    self.online = False
                    self.jpeg = self._placeholder()
                self.event.set()
                self.event = asyncio.Event()  # 新一帧唤醒所有等待者
                await asyncio.sleep(max(0, interval - (time.monotonic() - t0)))
        finally:
            await loop.run_in_executor(None, cap.release)

    async def ensure_started(self) -> None:
        async with self._lock:
            if self._task is None or self._task.done():
                self._task = asyncio.create_task(self._run())

    async def frames(self):
        """MJPEG 生成器：等新帧 → yield（多连接共享同一份 jpeg，编码每帧只做一次）。"""
        await self.ensure_started()
        while True:
            if self.jpeg:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(self.jpeg)).encode() + b"\r\n\r\n" + self.jpeg + b"\r\n"
                )
            await self.event.wait()


_CAMS = [CamStream(slot, dev) for slot, dev in enumerate(CAM_IDS)]


@app.get("/api/cams")
async def cams_status() -> dict:
    return {"cams": [{"slot": c.slot, "device": c.device, "online": c.online} for c in _CAMS]}


@app.get("/cam/{slot}.mjpg")
async def cam_mjpg(slot: int):
    from fastapi.responses import StreamingResponse
    if slot >= len(_CAMS):
        return {"ok": False, "detail": "slot out of range"}
    return StreamingResponse(
        _CAMS[slot].frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    # m2：WS 不受 CORS 中间件管辖——跨站网页可发起 ws://127.0.0.1:8800 注入。
    # Origin 校验：为空（同源 fetch/非浏览器客户端）或主机属于本机（127.0.0.1/localhost/局域网 IP）才放行。
    origin = ws.headers.get("origin", "")
    if origin:
        from urllib.parse import urlparse
        if urlparse(origin).hostname not in ("127.0.0.1", "localhost", _LAN_IP()):
            await ws.close(code=4003)
            return
    await ws.accept()
    role = "web"
    try:
        # 首条消息应为 hello（带 role 白名单）；超时未说 hello 按默认 web 处理
        try:
            first = json.loads(await asyncio.wait_for(ws.receive_text(), timeout=5))
            if not isinstance(first, dict):
                first = {}
            if first.get("type") == "hello":
                role = str(first.get("role") or "web")
        except (asyncio.TimeoutError, ValueError):
            pass
        if role not in ("web", "pc"):  # role 白名单（P5-D 前防自报冒充权威端）
            await ws.close(code=4002)
            return
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
