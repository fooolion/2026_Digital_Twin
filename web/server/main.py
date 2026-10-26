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

# 桥接鉴权 token（P5-D）：客户端 hello 需带 token（web 页从 /api/health 拿不到——见下）。
# 设计：token 由环境变量 COMPANION_TOKEN 指定（比赛 PC 固定配置）；未设置时自动生成并
# 打印到控制台（本机操作员手动抄给 pc 客户端；web 页走同源无需 token 的降级路径）。
import secrets
TOKEN = os.environ.get("COMPANION_TOKEN") or secrets.token_hex(8)

# PICO 大屏（P5-C）：实时流走 MediaMTX 的 WHEP 端点（OBS 推流 → MediaMTX → 网页 WHEP 拉流，
# 不经本服务转发）。环境变量 PICO_WHEP_URL 配置后 /api/stream 下发；空 = 未部署实时链路。
# 例：PICO_WHEP_URL="http://192.168.1.100:8889/pico/whep"（MediaMTX WebRTC 默认端口 8889）
PICO_WHEP_URL = os.environ.get("PICO_WHEP_URL", "")
# 预录视频（调试期/老师认可的过渡形态）：约定文件 web/public/media/pico.mp4（build 后进 dist/media/）
PICO_VIDEO = DIST / "media" / "pico.mp4"

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
    """WS 桥：成员管理 + 最新状态缓存 + 广播。单进程内存态（骨架级，够用）。
    操作员互斥（P5-D 复查发现 5）：同一时刻仅一个客户端可下发 input——
    首个 armed=true 的 input 占坑（记 ws），闭锁帧（armed=false）或断开释放；
    他人占坑时后来的 input 被拒（error: operator busy）。"""

    def __init__(self) -> None:
        self.members: dict[WebSocket, dict] = {}  # ws -> {role, joined_ts}
        self.latest: dict | None = None           # 最近一次 state 消息
        self.operator: WebSocket | None = None    # 当前 ARMED 发送源（互斥槽）
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
        # 补发最近状态：仅 input（teleop 页预热语义）。telemetry 不补发——
        # 陈旧遥测会被监控页 feedReal 当真流首帧，锁死 realMode 成"空白假实时图"
        if self.latest is not None and self.latest.get("src") == "input"                 and time.time() - self.latest.get("ts", 0) < 600:
            await ws.send_json(self.latest)

    async def leave(self, ws: WebSocket) -> None:
        async with self._lock:
            self.members.pop(ws, None)
            if self.operator is ws:
                self.operator = None  # 断开释放操作员槽

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
            payload = msg.get("payload")
            if mtype == "input":
                # 操作员互斥：armed=true 占坑/续用；armed=false 释放；他人占坑时拒绝
                armed = bool(payload.get("armed")) if isinstance(payload, dict) else False
                if armed and self.operator not in (None, ws):
                    await ws.send_json({"type": "error", "detail": "operator busy"})
                    return
                if armed:
                    self.operator = ws
                elif self.operator is ws:
                    self.operator = None
            # 桥核心语义：input/telemetry 即时广播给其他端，并缓存为最新 state
            state = {"type": "state", "src": mtype, "payload": payload, "ts": time.time()}
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
    读不到帧（未接/被占）→ 生成占位帧（灰底文字），流不断。
    懒启动（首个请求才开相机）；空闲自回收（连接归零 30s → 停任务释放相机，下次请求重开）。"""

    IDLE_RECYCLE_S = 30.0

    def __init__(self, slot: int, device: int) -> None:
        self.slot, self.device = slot, device
        self.jpeg: bytes | None = None
        self.online = False
        self.frame_no = 0            # 帧序号：等待者比较序号（替代共享 Event，消除 set/替换竞态）
        self.clients = 0             # 活跃 MJPEG 连接数（空闲回收依据）
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
            idle_since: float | None = None
            while True:
                # 空闲自回收：无连接持续空闲 → 退出任务释放相机设备（下次请求懒启动重开）
                if self.clients == 0:
                    if idle_since is None:
                        idle_since = time.monotonic()
                    elif time.monotonic() - idle_since > self.IDLE_RECYCLE_S:
                        return  # finally 释放相机
                else:
                    idle_since = None
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
                self.frame_no += 1
                await asyncio.sleep(max(0, interval - (time.monotonic() - t0)))
        finally:
            await loop.run_in_executor(None, cap.release)
            self._task = None

    async def ensure_started(self) -> None:
        async with self._lock:
            if self._task is None or self._task.done():
                self._task = asyncio.create_task(self._run())

    async def frames(self):
        """MJPEG 生成器：连接计数 → 等新帧号 → yield（编码每帧一次，多连接共享）。
        ⚠ ensure_started 必须在 _lock 之外调用——asyncio.Lock 不可重入，
        持锁调用会同协程自锁死（本函数曾因此挂死全部 MJPEG 连接）。"""
        await self.ensure_started()
        async with self._lock:
            self.clients += 1
        seen = self.frame_no
        try:
            while True:
                if self.jpeg and self.frame_no != seen:
                    seen = self.frame_no
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(self.jpeg)).encode() + b"\r\n\r\n" + self.jpeg + b"\r\n"
                    )
                else:
                    await asyncio.sleep(1.0 / CAM_FPS)  # 无新帧：按帧率间隔轮询（15fps 下开销可忽略）
        finally:
            async with self._lock:
                self.clients -= 1


_CAMS = [CamStream(slot, dev) for slot, dev in enumerate(CAM_IDS)]


@app.get("/api/cams")
async def cams_status() -> dict:
    return {"cams": [{"slot": c.slot, "device": c.device, "online": c.online} for c in _CAMS]}


@app.get("/api/stream")
async def stream_config() -> dict:
    """PICO 大屏流配置：前端按 whep > video > 占位 三层降级。"""
    return {
        "whep": PICO_WHEP_URL,                       # 实时流（MediaMTX WHEP）；空=未部署
        "video": PICO_VIDEO.is_file(),               # 预录视频是否就位（dist/media/pico.mp4）
        "video_url": "/2026_Digital_Twin/media/pico.mp4" if PICO_VIDEO.is_file() else None,
    }


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
        # 首条消息应为 hello（带 role 白名单 + pc 角色必须带 token）；超时未说 hello 按默认 web 处理
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
        # pc 角色（权威端，可下发控制）必须带正确 token；web 角色只观察+发 input（低权限）
        if role == "pc" and str(first.get("token") or "") != TOKEN:
            await ws.close(code=4001)
            return
        await _HUB.join(ws, role)
        while True:
            raw = await ws.receive_text()
            await _HUB.handle(ws, raw)
    except WebSocketDisconnect:
        pass
    finally:
        await _HUB.leave(ws)


class CachedStaticFiles(StaticFiles):
    """分层 Cache-Control：带 hash 的 assets 一年 immutable；models 一天；
    HTML 不缓存（改版即生效）。原 StaticFiles 只发 ETag，21MB STL 几天后会静默全量重拉。"""

    async def get_response(self, path: str, scope):
        resp = await super().get_response(path, scope)
        if resp.status_code == 200:
            # Windows 下 StaticFiles 传来的 path 是反斜杠——统一正斜杠再前缀判断
            norm = path.replace("\\", "/")
            if norm.startswith("assets/"):
                resp.headers["Cache-Control"] = "public, max-age=31536000, immutable"
            elif norm.startswith("models/"):
                resp.headers["Cache-Control"] = "public, max-age=86400"
            elif norm.endswith(".html"):
                resp.headers["Cache-Control"] = "no-cache"
        return resp


# 静态托管必须最后挂载（否则吞掉 /api 与 /ws 路由）。
# 挂两路：带 /2026_Digital_Twin/ 前缀（dist 产物因 GitHub Pages base 而引用此前缀）
# 与根路径（局域网设备直开 http://本机IP:8800/ 即可访问）。
if DIST.is_dir():
    app.mount("/2026_Digital_Twin", CachedStaticFiles(directory=str(DIST), html=True), name="dist-prefixed")
    app.mount("/", CachedStaticFiles(directory=str(DIST), html=True), name="dist")
else:
    @app.get("/")
    async def no_dist() -> dict:
        return {"ok": False, "hint": "web/dist 不存在——先 cd web && npm run build"}


if __name__ == "__main__":
    print(f"[companion] http://{HOST}:{PORT}  (dist={'ok' if DIST.is_dir() else '缺失，请先 build'})")
    print(f"[companion] bridge token: {TOKEN}  (pc 角色连接需带；COMPANION_TOKEN 环境变量可固定)")
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
