# web/ —— 数字孪生网页端

「报废电池拆解数字孪生集成机器人遥操作系统」的网页端：六页站点（总览驾驶舱 + 老师工作流①-④ + 关于页），浏览器内跑真 MuJoCo 物理仿真，HUD 工业大屏视觉（2026-10-25 按指导老师反馈重组：定位为**操作员用的应用软件**而非宣传页）。

**线上**：https://fooolion.github.io/2026_Digital_Twin/

**场景融合待办**：真电池模型/场景尺寸到位后，替换 `public/models/` 场景里的 `demo_cube` 占位（换 geom、保持 body 名即可，演示控制器无缝对接）。路线图详见根目录 `工作日志.md`「四-D」。

## 运行

```powershell
cd web
npm install        # 依赖：three / @mujoco/mujoco / urdf-loader / echarts / gsap / lenis / tailwindcss / vite / typescript
npm run dev        # 开发（http://localhost:5173/2026_Digital_Twin/，注意带 base 前缀）
npm run build      # 产物 dist/（6 页）
```

## 伴生服务（P5-A 骨架，比赛 PC 本机跑）

```powershell
# 先构建（服务托管 dist 产物）
cd web && npm run build
# 启动（依赖 fastapi/uvicorn 已装 .rebot venv）
mujoco_sim/.rebot/Scripts/python.exe web/server/main.py
# → http://127.0.0.1:8800/（局域网设备访问 http://本机IP:8800/）
```

- `GET /api/health`：健康检查（总览页 companion.ts 探测此端点，在线→状态灯转绿"实时模式"）
- PICO 大屏三层降级（P5-C）：① 实时流——环境变量 `PICO_WHEP_URL`（MediaMTX 的 WHEP 端点，如 `http://192.168.1.100:8889/pico/whep`；OBS 推流→MediaMTX→网页 WHEP 直连拉流，src/whep.ts）② 预录视频——放入 `web/public/media/pico.mp4`（**必须 H.264 编码**，Chromium 不支持 mp4v；mp4 不入库，.gitkeep 保留目录）后 build 即自动循环播放 ③ 占位层（附操作指引）。`GET /api/stream` 下发配置。
- `GET /cam/{0,1}.mjpg`：双摄像头 MJPEG 流（P5-B；懒启动、15fps、多连接共享编码；相机未接自动占位帧流不断）；`GET /api/cams` 在线状态。相机映射默认 `1,2`（工业相机侧墙/后墙），环境变量 `COMPANION_CAMS="0,2"` 可覆盖——**相机编号本机相关，换机器重确认**
- `WS /ws`：桥接 hub（角色 web/pc；input/telemetry 即时广播 + 最新状态补发——P5-B~E 的地基）
- 静态托管挂双路径（根 + `/2026_Digital_Twin/` 前缀，兼容 dist 的 Pages base）

## 部署（GitHub Pages，已配自动发布）

push 到 main → `.github/workflows/deploy.yml` 自动构建发布。`vite.config.ts` 的 `base: '/2026_Digital_Twin/'` 与源码里 `import.meta.env.BASE_URL` 拼接模型路径——改仓库名时两处同步。

## 技术栈

| 层 | 选型 |
|---|---|
| 构建 | Vite 8（六 html 入口）+ TypeScript（noUnused* 严格项，未开全量 strict）+ Tailwind CSS v4（@theme 令牌） |
| 仿真 | @mujoco/mujoco 3.16 WASM（浏览器内编译 MJCF+STL，MjVFS 注入） |
| 渲染 | three.js 0.186（仿真网格从 MuJoCo 编译结果生成）+ urdf-loader（总览真机 URDF 视口） |
| 图表 | ECharts 6 按需引入（echarts/core + Line/Grid/Tooltip/Legend，异步 chunk） |
| 动效 | GSAP + Lenis（关于页滚动叙事）；HUD 动画纯 CSS |

## 文件清单与功能说明

### 页面入口（vite.config.ts 六入口，一一对应）

| 文件 | 页面 | 引入的模块 | 说明 |
|---|---|---|---|
| index.html | 总览驾驶舱 | src/home.ts + cockpit.css | 左系统状态/场景配置（数据驱动预留）+ 中真机 URDF 视口（官方 DM visual 21MB 带进度条）+ 右 KPI + 底部四步工作流 |
| workshop.html | ①零件库 | src/workshop.ts + parts.css | STL/GLB/GLTF/OBJ 本地导入（不上传、毫米自动识别、blob 映射解析 glTF Separate）+ 预览视口；「投放仿真」预留 |
| sim.html | ②仿真规划 | src/sim.ts + tcp-ik.ts + demo.ts + panel.ts | 浏览器真 MuJoCo：TCP 拖拽 IK / 关节面板 / 一键夹取演示（关节轨迹播放器+运动学跟随）/ 路径录制回放 / 下发弹窗（UI 先行）。jp-* 面板样式内联于此文件 |
| teleop.html | ③遥操作 | src/teleop.ts + teleop.css | PICO 手柄键鼠镜像（左 X/Y 右 A/B，PICO 4 真机布局）+ 三级防护状态机（LOCKED→ARMED⇄E-STOP，可手动闭锁）+ 映射表/事件日志 |
| live.html | ④运行监控 | src/live.ts + monitor.css | 视频占位（PICO 大屏+双相机，P5 填 src 即活）+ 急停 IEC 闭锁 + 速度档 + ECharts 六关节滚动遥测（演示数据） |
| about.html | 关于 | src/about.ts + landing.css | 宣传内容（痛点/原理/亮点/素材位）；匿名评审要求：**不出现任何校名/姓名/比赛名** |

### src/ 模块

| 文件 | 职责 |
|---|---|
| nav.ts | 共享：mountNav（步骤式导航）+ toast + renderStatList（跨页状态列表组件） |
| theme.css | 共享：设计令牌 + HUD 原语（.hud-panel 四角括号/.lamp 三态灯/.step-no/.vp-tag/.stat-list）+ 导航 + sr-only。**跨页组件必须住这里** |
| home.ts / about.ts / live.ts / teleop.ts / workshop.ts | 各页逻辑（文件头有 Rev 与 P5 接入点说明） |
| sim.ts | 仿真工作台主体：MuJoCo 加载/渲染同步/CTC 控制/拖拽交互/任务调度（629 行，最大模块） |
| tcp-ik.ts | IK：servo() 逐帧伺服 + solveTo() 离线航点（6D 迭代在此臂病态，姿态约束走三级网格直搜） |
| demo.ts | 任务控制器：夹取演示状态机 + 路径录制回放 |
| panel.ts | 关节控制面板（DOM 生成与状态同步） |

### public/ 资产

| 路径 | 内容 | 体积 |
|---|---|---|
| models/ReBot_Arm_DM.xml | 网页版仿真场景（桌面权威模型副本 + 工作台面 + demo_cube 占位样件） | — |
| models/meshes/ | 仿真网格（collision/visual/shared/mujoco_collision，22 个 STL） | ~21MB |
| models/ReBot_Arm_DM_preview.urdf | 总览视口 URDF 副本（mesh 路径重写指 visual/） | — |
| models/visual/ | 官方 DM 分色视觉网格 30 件（**原始全分辨率**，用户决策保真优先） | ~21MB（合计 public ~42MB） |

### web 外的关联文件

- `mujoco_sim/models/ReBot_Arm_DM.xml`：桌面权威模型（网页版的上游）
- `工作日志.md`：四-D 节为网页工程路线图与挂账（本地文件，不入库）
- `.github/workflows/deploy.yml`：自动部署

## 已知的网页版特有注意点

- **Web MuJoCo 3.16 绑定**：无 `geom_xquat`（用 body 世界位姿 ∘ geom 静态偏移组合）；编译器把网格顶点变换进 geom 系（勿再烘焙 mesh_pos/quat，会双重偏移）；`DoubleBuffer` 构造参数=元素个数、视图方法 `GetView()` 大写 G（官方 README 过时，以 d.ts 为准）；`eq_active` 在 MjData 不在 MjModel。
- **Z-up 世界**：Circle/Ring 默认 XY 平面天然水平（勿加 Y-up 习惯的 rotation.x=-π/2）；GridHelper 需 rotation.x=π/2 放平；竖直圆柱 rotation.x=π/2。
- **URDF J2/J3 限位上限 0**（与 MuJoCo 方向不同）、右指行程 [-0.05,0]——动画/IK 方程必须用限位内区间。
- **urdf-loader loadAsync 在骨架建好即 resolve**，网格异步继续到——取景/测量必须等 mesh 计数就位。
- **演示抓取为运动学跟随**（非手指接触力闭环）；回放/演示共用关节空间轨迹播放器（不经逐帧 IK）。
- timestep 0.001（桌面 0.002）；CTC KP=[80,100,100,35,25,18,300] / KD=[8,10,10,4,3,2.5,40]（照参考站真机同步参数）。

## 调试

- `window.__mj`（sim 页）：{ mujoco, model, data }——控制台现场检查物理状态。
- 两处 `[MJ]` console.log（sim.ts 内，有注释声明）为加载诊断，有意保留。
