# web/ —— 数字孪生网页端

对外宣传 + 功能集成的网站（5 个页面），当前已实现**宣传首页**与**浏览器内机械臂仿真工作台**（功能页2 内核，含一键夹取演示与路径录制回放）；直播 / 遥操作 / 创意工坊为施工占位页。

**进度**：P1 面板 / P2 骨架 / P3 首页 / P4 演示+录制回放 均已完成（P4 暂告段落）；P5 伴生服务+直播、P6 遥操作、P7 创意工坊、P8 部署待做。**场景融合待办**：真电池模型/场景尺寸到位后，替换 `public/models/` 场景里的 `demo_cube` 占位（换 geom、保持 body 名即可，演示控制器无缝对接）。路线图详见根目录 `工作日志.md`「四-D」。

## 运行

```powershell
cd web
npm install        # 首次
npm run dev        # 开发 → http://localhost:5173（子页面 /sim.html 等）
npm run build      # 打包纯静态产物到 dist/（可部署 GitHub Pages）
```

要求 Node.js ≥ 20。

## 部署（GitHub Pages，已配自动发布）

push 到 `main` 后 GitHub Actions 自动构建并发布：**https://fooolion.github.io/2026_Digital_Twin/**（仿真页 `/sim.html`）。工作流见 `.github/workflows/deploy.yml`，无需任何手动操作。

- **首次启用**（一次性，仓库网页操作）：Settings → Pages → Build and deployment → Source 选 **GitHub Actions**；之后每次 push 自动部署。
- `vite.config.ts` 的 `base: '/2026_Digital_Twin/'` 与网址子路径绑定——改仓库名须同步改；运行时资源（模型 XML/STL）必须用 `import.meta.env.BASE_URL` 拼 URL（见 `src/sim.ts`），写死 `/models/...` 在子路径下会 404。

## 技术栈

- **Vite + TypeScript**（无框架，照参考站 Yang-Ci/Rebot_Arm_AGV 的 web_mujoco 架构）
- **@mujoco/mujoco**（DeepMind 官方 WASM）：浏览器里直接编译加载机械臂场景
- **three**：渲染。机械臂网格从 MuJoCo 编译结果生成，不单独建模
- **Tailwind CSS v4**：站点主题与页面布局
- **GSAP + Lenis**：首页滚动叙事与平滑滚动
- 控制链路：TCP 拖拽（绿球=目标点）→ 阻尼最小二乘 IK → 逐关节 KP/KD 计算力矩控制（CTC）

---

## 文件清单与功能说明

### 页面入口（五页站点）

| 文件 | 功能 |
|---|---|
| `index.html` | **宣传首页**。结构：全页固定环境背景层（.ambient 网格+三光斑）→ 共享导航 → Hero（官方项目名三行渐变大标题 + 徽章 + 双 CTA + 4 项数据卡）→ 痛点区（3 卡 + 结论句）→ 工作原理四环节流程（渐变连线 + 步骤点亮）→ 核心亮点 6 卡 → 素材位（3 个 16:9 占位框，待真机照片/录屏）→ 团队区（2 教师 + 4 队员）→ CTA → footer。文案取自 `docs/作品设计方案（改）.docx`；样式在 `src/landing.css`，逻辑在 `src/home.ts` |
| `sim.html` | **仿真工作台**（功能页2，当前最完整）。全屏 three.js 画布 + 顶部导航 + HUD（运行状态 / TCP 误差 / 手动控制提示）+ 右侧关节控制面板 + 下发实体臂弹窗（#send-modal，UI 先行）。`.jp-*` 面板样式全部内联在本文件 `<style>` 中；逻辑在 `src/sim.ts` |
| `live.html` | **实时直播**（功能页1，施工占位）。规划：PICO 第一视角 WebRTC 大屏（OBS→MediaMTX）+ 双 USB 摄像头 MJPEG 小窗 + 手柄姿态可视化 |
| `teleop.html` | **遥操作镜像**（功能页3，施工占位）。规划：PICO 端操作界面还原 + 键鼠→手柄映射（经 WebSocket 直通 PC 权威端） |
| `workshop.html` | **创意工坊**（功能页4，施工占位）。规划：STL/OBJ/GLB 本地导入 + 场景拖放排布 + STEP（开发中）。已含"从本地导入电池模型"按钮（点击 toast 提示施工中） |

### src/ ——TypeScript 源码

| 文件 | 功能 |
|---|---|
| `sim.ts` | **仿真页入口**（最大的模块）。① 资源加载：fetch XML+STL → MjVFS → MuJoCo 编译；② three 场景/相机/灯光/网格；③ geom 渲染：mesh 类走编译网格缓存，box（样件）/plane（桌面）按 geom_size 生成（group 3 调试碰撞盒跳过）；④ 渲染同步：body 世界位姿 ∘ geom 静态偏移（绑定未暴露 geom_xquat 的替代方案）；⑤ 物理循环：1kHz 固定步长、逐关节 KP/KD 计算力矩控制；⑥ 拖拽交互：抓球（相机平行面）/ 死区回位 / 人工抢占；⑦ 面板 10Hz 同步 + 路径线可视化 + 下发弹窗接线；⑧ 调试句柄 `window.__mj` |
| `tcp-ik.ts` | **逆运动学**（双通路）。`servo(target, dt)`：每帧一步的 3D DLS（拖拽/路径回放的连续跟踪，影子状态进出恢复物理态）；`solveTo(seed, target, axisDesired?)`：演示航点离线求解——无姿态诉求走 `solve3D`（链式播种 3D DLS），带竖直下压轴约束的接触航点走 `solveByGrid`（三级网格直搜：10° 宽窗→±10°·5°→±4°·2°，J1=目标 yaw、J5=J6=0；6D 迭代法在本臂 Gram 近奇异会漂进镜像解，实测网格 ≤5mm/轴向 −1.0）；`resetToPhysics()` 抓球/回位时重播种防跳变 |
| `demo.ts` | **一键夹取演示 + 路径录制回放**。演示=关节空间轨迹播放器：start() 时八个航点逐个 solveTo 离线解成关节角（接近/下探/投放三航点带竖直下压轴约束），tick() 按段 smoothstep 插值直接写 targetQ（不经逐帧 IK，运动平滑）；夹爪命令：展开=全开、抓取=闭合到 2.5rad（单指 20mm≈40mm 立方）、投放=张开；抓/放事件以**物理 TCP 收敛到航点 10mm 内**为准（settle，段中点起判+超时兜底——插值到位≠物理到位，防隔空抓取）；抓取=运动学跟随（捕获样件相对夹爪位姿，样件每帧严格随夹爪）；投放=释放跟随真实掉落。另含路径录制（拖球时 90ms 采样）/回放（**关节空间轨迹**：回放开始时把 TCP 路径离线链式解成关节角序列，按 Catmull-Rom 插值直接写 jointTarget——不经逐帧 IK；前置 800ms 引导段从当前姿态平滑滑入录制起点） |
| `panel.ts` | **右侧关节控制面板**（纯 DOM 模块）。6 关节滑条（目标/当前双读数）+ 夹爪开度滑条 + 张开/闭合按钮 + 任务·路径区（一键演示/记录/回放/清除/下发实体臂）+ 复位按钮；对外提供按钮态权威同步接口（setDemoRunning/setRecRunning/setPathInfo） |
| `nav.ts` | **共享顶部导航**：五页链接 active 高亮、滚动加深、GitHub 链接；另导出 `bindReveal`（滚动显现 IntersectionObserver）与 `toast`（轻提示），供各页复用 |
| `home.ts` | **首页逻辑**：mountNav + 滚动显现 + Hero 光斑随鼠标漂移（reduced-motion 用户与触屏自动禁用） |
| `stub.ts` | **三个占位页共用逻辑**：mountNav + 滚动显现 + 创意工坊导入按钮的 toast 占位 |
| `theme.css` | **Tailwind v4 入口 + 全站设计令牌**（ink/acc/warn/fog/mist 色板）+ 共享组件样式（玻璃导航 / 按钮 / 滚动显现 / toast / 全页固定环境背景层 .ambient） |
| `landing.css` | **首页专属样式**：Hero（光斑/网格底纹/渐变标题）、痛点卡、四环节流程连线、亮点网格、素材占位框、团队头像、CTA、footer |

### public/ ——浏览器静态资产

| 文件 | 功能 |
|---|---|
| `models/ReBot_Arm_DM.xml` | **网页版 MuJoCo 场景**（自包含副本）。= 桌面权威模型 `mujoco_sim/models/ReBot_Arm_DM.xml` 的网页变体：追加工作台面 plane + 占位电池样件（40mm 立方 `demo_cube`，freejoint）。两处差异：网页侧 `timestep=0.001`（桌面 0.002）；样件 body 名保留，将来换真电池模型只换 geom 不动代码 |
| `models/meshes/**` | 机械臂 22 个 STL 网格（collision/mujoco_collision/shared/visual），加载时经 MjVFS 注入浏览器编译 |

### 配置

| 文件 | 功能 |
|---|---|
| `vite.config.ts` | 五个 html 入口 + @tailwindcss/vite 插件 |
| `package.json` / `package-lock.json` | 依赖与脚本（运行时依赖 three / @mujoco/mujoco / gsap / lenis；开发依赖 vite / typescript / tailwindcss / @tailwindcss/vite / @types/three） |
| `tsconfig.json` | TypeScript 配置（严格模式） |
| `.gitignore` | 排除 node_modules / dist |

---

## web 外的关联文件（仓库其他位置）

| 文件 | 与网页工程的关系 |
|---|---|
| `mujoco_sim/models/ReBot_Arm_DM.xml` | **桌面权威 MuJoCo 模型**（真机标定参数所在）。`public/models/` 的网页副本源自它；桌面 `timestep=0.002`，网页侧运行时改为 0.001，互不影响 |
| `工作日志.md` →「四-D. 网页工程计划与待办」 | P1-P8 路线图（P1 面板 / P2 骨架 / P3 首页 / P4 演示已完成；P5 伴生服务+直播 / P6 遥操作 / P7 创意工坊 / P8 部署待做）与全部网页阶段记录 |
| `docs/作品设计方案（改）.docx` | 首页文案的权威来源（项目名/痛点/四环节/亮点均提炼自它） |
| `docs/报名表.jpg` | 团队区人员信息来源（仅姓名/分工上页，敏感信息不上页） |

---

## 已知的网页版特有注意点

- `@mujoco/mujoco` 绑定**未暴露 `geom_xquat`** → 渲染用 body 位姿 ∘ geom 静态偏移组合
- `tcp` site = **两指垫接触面中心**（end_link 系 x=-0.015，距指尖平面 15mm）——演示抓取高度按它算，别当成腕部
- 演示的夹爪开合是**纯视觉命令**：指-样件无碰撞对（contype/conaffinity 不互认），闭合角 2.5rad 正好贴住 40mm 立方
- 夹爪齿轮电机偏慢（~100°/s）：抓取命令在物理收敛时才下发，闭合动作会延续到保持段/抬升前段完成
- 3.16 编译器把 mesh 顶点直接存在 geom 系（勿再叠 mesh_pos 烘焙，会双重偏移）
- 网页侧 `model.opt.timestep=0.001`（桌面 XML 仍是 0.002，互不影响）
- 演示抓取为**运动学跟随**（样件 kinematic 随夹爪，非手指接触力闭环）——演示级替代，真实抓取由真机执行
- `public/models/` 21MB 原始 STL：开发期直接伺服；正式部署前学参考站做 gzip 预压缩

## 调试

控制台 `window.__mj`（mujoco / model / data / camera / ball / ikTarget / ik / demo / tcpSite）与 `window.__mjDbg()`；面板状态行与 HUD 常驻显示演示阶段与 TCP 误差。
