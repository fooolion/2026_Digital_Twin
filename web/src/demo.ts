/**
 * 任务控制器：一键夹取演示（关节空间轨迹播放器 + 运动学跟随抓取）
 *           + 路径录制回放（Catmull-Rom 关节插值 + 引导段）。
 * Rev: P5 · 2026-10-29
 */
import * as THREE from 'three';
import type { TcpIk } from './tcp-ik';

const mjOBJ_BODY = 1;
// tcp site = 两指垫接触面中心（模型 x=-0.015）；竖直下压姿态下指垫带正好套住样件中段
const GRASP_DZ = 0.008;       // 抓取点：tcp 在样件中心上方 8mm（指垫 ±10.5mm → 夹住 40mm 立方中段）
const HOVER_DZ = 0.10;        // 悬停点：tcp 在样件中心上方 100mm
const SAFE_Z = 0.30;          // 搬运安全高度（tcp 目标）
const DROP_TCP_Z = 0.029;     // 投放点 tcp 高度（桌面 0 + 样件半高 0.021 + 抓取偏移 0.008）
const GRIP_CUBE_RAD = 2.5;    // 闭合到 40mm 立方的齿轮角：单指 20mm ÷ 齿轮半径 0.008m（无指-件碰撞，纯视觉贴合）
const SETTLE_ERR = 0.010;     // 到位判定：物理 TCP 距航点 <10mm（等 CTC 收敛）
const GRASP_SETTLE_ERR = 0.004; // 抓取专用更严门槛：样件与指垫无物理碰撞（纯运动学跟随），
                             // 10mm 会在夹爪还悬空时就带走样件（隔空抓取感）——收紧到 4mm
                             // 且要求竖直分量同样 <4mm（指垫真正跨住样件中段才算贴上）
const GRASP_AXIS = [0, 0, -1] as const; // 夹爪轴（end_link x̂）目标方向：竖直朝下，全程顶抓姿态
const GRASP_J6 = -Math.PI / 4; // 抓取段 J6 期望：直搜解 J6 恒 0 → 指开合方向=yaw+90°（对立方斜 45°），
                               // J6=-45° 把开合方向转回世界 y，夹住样件的一对平行面
const JOINT_SPEED = 0.9;      // 段插值关节角速度上限（rad/s），柔和
const REC_INTERVAL = 90;      // 录制采样间隔（ms）
const REC_MAX = 1800;         // 录制点数上限（90ms×1800≈2.7 分钟）：防忘停录制导致 path/jPath
                             // 无限增长；到顶自动停（按钮态由 sim.ts 每帧轮询复位）

// 跟随/捕获段的复用临时量：演示全程每帧执行，new 会以 ~5 对象/帧 的速度制造 GC 垃圾
const _q1 = new THREE.Quaternion();
const _q2 = new THREE.Quaternion();
const _q3 = new THREE.Quaternion();
const _v1 = new THREE.Vector3();
const _m1 = new THREE.Matrix4(); // 释放落地高度计算用（releaseGrasp，低频）

interface Waypoint { x: number; y: number; z: number; t: number; }
interface JointWaypoint { q: Float64Array<ArrayBufferLike>; t: number; }

/** Catmull-Rom 标量插值（p1→p2 之间，f∈[0,1]；C1 连续——逐点 smoothstep 会在每个
 *  采样点停顿，密集采样点的回放必须用样条） */
function catmullRom(p0: number, p1: number, p2: number, p3: number, f: number): number {
  const f2 = f * f, f3 = f2 * f;
  return 0.5 * ((2 * p1) + (-p0 + p2) * f +
    (2 * p0 - 5 * p1 + 4 * p2 - p3) * f2 +
    (-p0 + 3 * p1 - 3 * p2 + p3) * f3);
}
interface Seg {
  q: Float64Array<ArrayBufferLike>; // 段终点关节角
  p: readonly [number, number, number]; // 段终点航点（settle 判定用）
  dur: number;                // 插值时长（s）
  hold: number;               // 到位后保持（s）
  applied: boolean;           // 到位事件是否已触发
  grasp?: 'cube' | 'busbar';  // 到位：闭合夹爪 + 捕获相对位姿进入跟随（目标 body）
  release?: 'cube' | 'busbar';// 到位：退出跟随（样件钉住在当前位置）
  drop?: boolean;             // 到位释放时播放落体动画到地面再钉住（母线放置专用；手动停止不落）
  gripCmd?: number;           // 到位：下发夹爪目标
  settle?: number;            // 到位后等待物理 TCP 收敛的上限（s）——插值走完≠物理到位
  label: string;
}

export interface DemoDeps {
  ik: TcpIk;
  data: any;
  model: any;
  mujoco: any;
  /** 拖拽目标点（录制采样来源） */
  ikTarget: THREE.Vector3;
  /** 绿球是否正被拖动（录制只在拖动中采样） */
  isDragging(): boolean;
  /** 演示段到达时下发夹爪目标（rad），主循环同步面板滑条 */
  setGripTarget(rad: number): void;
  /** 夹爪全开目标角（rad） */
  gripOpenRad: number;
}

export function createDemo(deps: DemoDeps) {
  const { ik, data, model, mujoco } = deps;

  const bodyId: number = mujoco.mj_name2id(model, mjOBJ_BODY, 'demo_cube');
  const hasCube = bodyId >= 0;
  const homePos = hasCube
    ? new THREE.Vector3(data.xpos[3 * bodyId], data.xpos[3 * bodyId + 1], data.xpos[3 * bodyId + 2])
    : new THREE.Vector3(0.34, 0.02, 0.021);
  const cubeQadr: number = hasCube ? model.jnt_qposadr[model.body_jntadr[bodyId]] : 0;
  const cubeDofadr: number = hasCube ? model.jnt_dofadr[model.body_jntadr[bodyId]] : 0;

  // ===== 母线（A11 阶段二拆件）：独立 freejoint body，运动学钉住 + 跟随 =====
  const busbarId: number = mujoco.mj_name2id(model, mjOBJ_BODY, 'battery_busbar');
  const hasBusbar = busbarId >= 0;
  const busQadr: number = hasBusbar ? model.jnt_qposadr[model.body_jntadr[busbarId]] : 0;
  const busDofadr: number = hasBusbar ? model.jnt_dofadr[model.body_jntadr[busbarId]] : 0;
  // 出生位姿（XML freejoint 初始值 = 定稿套配位姿）；钉点位姿 = 非抓取时母线凝固处
  const busHome = hasBusbar
    ? {
        pos: new THREE.Vector3(data.qpos[busQadr], data.qpos[busQadr + 1], data.qpos[busQadr + 2]),
        quat: new THREE.Quaternion(
          data.qpos[busQadr + 4], data.qpos[busQadr + 5], // x, y
          data.qpos[busQadr + 6], data.qpos[busQadr + 3], // z, w（qpos 序 xyzw）
        ),
      }
    : null;
  const busPin = hasBusbar ? { pos: busHome!.pos.clone(), quat: busHome!.quat.clone() } : null;

  // 夹取参数（2026-10-29 第六版·分阶段渐进接近 + 竖直向下释放）：
  //   母线真实几何（busbar.stl 实测）：49×99×126mm 立式拱桥（两根平行横轨+端部立柱），
  //   抓取的横梁 = 底部 26mm 高的一条；长轴 126mm 沿世界 y、梁远面 world x≈0.336、
  //   拱桥前壁 x≈0.334。
  //   夹取点 tcp 比梁中心提前 10mm（指尖自 tcp 前伸 15mm——XML 指垫盒 model x∈[−30,0]），
  //   指尖止于 0.331，距拱壁 3mm——不提前则闭合姿态本身就穿模（与开度无关）。
  //   渐进接近（用户路径）：启动即开爪 20mm/侧 → 展开点(J6=90° 水平径向姿态) →
  //   同 x 垂直下降到梁高（指尖全程在梁区外侧，斜线下降会扫梁）→ 开爪 30mm/侧 →
  //   水平滑入 40mm（横梁进入指区，间隙 60mm 不穿模）→ 闭合夹住上下平面 →
  //   垂直拔出 → 搬运 → 翻转调姿（轴竖直向下/肘抬高换分支）→ 下降张开包围 → 松爪落体。
  //   解算顺序 ≠ 播放顺序；持物段 branchLock 锁分支，调姿点全局重解。
  const BUSBAR_TCP_OFFSET = new THREE.Vector3(0.001, 0.015, 0.063); // 文件坐标；x 比梁中心提前 10mm 防指尖穿拱壁
  const GRIP_BUSBAR_RAD = 1.625;      // 上下平面间距 26mm → 单指 13mm ÷ 齿轮半径 8mm
  const GRIP_BUSBAR_PRE = 2.5;        // 接近段预备开度：行程 20mm/侧（不必全开，边走边开）
  const GRIP_BUSBAR_SLIDE = 3.75;     // 滑入段开度：行程 30mm/侧（间隙 60mm，横梁进入指区）
  const BUSBAR_LIFT = 0.06;           // 拔出高度（孔板脱离极柱 + 余量）
  const BUSBAR_HIGH_Z = 0.20;         // 升起/搬运高度（B 分支可达上限内；0.30 需换分支）
  const BUSBAR_PLACE_XY = new THREE.Vector3(-0.30, 0.30, 0); // 黄圈圆心（用户定稿坐标）
  const BUSBAR_AXIS_DOWN = [0, 0, -1] as const; // 放置段轴：竖直向下（腕部俯冲、肘抬高）
  const BUSBAR_PLACE_J6 = -Math.PI / 4; // 放置段 J6：闭合线对准世界 x（实测标定，非手推公式）
  const BUSBAR_ADJUST_Z = 0.15;       // 调姿高度：轴竖直向下在该半径(0.42m)的 tcp 可达上限 ~0.19
  const BUSBAR_PLACE_TCP_Z = 0.04;    // 释放 tcp 高度（拱底 ~2mm 即将触地，短落体）
  const GRAV = 9.81;                  // 落体动画重力（m/s²）
  // 母线真实包围盒·件系（busbar.stl 实测：49×99×126mm）——落地静止高度按释放瞬间
  // 姿态实时算（见 releaseGrasp）：竖直半高模型曾按 20×26×80 小梁假设，实测下陷 ~28mm
  const BUS_MIN = [0, 0, 0] as const;
  const BUS_MAX = [0.049, 0.099, 0.126] as const;
  /** 航点径向轴：水平外向单位向量（随航点 yaw 同转）。固定世界方向轴（如 [1,0,0]）在
   *  yaw≠0 的航点不可满足——J5=0 时夹爪轴被锁在目标射线的竖直平面内，轴约束失效。 */
  const busRadial = (p: readonly [number, number, number]): [number, number, number] => {
    const l = Math.hypot(p[0], p[1]) || 1;
    return [p[0] / l, p[1] / l, 0];
  };

  // 投放点 = 出生点 + 固定偏移；创建时即初始化（投放环从加载起就在正确位置）
  // 定稿（目视调整）：出生 (0.30,0.30)、投放 = 出生 + (-0.3,0.1) = (0,0.40)——
  // 均在 +y 侧远离电池（电池 x∈[0.33,0.55]），搬运走廊不与模组交叉
  const dropPos = homePos.clone().add(new THREE.Vector3(-0.3, 0.1, 0));

  let running = false;
  let done = false;
  let segs: Seg[] = [];
  let segIdx = 0;
  let segT = 0;
  const fromQ = new Float64Array(6);       // 当前段起点角
  const jointTarget = new Float64Array(6); // 对外：插值中的关节目标（主循环写 targetQ）

  // ---- 抓取跟随（kinematic grasp）----
  let graspHeld = false;
  const graspRelPos = new THREE.Vector3();     // 抓取瞬间样件相对夹爪的位置（end_link 系）
  const graspRelQuat = new THREE.Quaternion(); // 抓取瞬间样件相对夹爪的四元数
  const endLinkId: number = hasCube ? mujoco.mj_name2id(model, 1, 'end_link') : -1;

  // ---- 路径录制/回放（走伺服通路）----
  let rec = false;
  let recStart = 0;
  let lastSample = 0;
  let path: Waypoint[] = [];
  let replay = false;
  let replayT = 0;
  /** 回放的关节空间轨迹（startReplay 时由 TCP 路径离线解出，与 path 按下标对齐） */
  let jPath: JointWaypoint[] = [];

  /** 样件放回出生点 + 清速度 + 退出跟随（复位/演示开始时调用，保证可重复） */
  function resetCube(): void {
    if (!hasCube) return;
    graspHeld = false;
    data.qpos[cubeQadr] = homePos.x;
    data.qpos[cubeQadr + 1] = homePos.y;
    data.qpos[cubeQadr + 2] = homePos.z;
    data.qpos[cubeQadr + 3] = 1; data.qpos[cubeQadr + 4] = 0;
    data.qpos[cubeQadr + 5] = 0; data.qpos[cubeQadr + 6] = 0;
    for (let i = 0; i < 6; i++) data.qvel[cubeDofadr + i] = 0;
    ik.resetToPhysics();
  }

  /** 离线解一个航点：axisVec 给定轴约束（母线侧抓水平径向）；否则竖直下压（网格直搜） */
  function solveWaypoint(seed: Float64Array, p: readonly [number, number, number],
                         axis?: boolean, j6?: number,
                         axisVec?: readonly [number, number, number], j5?: number): Float64Array {
    return ik.solveTo(seed, p, axisVec ?? (axis ? GRASP_AXIS : undefined), j6, j5);
  }

  /** 生成整段演示计划（8 航点 → 8 段关节插值） */
  function buildPlan(): void {
    const c = homePos, d = dropPos;
    const W: {
      p: [number, number, number]; label: string; axis?: boolean; j6?: number;
      grasp?: 'cube' | 'busbar'; release?: 'cube' | 'busbar'; grip?: number; settle?: number;
    }[] = [
      { p: [c.x * 0.7, c.y, 0.38], label: '展开', grip: deps.gripOpenRad }, // 先张开夹爪再接近
      { p: [c.x, c.y, c.z + HOVER_DZ], label: '接近样件', axis: true, j6: GRASP_J6 },
      { p: [c.x, c.y, c.z + GRASP_DZ], label: '下探抓取', axis: true, j6: GRASP_J6, grasp: 'cube', grip: GRIP_CUBE_RAD, settle: 2.0 },
      { p: [c.x, c.y, c.z + HOVER_DZ], label: '抬升', axis: true, j6: GRASP_J6 },
      { p: [c.x, c.y, SAFE_Z], label: '升起', axis: true, j6: GRASP_J6 },
      { p: [d.x, d.y, SAFE_Z], label: '搬运', axis: true, j6: GRASP_J6 },
      { p: [d.x, d.y, DROP_TCP_Z], label: '下降投放', axis: true, j6: GRASP_J6, release: 'cube', grip: deps.gripOpenRad, settle: 1.5 },
      { p: [d.x + 0.05, d.y + 0.05, SAFE_Z], label: '回位', grip: 0 }, // 回位收拢夹爪（空手移动的标准姿态，演示结束保持闭合）
    ];
    segs = [];
    let seed: Float64Array<ArrayBufferLike> = new Float64Array(ik.angles); // 从当前 IK 解链式播种
    for (const wp of W) {
      const q = solveWaypoint(seed, wp.p, wp.axis, wp.j6);
      let maxD = 0;
      for (let i = 0; i < 6; i++) maxD = Math.max(maxD, Math.abs(q[i] - seed[i]));
      const dur = Math.min(6, Math.max(0.9, maxD / JOINT_SPEED + 0.25));
      segs.push({ q, p: wp.p, dur, hold: 0, applied: false, grasp: wp.grasp, release: wp.release, gripCmd: wp.grip, settle: wp.settle, label: wp.label });
      seed = q;
    }
    // 抓取/投放在位事件带 settle 等待（等物理臂收敛到航点 10mm 内），hold 留足动作时间：
    // 抓取 hold 2.5s——齿轮电机力矩饱和（±0.6N·m）从全开闭到夹住 40mm 立方需 ~2s，
    // 1.2s 没闭完就抬升会"张着爪拎样件"；投放 hold 0.8s 保持张开可见
    segs[2].hold = 2.5;
    segs[6].hold = 0.8;
  }

  /** 拆母线演示计划（A11 阶段二·第六版·渐进接近）：启动即开爪 20mm/侧 →
   *  展开点（水平径向姿态、J6=90°）→ **同 x 垂直下降**到梁高（指尖自 tcp 前伸 15mm，
   *  全程在梁区外侧——斜线下降会让下指扫过横梁，用户录像实测）→ 开爪 30mm/侧 →
   *  **水平滑入 25mm**（横梁进入指区间隙 60mm）→ 闭合夹住上下平面 → 垂直拔出
   *  （密集中间航点保直线——孔柱 0.2mm 配合）→ 升起/搬运 0.20 → 黄圈上空 0.15
   *  翻转调姿（轴竖直向下、肘抬高换分支、J6=−45° 闭合线对准世界 x）→ 下降 0.04
   *  张开包围 → 原位松爪落体（真实包围盒算落地高度）。解算顺序 ≠ 播放顺序：
   *  夹持点先全局解锚定 B 分支（腕反折族），下降/展开反向解、拔出→搬运正向解全
   *  branchLock 窗口搜索，调姿点全局重解（换分支），放置段窗口续搜。逐航点独立
   *  全局搜索会在肘上/肘下两族解之间跳变，关节插值穿过跳变时末端姿态大幅翻滚。
   *  夹取点运行时从母线出生位姿推导（XML 位姿联动）。 */
  function buildBusbarPlan(): void {
    if (!hasBusbar || !busHome) throw new Error('场景中没有 battery_busbar（母线）');
    const g = busHome.pos.clone().add(BUSBAR_TCP_OFFSET.clone().applyQuaternion(busHome.quat));
    const G: [number, number, number] = [g.x, g.y, g.z];
    const BWD = 0.032; // 下降列后退量：指尖(前伸15mm)距梁近面标称 17mm（关节插值有前漂，实测留足）
    const DSC: [number, number, number] = [g.x - BWD, g.y, g.z];        // 下降对姿：梁高（与滑入同 x 同姿态）
    const STG: [number, number, number] = [g.x - BWD, g.y, g.z + 0.06]; // 展开：下降列顶部
    const P = BUSBAR_PLACE_XY;
    const CARRY: [number, number, number] = [P.x, P.y, BUSBAR_HIGH_Z];
    const ADJ: [number, number, number] = [P.x, P.y, BUSBAR_ADJUST_Z];
    const PLC: [number, number, number] = [P.x, P.y, BUSBAR_PLACE_TCP_Z];
    const LOCK = true;
    const lockSolve = (seed: Float64Array, p: readonly [number, number, number]): Float64Array =>
      ik.solveTo(seed, p, busRadial(p), Math.PI / 2, undefined, LOCK);
    const qGrasp = lockSolve(new Float64Array(ik.angles), G); // 种子=当前姿态：窗口必不达标 → 全局搜索锚定分支
    const qDsc = lockSolve(qGrasp, DSC); // 反向解：下降列与滑入同 x 同姿态（纯竖直差动）
    const qStg = lockSolve(qDsc, STG);
    const lift = (dz: number, seed: Float64Array): Float64Array =>
      lockSolve(seed, [G[0], G[1], G[2] + dz]);
    const qL1 = lift(0.02, qGrasp);
    const qL2 = lift(0.04, qL1);
    const qL3 = lift(BUSBAR_LIFT, qL2);
    const qRise = lockSolve(qL3, [G[0], G[1], BUSBAR_HIGH_Z]);
    const qCarry = lockSolve(qRise, CARRY);
    // 翻转调姿：轴竖直向下 + J6=−45°——B 族窗口必然不达标，solveTo 自动回退全局搜索
    // 换 A 分支（肘抬高），母线随腕在空中翻转；j6InGrid 让网格在正确滚转角下评估轴/位置
    // （先解后转 J6 再拉回会让轴向偏 ~9°，母线落地倾斜）；放置段从调姿解窗口续搜同分支
    const qAdj = ik.solveTo(qCarry, ADJ, BUSBAR_AXIS_DOWN, BUSBAR_PLACE_J6, undefined, false, true);
    const qPlace = ik.solveTo(qAdj, PLC, BUSBAR_AXIS_DOWN, BUSBAR_PLACE_J6, undefined, LOCK, true);
    const HOME: [number, number, number] = [P.x + 0.05, P.y + 0.05, SAFE_Z];
    // 回位走网格（径向轴自然撤退姿态）：solve3D 局部迭代从伸展的放置位姿走不出去（实测 491mm 误差）
    const qHome = ik.solveTo(qPlace, HOME, busRadial(HOME));
    segs = [];
    const mk = (q: Float64Array<ArrayBufferLike>, p: readonly [number, number, number],
                label: string, extra: Partial<Seg> = {}): void => {
      // 段时长按"播放顺序"的前一段算（解算链与播放顺序不同）
      const prev = segs.length ? segs[segs.length - 1].q : new Float64Array(ik.angles);
      let maxD = 0;
      for (let i = 0; i < 6; i++) maxD = Math.max(maxD, Math.abs(q[i] - prev[i]));
      const dur = Math.min(6, Math.max(0.9, maxD / JOINT_SPEED + 0.25));
      segs.push({ q, p, dur, hold: 0, applied: false, label, ...extra });
    };
    mk(qStg, STG, '展开', { gripCmd: GRIP_BUSBAR_PRE });       // 接近中开到 20mm/侧（不必全开）
    mk(qDsc, DSC, '下降对姿', { gripCmd: GRIP_BUSBAR_SLIDE }); // 垂直下降，开到 30mm/侧
    mk(qGrasp, G, '滑入夹持', { grasp: 'busbar', gripCmd: GRIP_BUSBAR_RAD, settle: 2.0 });
    mk(qL1, [G[0], G[1], G[2] + 0.02], '垂直拔出·一');
    mk(qL2, [G[0], G[1], G[2] + 0.04], '垂直拔出·二');
    mk(qL3, [G[0], G[1], G[2] + BUSBAR_LIFT], '拔出');
    mk(qRise, [G[0], G[1], BUSBAR_HIGH_Z], '升起');
    mk(qCarry, CARRY, '搬运');
    mk(qAdj, ADJ, '翻转调姿');
    mk(qPlace, PLC, '下降放置', { gripCmd: deps.gripOpenRad, settle: 1.5 }); // 到位：张开指垫包围（母线仍被跟随钉住）
    mk(new Float64Array(qPlace), PLC, '松爪释放', { release: 'busbar', drop: true });
    mk(qHome, HOME, '回位', { gripCmd: 0 }); // 回位收拢夹爪（空手移动的标准姿态，演示结束保持闭合）
    segs[10].dur = 0.6;  // 原位释放段：插值无位移，缩短到 0.6s（到位 0.3s 即松爪）
    segs[1].hold = 0.8;  // 下降对姿：留足开爪追到 30mm 的时间（滑入全程间隙 ≥ 50mm）
    segs[2].hold = 2.5;  // 夹持闭合时间（留足包覆可见性）
    segs[9].hold = 1.2;  // 放置：指垫张开脱离母线表面（扩出下落间隙后再松爪）
    segs[10].hold = 0.8; // 释放可见
  }

  /** 抓取到位：捕获样件相对夹爪的位姿，进入运动学跟随（目标 = cube 或 busbar）。
   *  busbar 带软启动混合：触发瞬间母线从钉住位姿平滑过渡到跟随位姿（0.4s），
   *  消除"夹持瞬间跳动"——触发时物理 TCP 若未完全收敛，瞬移会很明显。 */
  let graspTarget: 'cube' | 'busbar' = 'cube';
  let graspedQadr = cubeQadr;    // 跟随目标的 qpos 地址（beginGrasp 时按目标设置）
  let graspedDofadr = cubeDofadr;
  const BUS_BLEND = 0.4;         // 混合时长（s）
  let busBlendT = -1;            // 混合起始时刻（ms；<0 = 非混合态）
  const busBlendPos = new THREE.Vector3();
  const busBlendQuat = new THREE.Quaternion();
  // 松爪落体（母线放置）：freejoint 无碰撞，松爪即钉会悬浮半空——先播匀加速下落
  // 动画到地面静止高度（姿态保持不变）再钉住；静止高度按释放姿态实时算（restZ）
  let busDrop: { from: THREE.Vector3; quat: THREE.Quaternion; t0: number; dur: number; restZ: number } | null = null;
  function beginGrasp(target: 'cube' | 'busbar', now: number): void {
    if (target === 'cube' && !hasCube) return;
    if (target === 'busbar' && !hasBusbar) return;
    graspTarget = target;
    graspedQadr = target === 'cube' ? cubeQadr : busQadr;
    graspedDofadr = target === 'cube' ? cubeDofadr : busDofadr;
    if (target === 'busbar') {
      busBlendPos.set(data.qpos[busQadr], data.qpos[busQadr + 1], data.qpos[busQadr + 2]);
      busBlendQuat.set(
        data.qpos[busQadr + 4], data.qpos[busQadr + 5],
        data.qpos[busQadr + 6], data.qpos[busQadr + 3],
      );
      busBlendT = now;
    }
    const q4 = 4 * endLinkId;
    _q1.set(data.xquat[q4 + 1], data.xquat[q4 + 2], data.xquat[q4 + 3], data.xquat[q4]); // eQ
    _q2.copy(_q1).invert(); // eQ⁻¹（复用临时量，勿动 _q1——后面还要用）
    const e3 = 3 * endLinkId;
    graspRelPos.set(
      data.qpos[graspedQadr] - data.xpos[e3],
      data.qpos[graspedQadr + 1] - data.xpos[e3 + 1],
      data.qpos[graspedQadr + 2] - data.xpos[e3 + 2],
    ).applyQuaternion(_q2);
    // 关键：MuJoCo 四元数 qpos 是 wxyz，THREE.Quaternion.set 是 (x,y,z,w)——分量序
    // 直传会把姿态打乱（母线 home 姿态被换成 120° 错姿态 → 夹取瞬间翻转 120° 成
    // "竖直悬挂"，立方体因对称性看不出来）。项目第 5 次"字面相同约定不同"。
    _q3.set(
      data.qpos[graspedQadr + 4], data.qpos[graspedQadr + 5],
      data.qpos[graspedQadr + 6], data.qpos[graspedQadr + 3],
    );
    graspRelQuat.copy(_q2.multiply(_q3)); // eQ⁻¹·cQ
    graspHeld = true;
  }

  /** 释放跟随：绿块走物理；母线钉住当前位姿——drop=true（放置航点）先播落体动画
   *  再钉到地面，手动停止不带 drop：原地钉住（freejoint 无碰撞，不钉会穿桌坠落）。
   *  落地静止高度按释放瞬间姿态算：旋转包围盒最低点触地 →
   *  origin_z = −Σ min(R[2][i]·BUS_MIN[i], R[2][i]·BUS_MAX[i])（体原点在包围盒 (0,0,0) 角）。 */
  function releaseGrasp(target: 'cube' | 'busbar', drop = false): void {
    graspHeld = false;
    if (target === 'busbar' && hasBusbar && busPin) {
      const px = data.qpos[busQadr], py = data.qpos[busQadr + 1], pz = data.qpos[busQadr + 2];
      const quat = new THREE.Quaternion(
        data.qpos[busQadr + 4], data.qpos[busQadr + 5],
        data.qpos[busQadr + 6], data.qpos[busQadr + 3],
      );
      _m1.makeRotationFromQuaternion(quat);
      const e = _m1.elements; // 列主序：e[2]/e[6]/e[10] = 旋转矩阵第 3 行（件 x̂/ŷ/ẑ 的世界 z 分量）
      let restZ = 0;
      for (let i = 0; i < 3; i++) {
        const rz = e[2 + 4 * i]; // 件第 i 轴的世界 z 分量
        restZ += Math.min(rz * BUS_MIN[i], rz * BUS_MAX[i]);
      }
      restZ = Math.max(-restZ, 0.004); // 兜底：怪异姿态不让原点低于桌面
      if (drop && pz > restZ + 0.001) {
        busDrop = {
          from: new THREE.Vector3(px, py, pz), quat, restZ,
          t0: performance.now(), dur: Math.sqrt(2 * (pz - restZ) / GRAV),
        };
        return; // busPin 在落体结束时写入（busbarIdleTick）
      }
      busPin.pos.set(px, py, pz);
      busPin.quat.copy(quat);
    }
  }

  /** 母线钉住（sim.ts 主循环每帧调，任何状态）：落体动画 → 钉点位姿写回 + 清速度 */
  function busbarIdleTick(): void {
    if (!hasBusbar || !busPin) return;
    if (graspTarget === 'busbar' && graspHeld) return; // 跟随中由跟随逻辑写
    if (busDrop) {
      const f = Math.min(1, (performance.now() - busDrop.t0) / (busDrop.dur * 1000));
      const t = f * busDrop.dur;
      data.qpos[busQadr] = busDrop.from.x;
      data.qpos[busQadr + 1] = busDrop.from.y;
      data.qpos[busQadr + 2] = Math.max(busDrop.from.z - 0.5 * GRAV * t * t, busDrop.restZ);
      data.qpos[busQadr + 3] = busDrop.quat.w;
      data.qpos[busQadr + 4] = busDrop.quat.x;
      data.qpos[busQadr + 5] = busDrop.quat.y;
      data.qpos[busQadr + 6] = busDrop.quat.z;
      for (let i = 0; i < 6; i++) data.qvel[busDofadr + i] = 0;
      if (f >= 1) {
        busPin.pos.set(busDrop.from.x, busDrop.from.y, busDrop.restZ);
        busPin.quat.copy(busDrop.quat);
        busDrop = null;
      }
      return;
    }
    data.qpos[busQadr] = busPin.pos.x;
    data.qpos[busQadr + 1] = busPin.pos.y;
    data.qpos[busQadr + 2] = busPin.pos.z;
    data.qpos[busQadr + 3] = busPin.quat.w;
    data.qpos[busQadr + 4] = busPin.quat.x;
    data.qpos[busQadr + 5] = busPin.quat.y;
    data.qpos[busQadr + 6] = busPin.quat.z;
    for (let i = 0; i < 6; i++) data.qvel[busDofadr + i] = 0;
  }

  /** 母线放回出生（套配位姿）——拆母线演示开始/场景复位时调用，保证可重复 */
  function resetBusbar(): void {
    if (!hasBusbar || !busHome) return;
    graspHeld = false;
    busDrop = null;
    data.qpos[busQadr] = busHome.pos.x;
    data.qpos[busQadr + 1] = busHome.pos.y;
    data.qpos[busQadr + 2] = busHome.pos.z;
    data.qpos[busQadr + 3] = busHome.quat.w;
    data.qpos[busQadr + 4] = busHome.quat.x;
    data.qpos[busQadr + 5] = busHome.quat.y;
    data.qpos[busQadr + 6] = busHome.quat.z;
    for (let i = 0; i < 6; i++) data.qvel[busDofadr + i] = 0;
    busPin!.pos.copy(busHome.pos);
    busPin!.quat.copy(busHome.quat);
  }

  /** 物理 TCP 距航点的距离（settle 判定；site_xpos 来自上一步物理，无需额外 forward） */
  function tcpErr(p: readonly [number, number, number]): number {
    const o = 3 * ik.siteId;
    return Math.hypot(data.site_xpos[o] - p[0], data.site_xpos[o + 1] - p[1], data.site_xpos[o + 2] - p[2]);
  }

  /** 段到位事件（抓取/释放/夹爪命令），物理收敛或超时兜底时调用 */
  function applySeg(seg: Seg, now: number): void {
    seg.applied = true;
    if (seg.grasp) beginGrasp(seg.grasp, now);
    if (seg.release) releaseGrasp(seg.release, seg.drop === true);
    if (seg.gripCmd !== undefined) deps.setGripTarget(seg.gripCmd);
  }

  /** 主循环每帧调用（dt=秒，now=rAF 时间戳 ms） */
  function tick(dt: number, now: number): void {
    // —— 路径录制（拖球中采样 ikTarget）；到 REC_MAX 自动停（防忘停导致无限增长）——
    if (rec && deps.isDragging() && now - lastSample > REC_INTERVAL) {
      lastSample = now;
      path.push({ x: deps.ikTarget.x, y: deps.ikTarget.y, z: deps.ikTarget.z, t: now - recStart });
      if (path.length >= REC_MAX) stopRec(); // 按钮态由 sim.ts 每帧轮询 recRunning 自动复位
    }
    // —— 路径回放（关节空间轨迹播放，与演示同一通路；不经逐帧 IK）——
    if (replay && jPath.length >= 2) {
      replayT += dt * 1000;
      const last = jPath[jPath.length - 1];
      if (replayT >= last.t) {
        jointTarget.set(last.q);
        const p = path[path.length - 1];
        deps.ikTarget.set(p.x, p.y, p.z);
        stopReplay();
      } else {
        let i = 0;
        while (i < jPath.length - 2 && jPath[i + 1].t <= replayT) i++;
        const a = jPath[i], b = jPath[i + 1];
        const f = Math.min(1, Math.max(0, (replayT - a.t) / Math.max(b.t - a.t, 1)));
        const p0 = jPath[Math.max(i - 1, 0)].q, p1 = a.q, p2 = b.q,
              p3 = jPath[Math.min(i + 2, jPath.length - 1)].q;
        for (let k = 0; k < 6; k++) {
          jointTarget[k] = catmullRom(p0[k], p1[k], p2[k], p3[k], f);
        }
        // 绿球沿录制的 TCP 路径同步走（视觉：球沿橙色录制线移动）。
        // 注意下标错位：jPath[0] 是引导段，jPath[k≥1] ↔ path[k-1]——段 0 球停录制起点
        const pi = Math.min(Math.max(i - 1, 0), path.length - 2);
        const pa = path[pi], pb = path[pi + 1];
        const fBall = i === 0 ? 0 : f;
        deps.ikTarget.set(
          pa.x + (pb.x - pa.x) * fBall, pa.y + (pb.y - pa.y) * fBall, pa.z + (pb.z - pa.z) * fBall);
      }
    }
    // —— 演示段播放（关节插值，不走 IK）——
    if (!running) return;
    segT += dt;
    let guard = 0;
    while (guard++ < 8) {
      const seg = segs[segIdx];
      // 到位事件：物理 TCP 距航点足够近才触发（段中点起判；settle 上限兜底）。
      // 抓取段用更严门槛 + 竖直分量判定：样件无指-件碰撞，靠 TCP 几何贴近模拟"接触"，
      // 门槛松了夹爪还悬空样件就跟着走（隔空抓取），死等插值走完则夹爪在半空闭合。
      const gate = seg.grasp ? GRASP_SETTLE_ERR : SETTLE_ERR;
      const zErr = Math.abs(data.site_xpos[3 * ik.siteId + 2] - seg.p[2]);
      if (!seg.applied && segT >= seg.dur * 0.5 &&
          ((tcpErr(seg.p) < gate && zErr < gate) || segT >= seg.dur + (seg.settle ?? 0))) {
        applySeg(seg, now);
      }
      const total = seg.dur + seg.hold;
      if (segT < total) break;
      if (!seg.applied) applySeg(seg, now); // 兜底：settle 超时也触发（hold 末尾）
      fromQ.set(seg.q); // 下一段起点 = 本段终点
      segT -= total;
      segIdx++;
      if (segIdx >= segs.length) { running = false; done = true; return; }
    }
    if (!running) return;
    const f0 = Math.min(segT / segs[segIdx].dur, 1);
    const e = f0 * f0 * (3 - 2 * f0); // smoothstep
    for (let i = 0; i < 6; i++) {
      jointTarget[i] = fromQ[i] + (segs[segIdx].q[i] - fromQ[i]) * e;
    }
    // —— 抓取跟随：目标每帧严格随夹爪（视觉上手指贴着物体）——
    if (graspHeld) {
      const e3 = 3 * endLinkId, q4 = 4 * endLinkId;
      _q1.set(data.xquat[q4 + 1], data.xquat[q4 + 2], data.xquat[q4 + 3], data.xquat[q4]); // eQ
      _v1.copy(graspRelPos).applyQuaternion(_q1);
      const fx = data.xpos[e3] + _v1.x, fy = data.xpos[e3 + 1] + _v1.y;
      let fz = data.xpos[e3 + 2] + _v1.z;
      if (graspTarget === 'cube') {
        // z 不低于出生位（桌面静止高度）：夹爪收敛末段仍略高于航点时，跟随会把样件
        // 压向桌面（无指-件碰撞，物理不挡）——钳制住，样件只贴桌面滑，不穿桌
        fz = Math.max(fz, homePos.z - 0.0005);
      }
      _q1.multiply(graspRelQuat); // 跟随目标四元数（eQ·rel）
      let px = fx, py = fy, pz = fz;
      if (graspTarget === 'busbar' && busBlendT >= 0) {
        // 软启动混合：从抓取瞬间的钉住位姿平滑过渡到跟随位姿（消除触发瞬间的跳动）
        const f = Math.min(1, (now - busBlendT) / (BUS_BLEND * 1000));
        const s = f * f * (3 - 2 * f);
        px = busBlendPos.x + (fx - busBlendPos.x) * s;
        py = busBlendPos.y + (fy - busBlendPos.y) * s;
        pz = busBlendPos.z + (fz - busBlendPos.z) * s;
        _q2.copy(busBlendQuat).slerp(_q1, s);
        _q1.copy(_q2);
        if (f >= 1) busBlendT = -1; // 混合完成 → 纯跟随
      }
      data.qpos[graspedQadr] = px;
      data.qpos[graspedQadr + 1] = py;
      data.qpos[graspedQadr + 2] = pz;
      data.qpos[graspedQadr + 3] = _q1.w; data.qpos[graspedQadr + 4] = _q1.x;
      data.qpos[graspedQadr + 5] = _q1.y; data.qpos[graspedQadr + 6] = _q1.z;
      for (let i = 0; i < 6; i++) data.qvel[graspedDofadr + i] = 0;
    }
  }

  // ---- 对外接口 ----
  function start(): void {
    if (!hasCube) throw new Error('场景中没有 demo_cube（占位样件）');
    stopRec();
    stopReplay();
    resetCube(); // 每轮演示从干净状态开始（可重复）
    deps.setGripTarget(deps.gripOpenRad); // 夹爪立即开始全开（边走边开，接近前已就绪）
    running = true;
    done = false;
    segIdx = 0;
    segT = 0;
    fromQ.set(ik.angles); // 从当前 IK 解起步
    buildPlan();
  }

  function startBusbar(): void {
    if (!hasBusbar) throw new Error('场景中没有 battery_busbar（母线）');
    stopRec();
    stopReplay();
    stop();
    resetBusbar(); // 母线放回极柱（重复演示可复位）
    // 夹爪启动即开始张开（用户定：20mm/侧预备开度即可，不必全开等 4.8s——
    // 分级开度 20→30mm 与渐进接近同步，滑入前到位）
    deps.setGripTarget(GRIP_BUSBAR_PRE);
    running = true;
    done = false;
    segIdx = 0;
    segT = 0;
    fromQ.set(ik.angles);
    buildBusbarPlan();
  }

  function stop(): void {
    running = false;
    // 中途停止退出跟随：绿块走物理掉落；母线钉住在当前位（不钉会穿桌坠落）
    if (graspHeld) releaseGrasp(graspTarget);
  }

  function startRec(): void {
    stop();
    stopReplay();
    path = [];
    rec = true;
    recStart = performance.now();
    lastSample = 0;
  }

  function stopRec(): void {
    rec = false;
  }

  function clearPath(): void {
    stopRec();
    stopReplay();
    path = [];
    jPath = [];
  }

  function startReplay(): void {
    if (path.length < 2) throw new Error('还没有录制的路径');
    stop();
    rec = false;
    // 种子取物理当前角（ik.angles 可能滞后物理——拖拽结束 1.2s 后才重播种），
    // 并前置 800ms 引导段：从当前姿态平滑滑到录制起点，回放不从跳变起步
    const seed0 = new Float64Array(6);
    for (let i = 0; i < 6; i++) seed0[i] = data.qpos[i];
    jPath = [{ q: new Float64Array(seed0), t: path[0].t - 800 }];
    let seed: Float64Array = new Float64Array(seed0);
    for (const wp of path) {
      const q = ik.solveTo(seed, [wp.x, wp.y, wp.z]);
      jPath.push({ q, t: wp.t });
      seed = new Float64Array(q);
    }
    if (jPath.length < 3) throw new Error('路径求解失败'); // 防御断言：path≥2 时不可达
    replay = true;
    replayT = jPath[0].t; // 从引导段起播
  }

  function stopReplay(): void {
    replay = false;
  }

  return {
    start, stop, startBusbar,
    get running() { return running; },
    /** 当前计划种类（按钮态区分：两个演示共用 running 标志） */
    get planKind(): 'cube' | 'busbar' { return segs.some((s) => s.grasp === 'busbar') ? 'busbar' : 'cube'; },
    get label() { return running ? segs[segIdx]?.label ?? '' : (done ? '完成' : '待命'); },
    get jointTarget() { return jointTarget; },
    tick,
    busbarIdleTick, resetBusbar,
    resetCube,
    startRec, stopRec,
    get recRunning() { return rec; },
    get recCount() { return path.length; },
    getPath(): Waypoint[] { return path; },
    clearPath,
    startReplay, stopReplay,
    get replayRunning() { return replay; },
    get dropPoint() { return dropPos; },
    get busbarDropPoint() { return BUSBAR_PLACE_XY; },
  };
}
