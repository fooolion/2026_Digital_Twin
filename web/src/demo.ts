/**
 * 任务控制器：一键夹取演示（关节空间轨迹播放器 + 运动学跟随抓取）
 *           + 路径录制回放（Catmull-Rom 关节插值 + 引导段）。
 * Rev: P4 · 2026-10-22
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
const SETTLE_ERR = 0.010;     // 抓/放到位判定：物理 TCP 距航点 <10mm 才触发事件（等 CTC 收敛，防"隔空抓取"）
const GRASP_AXIS = [0, 0, -1] as const; // 夹爪轴（end_link x̂）目标方向：竖直朝下，全程顶抓姿态
const JOINT_SPEED = 0.9;      // 段插值关节角速度上限（rad/s），柔和
const REC_INTERVAL = 90;      // 录制采样间隔（ms）
const REC_MAX = 1800;         // 录制点数上限（90ms×1800≈2.7 分钟）：防忘停录制导致 path/jPath
                             // 无限增长；到顶自动停（按钮态由 sim.ts 每帧轮询复位）

// 跟随/捕获段的复用临时量：演示全程每帧执行，new 会以 ~5 对象/帧 的速度制造 GC 垃圾
const _q1 = new THREE.Quaternion();
const _q2 = new THREE.Quaternion();
const _q3 = new THREE.Quaternion();
const _v1 = new THREE.Vector3();

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
  grasp?: boolean;            // 到位：闭合夹爪 + 捕获相对位姿进入跟随
  release?: boolean;          // 到位：退出跟随（样件掉落）
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

  /** 离线解一个航点：接触关键航点带竖直下压轴约束（网格直搜），过渡航点纯位置（链式播种） */
  function solveWaypoint(seed: Float64Array, p: readonly [number, number, number], axis?: boolean): Float64Array {
    return ik.solveTo(seed, p, axis ? GRASP_AXIS : undefined);
  }

  /** 生成整段演示计划（8 航点 → 8 段关节插值） */
  function buildPlan(): void {
    const c = homePos, d = dropPos;
    const W: {
      p: [number, number, number]; label: string; axis?: boolean;
      grasp?: boolean; release?: boolean; grip?: number; settle?: number;
    }[] = [
      { p: [c.x * 0.7, c.y, 0.38], label: '展开', grip: deps.gripOpenRad }, // 先张开夹爪再接近
      { p: [c.x, c.y, c.z + HOVER_DZ], label: '接近样件', axis: true },
      { p: [c.x, c.y, c.z + GRASP_DZ], label: '下探抓取', axis: true, grasp: true, grip: GRIP_CUBE_RAD, settle: 2.0 },
      { p: [c.x, c.y, c.z + HOVER_DZ], label: '抬升' },
      { p: [c.x, c.y, SAFE_Z], label: '升起' },
      { p: [d.x, d.y, SAFE_Z], label: '搬运' },
      { p: [d.x, d.y, DROP_TCP_Z], label: '下降投放', axis: true, release: true, grip: deps.gripOpenRad, settle: 1.5 },
      { p: [d.x + 0.05, d.y + 0.05, SAFE_Z], label: '回位', grip: 0 }, // 回位收拢夹爪（空手移动的标准姿态，演示结束保持闭合）
    ];
    segs = [];
    let seed: Float64Array<ArrayBufferLike> = new Float64Array(ik.angles); // 从当前 IK 解链式播种
    for (const wp of W) {
      const q = solveWaypoint(seed, wp.p, wp.axis);
      let maxD = 0;
      for (let i = 0; i < 6; i++) maxD = Math.max(maxD, Math.abs(q[i] - seed[i]));
      const dur = Math.min(6, Math.max(0.9, maxD / JOINT_SPEED + 0.25));
      segs.push({ q, p: wp.p, dur, hold: 0, applied: false, grasp: wp.grasp, release: wp.release, gripCmd: wp.grip, settle: wp.settle, label: wp.label });
      seed = q;
    }
    // 抓取/投放在位事件带 settle 等待（等物理臂收敛到航点 10mm 内），hold 留足闭合动作可见时间
    segs[2].hold = 1.2;
    segs[6].hold = 0.8;
  }

  /** 抓取到位：捕获样件相对夹爪的位姿，进入运动学跟随 */
  function beginGrasp(): void {
    if (!hasCube) return;
    const q4 = 4 * endLinkId;
    _q1.set(data.xquat[q4 + 1], data.xquat[q4 + 2], data.xquat[q4 + 3], data.xquat[q4]); // eQ
    _q2.copy(_q1).invert(); // eQ⁻¹（复用临时量，勿动 _q1——后面还要用）
    const e3 = 3 * endLinkId;
    graspRelPos.set(
      data.qpos[cubeQadr] - data.xpos[e3],
      data.qpos[cubeQadr + 1] - data.xpos[e3 + 1],
      data.qpos[cubeQadr + 2] - data.xpos[e3 + 2],
    ).applyQuaternion(_q2);
    _q3.set(
      data.qpos[cubeQadr + 3], data.qpos[cubeQadr + 4],
      data.qpos[cubeQadr + 5], data.qpos[cubeQadr + 6],
    );
    graspRelQuat.copy(_q2.multiply(_q3)); // eQ⁻¹·cQ
    graspHeld = true;
  }

  /** 物理 TCP 距航点的距离（settle 判定；site_xpos 来自上一步物理，无需额外 forward） */
  function tcpErr(p: readonly [number, number, number]): number {
    const o = 3 * ik.siteId;
    return Math.hypot(data.site_xpos[o] - p[0], data.site_xpos[o + 1] - p[1], data.site_xpos[o + 2] - p[2]);
  }

  /** 段到位事件（抓取/释放/夹爪命令），物理收敛或超时兜底时调用 */
  function applySeg(seg: Seg): void {
    seg.applied = true;
    if (seg.grasp) beginGrasp();
    if (seg.release) graspHeld = false;
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
      // 到位事件：物理 TCP 距航点 <SETTLE_ERR 即触发（段中点起判；settle 上限兜底）。
      // 插值到位≠物理到位——提前抓会把"追赶落差"固化进跟随偏移（隔空抓取），
      // 死等插值走完则夹爪在半空闭合，所以以物理收敛为准。
      if (!seg.applied && segT >= seg.dur * 0.5 &&
          (tcpErr(seg.p) < SETTLE_ERR || segT >= seg.dur + (seg.settle ?? 0))) {
        applySeg(seg);
      }
      const total = seg.dur + seg.hold;
      if (segT < total) break;
      if (!seg.applied) applySeg(seg); // 兜底：settle 超时也触发（hold 末尾）
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
    // —— 抓取跟随：样件每帧严格随夹爪（视觉上手指贴着方块）——
    if (graspHeld && hasCube) {
      const e3 = 3 * endLinkId, q4 = 4 * endLinkId;
      _q1.set(data.xquat[q4 + 1], data.xquat[q4 + 2], data.xquat[q4 + 3], data.xquat[q4]); // eQ
      _v1.copy(graspRelPos).applyQuaternion(_q1);
      data.qpos[cubeQadr] = data.xpos[e3] + _v1.x;
      data.qpos[cubeQadr + 1] = data.xpos[e3 + 1] + _v1.y;
      data.qpos[cubeQadr + 2] = data.xpos[e3 + 2] + _v1.z;
      _q1.multiply(graspRelQuat); // eQ·rel（_q1 后续不再以 eQ 身份使用）
      data.qpos[cubeQadr + 3] = _q1.w; data.qpos[cubeQadr + 4] = _q1.x;
      data.qpos[cubeQadr + 5] = _q1.y; data.qpos[cubeQadr + 6] = _q1.z;
      for (let i = 0; i < 6; i++) data.qvel[cubeDofadr + i] = 0;
    }
  }

  // ---- 对外接口 ----
  function start(): void {
    if (!hasCube) throw new Error('场景中没有 demo_cube（占位样件）');
    stopRec();
    stopReplay();
    resetCube(); // 每轮演示从干净状态开始（可重复）
    running = true;
    done = false;
    segIdx = 0;
    segT = 0;
    fromQ.set(ik.angles); // 从当前 IK 解起步
    buildPlan();
  }

  function stop(): void {
    running = false;
    graspHeld = false; // 中途停止退出跟随（样件自然掉落）
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
    start, stop,
    get running() { return running; },
    get label() { return running ? segs[segIdx]?.label ?? '' : (done ? '完成' : '待命'); },
    get jointTarget() { return jointTarget; },
    tick,
    resetCube,
    startRec, stopRec,
    get recRunning() { return rec; },
    get recCount() { return path.length; },
    getPath(): Waypoint[] { return path; },
    clearPath,
    startReplay, stopReplay,
    get replayRunning() { return replay; },
    get dropPoint() { return dropPos; },
  };
}
