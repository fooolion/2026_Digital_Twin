/**
 * TCP 逆运动学模块：servo() 拖拽逐帧伺服（3×3 DLS）+ solveTo() 离线航点求解
 *           （3D 链式迭代 / 带竖直下压轴约束时走三级网格直搜——6D 迭代法在此臂病态不可用）。
 * 影子纪律：求解在 data.qpos 影子上进行，进出必须 save/restore。
 * Rev: P1/P5 · 2026-10-29
 */

// 参考站同名参数（实测验证过的量级）
const GAIN = 12;                 // 误差步进增益
const MIN_DAMPING = 0.018;       // DLS 最小阻尼
const MAX_DAMPING = 0.075;       // DLS 最大阻尼（奇异附近）
const SINGULARITY_THRESHOLD = 0.08;
const MAX_JOINT_SPEED = 2.8;     // rad/s
const REACH_ERROR = 0.0015;      // 1.5mm 判"到达"
const IK_ITERATIONS = 3;         // servo 内迭代轮数（限位裁剪后的误差再分配）
const SOLVE_MAX_ITER = 150;      // solveTo 离线收敛的最大轮数（矢量限幅后步长小，轮数给足）
const clip = (v: number, a: number, b: number) => Math.min(b, Math.max(a, v));

export interface TcpIk {
  /** tcp site 的 id（sim.ts 侧读 site_xpos 复用，免重复 mj_name2id） */
  readonly siteId: number;
  /** 当前 IK 解（6 个臂关节角，rad，已含限位裁剪）——作为物理 PD 的目标 */
  readonly angles: Float64Array;
  /** 拖拽/回放模式：朝目标走一步（dt=帧时长 s）。返回当前 TCP 误差模长 (m)。 */
  servo(target: readonly [number, number, number], dt: number): number;
  /** 离线求解：从 seed 出发迭代到收敛（≤SOLVE_MAX_ITER=150 轮），返回解的角度副本（不碰物理态）。
   *  axisDesired（可选）：末端轴（end_link x̂ = 夹爪指向）的世界方向目标。
   *  j6（可选）：期望 J6 角（rad，对齐样件平行面）——轴约束解先置该值再锁 J6 拉回位置。
   *  j5（可选）：侧抓腕角绝对值（rad）——夹爪轴水平（如母线横梁上下平面夹持）时传入，
   *  网格对 ±j5 取轴误差更优者，返回的 q[4] 即选定符号。
   *  branchLock（可选）：true 时先围绕 seed 解的同分支邻域（±35°）窗口搜索，位置 <4cm 且
   *  轴误差 <8.6° 即采纳，不达标才回退全局搜索——持物段（母线拆解）分支连续性的关键：
   *  本臂水平径向位姿有肘上/肘下两族解，逐航点独立全局搜索会在两族间跳变，关节插值
   *  穿过跳变时末端姿态大幅翻滚（持物翻滚/母线竖直悬挂的根因）。 */
  solveTo(seed: Float64Array, target: readonly [number, number, number],
          axisDesired?: readonly [number, number, number], j6?: number, j5?: number,
          branchLock?: boolean): Float64Array; // 恒有解（网格/迭代路径不返回 null）
  /** 把 IK 解重置为物理当前关节角（抓球/死区回位时调用——否则 IK 解与物理姿态
   *  脱节，回位瞬间 CTC 目标角跳变，臂会"弹"一下）。 */
  resetToPhysics(): void;
}

export function createTcpIk(mujoco: any, model: any, data: any): TcpIk {
  const mjOBJ_SITE = 6;
  const mjOBJ_BODY = 1;
  const siteId: number = mujoco.mj_name2id(model, mjOBJ_SITE, 'tcp');
  if (siteId < 0) throw new Error('模型里找不到 site "tcp"');
  const endLinkId: number = mujoco.mj_name2id(model, mjOBJ_BODY, 'end_link');

  const N = 6; // 6 个臂关节 = qpos/dof 0..5（齿轮=6、双指滑轨=7,8 不进 IK）
  const lo = new Float64Array(N), hi = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    lo[i] = model.jnt_range[i * 2];
    hi[i] = model.jnt_range[i * 2 + 1];
  }

  const angles = new Float64Array(N);
  const savedQpos = new Float64Array(data.qpos.length); // 影子状态快照（复用，免每帧分配）

  const snap = (): void => { for (let i = 0; i < N; i++) data.qpos[i] = angles[i]; };
  const forward = (): void => mujoco.mj_forward(model, data);

  function resetToPhysics(): void {
    for (let i = 0; i < N; i++) angles[i] = clip(data.qpos[i], lo[i], hi[i]);
  }
  resetToPhysics(); // IK 解从当前物理姿态起步

  /** 把整组臂角写入 data 并 forward（调用方负责保存/恢复） */
  function putQ(a: Float64Array): void {
    for (let i = 0; i < N; i++) data.qpos[i] = clip(a[i], lo[i], hi[i]);
    forward();
  }

  function tcp(): [number, number, number] {
    const o = siteId * 3;
    return [data.site_xpos[o], data.site_xpos[o + 1], data.site_xpos[o + 2]];
  }

  // 解析雅可比：mj_jacSite 的 out-param 走 DoubleBuffer（构造参数是元素个数），
  // 预分配复用；jacr（旋转部分）本 IK 用不到但必须给缓冲。行主序 3×nv。
  const nv: number = model.nv;
  const jacpBuf = new mujoco.DoubleBuffer(3 * nv);
  const jacrBuf = new mujoco.DoubleBuffer(3 * nv);
  const jacp = jacpBuf.GetView() as Float64Array;

  /** 当前 data 运动学下的雅可比 J[3][N]（只取前 6 个 dof 列；调用前须已 forward） */
  function jacobian(): number[][] {
    mujoco.mj_jacSite(model, data, jacpBuf, jacrBuf, siteId);
    const J: number[][] = [[], [], []];
    for (let col = 0; col < N; col++) {
      J[0].push(jacp[col]);
      J[1].push(jacp[nv + col]);
      J[2].push(jacp[2 * nv + col]);
    }
    return J;
  }

  /** end_link x̂（夹爪指向）的世界方向（调用前须已 forward） */
  function endAxis(): [number, number, number] {
    const q4 = 4 * endLinkId;
    const w = data.xquat[q4], x = data.xquat[q4 + 1], y = data.xquat[q4 + 2], z = data.xquat[q4 + 3];
    return [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)];
  }
  const cross3 = (a: number[], b: number[]): [number, number, number] => [
    a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0],
  ];
  const norm3 = (a: number[]): number => Math.hypot(a[0], a[1], a[2]);

  /** 6 维迭代法（6×6 Gram DLS / 转置法）已删：本臂 6D Gram 近奇异，迭代漂进镜像
   *  解/限位（实测），带姿态航点改走 solveByGrid 三级网格直搜——纯函数评估无陷阱。 */

  const dot = (a: number[], b: number[]): number => a.reduce((s, v, i) => s + v * b[i], 0);
  const det3 = (A: number[][]): number =>
    A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  /** 解 (A)x=b（3x3），奇异返回 null */
  function solve3(A: number[][], b: number[]): number[] | null {
    const det = det3(A);
    if (Math.abs(det) < 1e-9) return null;
    const inv = [
      [(A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det, (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det, (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det],
      [(A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det, (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det, (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det],
      [(A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det, (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det, (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det],
    ];
    return inv.map((row) => row[0] * b[0] + row[1] * b[1] + row[2] * b[2]);
  }

  /** DLS：解 (JJᵀ+λ²I)y=e，返回 dq=Jᵀy。阻尼随奇异度自适应。 */
  function dls(J: number[][], e: number[]): number[] | null {
    const gram = [
      [dot(J[0], J[0]), dot(J[0], J[1]), dot(J[0], J[2])],
      [dot(J[1], J[0]), dot(J[1], J[1]), dot(J[1], J[2])],
      [dot(J[2], J[0]), dot(J[2], J[1]), dot(J[2], J[2])],
    ];
    const scale = Math.max((gram[0][0] + gram[1][1] + gram[2][2]) / 3, 1e-9);
    const normDet = Math.max(0, det3(gram)) / Math.max(scale * scale * scale, 1e-12);
    const sing = clip(1 - normDet / SINGULARITY_THRESHOLD, 0, 1);
    const lambda2 = (MIN_DAMPING + (MAX_DAMPING - MIN_DAMPING) * sing * sing) ** 2;
    const y = solve3(
      [
        [gram[0][0] + lambda2, gram[0][1], gram[0][2]],
        [gram[1][0], gram[1][1] + lambda2, gram[1][2]],
        [gram[2][0], gram[2][1], gram[2][2] + lambda2],
      ],
      e,
    );
    if (!y) return null;
    const dq = new Array<number>(N);
    for (let i = 0; i < N; i++) dq[i] = J[0][i] * y[0] + J[1][i] * y[1] + J[2][i] * y[2];
    return dq;
  }

  function servo(target: readonly [number, number, number], dt: number): number {
    if (dt <= 0) dt = 0.016;
    savedQpos.set(data.qpos);
    try {
      snap();
      forward();
      let c = tcp();
      let e = [target[0] - c[0], target[1] - c[1], target[2] - c[2]];
      let errN = Math.hypot(e[0], e[1], e[2]);
      const scale = clip(GAIN * dt, 0.08, 0.65);
      const maxStep = MAX_JOINT_SPEED * dt;

      // 内迭代（最多 3 轮）：关节顶到限位被裁剪后，剩余误差在下一轮重新分配给
      // 其余关节——缓解"单关节锁死卡住、其他关节不补位"。无改进即回滚并停（防打转）。
      for (let iter = 0; iter < IK_ITERATIONS && errN >= REACH_ERROR; iter++) {
        const J = jacobian();
        const dq = dls(J, [e[0] * scale, e[1] * scale, e[2] * scale]);
        if (!dq) break;
        const prev = angles.slice();
        const next = new Float64Array(N);
        for (let i = 0; i < N; i++) next[i] = angles[i] + clip(dq[i], -maxStep, maxStep);
        for (let i = 0; i < N; i++) angles[i] = clip(next[i], lo[i], hi[i]);
        snap();
        forward();
        c = tcp();
        const e2 = [target[0] - c[0], target[1] - c[1], target[2] - c[2]];
        const err2 = Math.hypot(e2[0], e2[1], e2[2]);
        if (err2 > errN * 0.98) { // 这步没变好 → 回滚
          for (let i = 0; i < N; i++) angles[i] = prev[i];
          snap();
          forward();
          break;
        }
        errN = err2;
        e = e2;
      }
      return errN;
    } finally {
      // 恢复物理态（qpos + 运动学缓存），渲染/步进不受影子计算影响
      for (let i = 0; i < savedQpos.length; i++) data.qpos[i] = savedQpos[i];
      forward();
    }
  }

  function solveTo(
    seed: Float64Array,
    target: readonly [number, number, number],
    axisDesired?: readonly [number, number, number],
    j6?: number,
    j5?: number,
    branchLock?: boolean,
  ): Float64Array {
    // 影子纪律与 servo 一致：进出恢复物理态——中途 putQ 的候选解（含被拒的）
    // 绝不能留在 data.qpos 里，否则演示建计划一结束物理臂被瞬移到垃圾姿态
    const saved = new Float64Array(data.qpos.length);
    saved.set(data.qpos);
    try {
      const q = axisDesired
        ? solveByGrid(target, axisDesired, j5, branchLock ? seed : undefined)
        : solve3D(seed, target);
      // j6 期望（对齐样件平行面）：直搜解 J6 恒 0——先置期望值，再锁 J6 用其余关节
      // 拉回位置（J6 滚转会带 tcp 漂移 ~11.5mm，由 J1..J4 补偿；轴方向大体保持竖直）
      if (j6 !== undefined) {
        q[5] = clip(j6, lo[5], hi[5]);
        return solve3D(q, target, j6);
      }
      return q;
    } finally {
      for (let i = 0; i < saved.length; i++) data.qpos[i] = saved[i];
      forward();
    }
  }

  const AXIS_W = 0.5;      // 轴向误差权重（rad ↔ m 折算）
  const WIN_SPAN = 35;     // branchLock 窗口半宽（deg）：同分支邻域
  const WIN_GATE_P = 0.04; // 窗口解验收：位置误差 <4cm（超出 → 全局重搜，允许必要换分支）
  const WIN_GATE_A = 0.15; // 窗口解验收：轴误差 <0.15（≈8.6°）

  /** 纯位置 3D DLS：链式种子迭代到收敛（无姿态诉求的过渡航点，历史验证收敛）。
   *  j6Lock：锁定 J6（rad）只解其余关节——配合 solveTo 的 J6 期望做位置拉回。 */
  function solve3D(seed: Float64Array, target: readonly [number, number, number],
                   j6Lock?: number): Float64Array {
    const q = new Float64Array(N);
    for (let i = 0; i < N; i++) q[i] = clip(seed[i], lo[i], hi[i]);
    putQ(q);
    let c = tcp();
    let err = Math.hypot(target[0] - c[0], target[1] - c[1], target[2] - c[2]);
    if (err < REACH_ERROR) return q;
    const stepFrac = 0.35;       // 每轮走剩余误差的 35%（离线求解无帧率约束）
    const maxStep = 0.06;        // 每轮关节步幅上限（rad）
    for (let iter = 0; iter < SOLVE_MAX_ITER && err >= REACH_ERROR; iter++) {
      if (j6Lock !== undefined) q[5] = clip(j6Lock, lo[5], hi[5]);
      putQ(q);
      const J = jacobian();
      c = tcp();
      const e = [target[0] - c[0], target[1] - c[1], target[2] - c[2]];
      const dq = dls(J, [e[0] * stepFrac, e[1] * stepFrac, e[2] * stepFrac]);
      if (!dq) break;
      for (let i = 0; i < N; i++) q[i] = clip(q[i] + clip(dq[i], -maxStep, maxStep), lo[i], hi[i]);
      putQ(q);
      c = tcp();
      const err2 = Math.hypot(target[0] - c[0], target[1] - c[1], target[2] - c[2]);
      if (err2 >= err) break; // 无改进 → 停（返回当前最优 q）
      err = err2;
    }
    return q;
  }

  /** 6D 航点三级网格直搜：J1 对准目标 yaw、J2/J3/J4 竖直平面三关节求解
   *  （竖直下压 J5=J6=0；侧抓传 j5=腕角绝对值，网格对 ±j5 取轴误差更优者）。
   *  粗 10° 宽窗 → ±10°·5° → ±4°·2°，纯函数评估、无局部最小陷阱。
   *  seed（可选）：先做种子 ±35° 窗口搜索（同分支连续，5° 粗扫 + 两级细化），
   *  达标即返回；否则走全局三级搜索。 */
  function solveByGrid(
    target: readonly [number, number, number],
    axisDesired: readonly [number, number, number],
    j5?: number,
    seed?: Float64Array,
  ): Float64Array {
    const yaw = Math.atan2(target[1], target[0]);
    const D = Math.PI / 180;
    const des = [...axisDesired];
    let j5Best = 0;
    /** 评 j2/j3/j4（度）+ j5（rad，直写）：写入 data + forward + 位置误差 + AXIS_W×轴向误差 */
    const evalQ = (j2: number, j3: number, j4: number, j5v: number): number => {
      for (let i = 0; i < N; i++) data.qpos[i] = 0;
      data.qpos[0] = yaw;
      data.qpos[1] = j2 * D; data.qpos[2] = j3 * D; data.qpos[3] = j4 * D;
      data.qpos[4] = j5v;
      forward();
      const c = tcp();
      return Math.hypot(target[0] - c[0], target[1] - c[1], target[2] - c[2]) +
        AXIS_W * norm3(cross3(endAxis(), des));
    };
    // 侧抓（j5 传入）：±j5 哪个使轴误差更小（J5 轴方向决定，解析上取决于安装）
    if (j5 !== undefined) {
      const e1 = evalQ(-90, -60, 15, j5);
      const e2 = evalQ(-90, -60, 15, -j5);
      j5Best = e1 <= e2 ? j5 : -j5;
    }
    const withJ5 = (j2: number, j3: number, j4: number): number => evalQ(j2, j3, j4, j5Best);
    // —— 窗口优先（branchLock）：围绕种子解的同分支邻域搜索。窗口解用分量误差验收
    //    （位置/轴，非加权代价）——达标即返回，持物段绝不换分支；不达标回退全局。 ——
    if (seed) {
      const loD = [lo[1] / D, lo[2] / D, lo[3] / D], hiD = [hi[1] / D, hi[2] / D, hi[3] / D];
      const s2 = clip(seed[1] / D, loD[0], hiD[0]);
      const s3 = clip(seed[2] / D, loD[1], hiD[1]);
      const s4 = clip(seed[3] / D, loD[2], hiD[2]);
      let wBest: number[] | null = null;
      let wErr = Infinity;
      for (let j2 = Math.max(loD[0], s2 - WIN_SPAN); j2 <= Math.min(hiD[0], s2 + WIN_SPAN); j2 += 5)
        for (let j3 = Math.max(loD[1], s3 - WIN_SPAN); j3 <= Math.min(hiD[1], s3 + WIN_SPAN); j3 += 5)
          for (let j4 = Math.max(loD[2], s4 - WIN_SPAN); j4 <= Math.min(hiD[2], s4 + WIN_SPAN); j4 += 5) {
            if (j2 > 0 || j3 > 0) continue; // J2/J3 上限 0
            const e = withJ5(j2, j3, j4);
            if (e < wErr) { wErr = e; wBest = [j2, j3, j4]; }
          }
      if (wBest) {
        for (const [span, step] of [[8, 4], [4, 2]] as const) {
          const b2 = wBest[0], b3 = wBest[1], b4 = wBest[2];
          for (let j2 = b2 - span; j2 <= b2 + span; j2 += step)
            for (let j3 = Math.max(loD[1], b3 - span); j3 <= Math.min(hiD[1], b3 + span); j3 += step)
              for (let j4 = Math.max(loD[2], b4 - span); j4 <= Math.min(hiD[2], b4 + span); j4 += step) {
                if (j2 > 0 || j3 > 0) continue;
                const e = withJ5(j2, j3, j4);
                // 原地写入（不再新配数组）：避免 wBest 的窄化类型与 j2 推断成环（TS7022）
                if (e < wErr) { wErr = e; wBest[0] = j2; wBest[1] = j3; wBest[2] = j4; }
              }
        }
        for (let i = 0; i < N; i++) data.qpos[i] = 0;
        data.qpos[0] = yaw;
        data.qpos[1] = wBest[0] * D; data.qpos[2] = wBest[1] * D; data.qpos[3] = wBest[2] * D;
        data.qpos[4] = j5Best;
        forward();
        const c = tcp();
        const pErr = Math.hypot(target[0] - c[0], target[1] - c[1], target[2] - c[2]);
        if (pErr < WIN_GATE_P && norm3(cross3(endAxis(), des)) < WIN_GATE_A) {
          return new Float64Array([
            clip(yaw, lo[0], hi[0]), wBest[0] * D, wBest[1] * D, wBest[2] * D, j5Best, 0,
          ]);
        }
      }
    }
    let best = [-90, -60, 15];
    let bestErr = withJ5(best[0], best[1], best[2]);
    // 一级：肘上工作区宽窗
    for (let j2 = -140; j2 <= -60; j2 += 10)
      for (let j3 = -110; j3 <= -10; j3 += 10)
        for (let j4 = -107; j4 <= 88; j4 += 10) { // J4 全限位覆盖（母线侧抓解需 J4≈-107）
          const e = withJ5(j2, j3, j4);
          if (e < bestErr) { bestErr = e; best = [j2, j3, j4]; }
        }
    // 二/三级：围绕当前最优逐步细化
    for (const [span, step] of [[10, 5], [4, 2]] as const) {
      const [b2, b3, b4] = best;
      for (let j2 = b2 - span; j2 <= b2 + span; j2 += step)
        for (let j3 = b3 - span; j3 <= b3 + span; j3 += step)
          for (let j4 = Math.max(-107, b4 - span); j4 <= Math.min(90, b4 + span); j4 += step) {
            if (j2 > 0 || j3 > 0) continue; // J2/J3 上限 0
            const e = withJ5(j2, j3, j4);
            if (e < bestErr) { bestErr = e; best = [j2, j3, j4]; }
          }
    }
    return new Float64Array([
      clip(yaw, lo[0], hi[0]), best[0] * D, best[1] * D, best[2] * D, j5Best, 0,
    ]);
  }

  return { siteId, angles, servo, solveTo, resetToPhysics };
}
