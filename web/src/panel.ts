/**
 * 关节控制面板：6 关节滑条（目标/当前双读数）+ 夹爪开度滑条/按钮 + 任务路径区。
 * 模式：与 sim.ts 的 targetQ 三来源对应（任务/拖拽/手动互斥）；视觉规范见 sim.html 的 .jp-* 样式。
 * Rev: P1 · 2026-10-19
 */

export interface JointSpec {
  label: string;
  minDeg: number;
  maxDeg: number;
}

export interface JointPanel {
  root: HTMLElement;
  /** 模式徽标：true=手动（橙）/ false=TCP 拖拽（绿） */
  setMode(manual: boolean): void;
  /** 夹爪按钮 active 态 */
  setGripOpen(open: boolean): void;
  /** IK 模式下让滑条跟随 IK 解（deg）；正在被用户拖动的滑条跳过 */
  syncSliders(anglesDeg: number[]): void;
  /** 当前角读数（deg，逐关节）+ 夹爪行程（mm） */
  syncReadouts(curDeg: number[], travelMm: number): void;
  /** 一键演示运行态（按钮文字/占用态切换） */
  setDemoRunning(run: boolean): void;
  /** 路径录制运行态 */
  setRecRunning(run: boolean): void;
  /** 路径信息行（点数/回放态；active=琥珀强调） */
  setPathInfo(text: string, active: boolean): void;
}

export function createJointPanel(
  mount: HTMLElement,
  joints: JointSpec[],
  gripMaxMm: number,
  cb: {
    onManual(joint: number, deg: number): void;
    /** 滑条拖出的连续开度（mm，单指行程） */
    onGripValue(mm: number): void;
    /** 开/合按钮（true=全开 / false=全闭） */
    onGrip(open: boolean): void;
    onReset(): void;
    onBadgeClick(): void;
    /** 隐藏/显示 TCP 球组（小球+球网+连线）——便于观察夹爪运动 */
    onBallToggle(hidden: boolean): void;
    onDemo(): void;
    onRec(): void;
    onReplay(): void;
    onClearPath(): void;
    onSendReal(): void;
  },
): JointPanel {
  const rows = joints
    .map(
      (j, i) => `
    <div class="jp-row">
      <div class="jp-row-head">
        <span class="jp-jname">${j.label}</span>
        <span class="jp-angles"><span class="jp-cur" id="jp-cur-${i}">0.0°</span><b class="jp-tgt" id="jp-tgt-${i}">0.0°</b></span>
      </div>
      <input type="range" class="jp-slider" id="jp-sl-${i}"
        min="${j.minDeg}" max="${j.maxDeg}" step="0.1" value="0" aria-label="${j.label} 目标角度">
    </div>`,
    )
    .join('');

  mount.innerHTML = `
    <div class="jp-head">
      <span class="jp-title">关节控制</span>
      <span class="jp-head-tools">
        <button type="button" class="jp-iconbtn" id="jp-eye" title="隐藏/显示 TCP 球（观察夹爪）" aria-pressed="false">
          <svg class="jp-eye-on" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M2 12s3.5-6.5 10-6.5S22 12 22 12s-3.5 6.5-10 6.5S2 12 2 12z"/><circle cx="12" cy="12" r="3"/></svg>
          <svg class="jp-eye-off" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M17.94 17.94A10.5 10.5 0 0 1 12 19.5C5.5 19.5 2 13 2 13a19 19 0 0 1 4.24-4.94"/><path d="M9.9 5.24A10.5 10.5 0 0 1 12 4.5c6.5 0 10 6.5 10 6.5a19 19 0 0 1-2.67 3.95"/><path d="M14.12 14.12a3 3 0 1 1-4.24-4.24"/><path d="M3 3l18 18"/></svg>
        </button>
        <span class="jp-mode" id="jp-mode" role="status" title="手动模式下点击返回 TCP 拖拽"><i class="jp-dot"></i><span id="jp-mode-text">TCP 拖拽</span></span>
      </span>
    </div>
    <div class="jp-rows">${rows}</div>
    <div class="jp-grip">
      <div class="jp-grip-head">
        <span class="jp-jname">夹爪</span>
        <span class="jp-angles"><span class="jp-cur">行程</span><b class="jp-tgt"><span id="jp-travel">0.0</span> mm</b></span>
      </div>
      <input type="range" class="jp-slider" id="jp-gsl" min="0" max="${gripMaxMm.toFixed(1)}" step="0.1" value="0"
        aria-label="夹爪开度（单指行程 mm）">
      <div class="jp-grip-btns">
        <button type="button" class="jp-btn" id="jp-open" aria-pressed="false">张开</button>
        <button type="button" class="jp-btn" id="jp-close" aria-pressed="true">闭合</button>
      </div>
    </div>
    <div class="jp-grip">
      <div class="jp-grip-head">
        <span class="jp-jname">任务 · 路径</span>
        <span class="jp-pathinfo" id="jp-pathinfo">未记录</span>
      </div>
      <button type="button" class="jp-btn jp-wide" id="jp-demo">▶ 一键夹取演示</button>
      <div class="jp-grip-btns">
        <button type="button" class="jp-btn" id="jp-rec">● 记录路径</button>
        <button type="button" class="jp-btn" id="jp-replay">↻ 回放</button>
      </div>
      <div class="jp-grip-btns">
        <button type="button" class="jp-btn" id="jp-clear">⨯ 清除路径</button>
        <button type="button" class="jp-btn" id="jp-send">⇩ 下发实体臂</button>
      </div>
    </div>
    <button type="button" class="jp-reset" id="jp-reset">
      <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" aria-hidden="true"><path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/></svg>
      复位
    </button>
  `;

  const $ = <T extends HTMLElement>(id: string): T => mount.querySelector('#' + id) as T;
  const modeEl = $('jp-mode');
  const modeText = $('jp-mode-text');
  const travelEl = $('jp-travel');
  const openBtn = $('jp-open');
  const closeBtn = $('jp-close');
  const gripSlider = $('jp-gsl') as HTMLInputElement;
  const sliders = joints.map((_, i) => $('jp-sl-' + i) as HTMLInputElement);
  const curs = joints.map((_, i) => $('jp-cur-' + i));
  const tgts = joints.map((_, i) => $('jp-tgt-' + i));

  const active = new Set<number>(); // 正被用户拖动的滑条（auto-sync 跳过）
  sliders.forEach((s, i) => {
    s.addEventListener('pointerdown', () => active.add(i));
    s.addEventListener('input', () => {
      cb.onManual(i, parseFloat(s.value));
      paintFill(i);
    });
  });
  const clearActive = (): void => active.clear();
  window.addEventListener('pointerup', clearActive);
  window.addEventListener('pointercancel', clearActive); // 触摸被系统打断也要清，否则该滑条永久失同步

  /** 滑条填充进度 + 目标角读数（value 变化后调用） */
  function paintFill(i: number): void {
    const s = sliders[i];
    const val = parseFloat(s.value);
    const span = parseFloat(s.max) - parseFloat(s.min) || 1; // 兜底除零（未限位关节）
    const pct = ((val - parseFloat(s.min)) / span) * 100;
    s.style.background = `linear-gradient(90deg, var(--jp-acc) ${pct}%, rgba(255,255,255,.10) ${pct}%)`;
    tgts[i].textContent = val.toFixed(1) + '°';
  }
  sliders.forEach((_, i) => paintFill(i));

  modeEl.addEventListener('click', () => cb.onBadgeClick()); // 手动模式下点击徽标返回 TCP 拖拽

  // TCP 球隐藏开关（观察夹爪运动时球/网/线会挡视线）
  const eyeBtn = $('jp-eye');
  eyeBtn.addEventListener('click', () => {
    const hidden = eyeBtn.classList.toggle('jp-off');
    eyeBtn.setAttribute('aria-pressed', String(hidden));
    cb.onBallToggle(hidden);
  });

  /** 夹爪滑条填充 + 开/合按钮 active 态（阈值：过半=开） */
  function paintGrip(open: boolean): void {
    const pct = (parseFloat(gripSlider.value) / (parseFloat(gripSlider.max) || 1)) * 100;
    gripSlider.style.background = `linear-gradient(90deg, var(--jp-acc) ${pct}%, rgba(255,255,255,.10) ${pct}%)`;
    openBtn.setAttribute('aria-pressed', String(open));
    closeBtn.setAttribute('aria-pressed', String(!open));
    openBtn.classList.toggle('jp-on', open);
    closeBtn.classList.toggle('jp-on', !open);
  }
  gripSlider.addEventListener('input', () => {
    const mm = parseFloat(gripSlider.value);
    paintGrip(mm >= gripMaxMm / 2);
    cb.onGripValue(mm);
  });
  openBtn.addEventListener('click', () => {
    gripSlider.value = gripMaxMm.toFixed(1);
    paintGrip(true);
    cb.onGrip(true);
  });
  closeBtn.addEventListener('click', () => {
    gripSlider.value = '0';
    paintGrip(false);
    cb.onGrip(false);
  });
  paintGrip(false);

  // ===== 任务/路径区（P4）=====
  const demoBtn = $('jp-demo');
  const recBtn = $('jp-rec');
  const replayBtn = $('jp-replay');
  const pathInfo = $('jp-pathinfo');
  demoBtn.addEventListener('click', () => cb.onDemo());
  recBtn.addEventListener('click', () => cb.onRec());
  replayBtn.addEventListener('click', () => cb.onReplay());
  $('jp-clear').addEventListener('click', () => cb.onClearPath());
  $('jp-send').addEventListener('click', () => cb.onSendReal());

  const resetBtn = $('jp-reset');
  const resetHTML = resetBtn.innerHTML;
  let resetFlashTimer = 0;
  resetBtn.addEventListener('click', () => {
    sliders.forEach((s, i) => { s.value = '0'; paintFill(i); });
    gripSlider.value = '0';
    paintGrip(false);
    cb.onReset();
    // 确认反馈：短暂变绿闪"✓ 已复位"，再恢复
    resetBtn.classList.add('jp-done');
    resetBtn.innerHTML = '✓ 已复位';
    window.clearTimeout(resetFlashTimer);
    resetFlashTimer = window.setTimeout(() => {
      resetBtn.classList.remove('jp-done');
      resetBtn.innerHTML = resetHTML;
    }, 900);
  });

  return {
    root: mount,
    setMode(manual: boolean): void {
      modeEl.classList.toggle('jp-manual', manual);
      modeText.textContent = manual ? '手动' : 'TCP 拖拽';
    },
    setGripOpen(open: boolean): void {
      gripSlider.value = open ? gripMaxMm.toFixed(1) : '0';
      paintGrip(open);
    },
    syncSliders(anglesDeg: number[]): void {
      sliders.forEach((s, i) => {
        if (active.has(i)) return;
        s.value = anglesDeg[i].toFixed(1);
        paintFill(i);
      });
    },
    syncReadouts(curDeg: number[], travelMm: number): void {
      curs.forEach((el, i) => { el.textContent = curDeg[i].toFixed(1) + '°'; });
      travelEl.textContent = travelMm.toFixed(1);
    },
    /** 演示运行态：按钮文字切换 + 停其它按钮的占用态 */
    setDemoRunning(run: boolean): void {
      demoBtn.textContent = run ? '■ 停止演示' : '▶ 一键夹取演示';
      demoBtn.classList.toggle('jp-on', run);
      recBtn.textContent = '● 记录路径';
      recBtn.classList.remove('jp-on');
    },
    setRecRunning(run: boolean): void {
      recBtn.textContent = run ? '■ 停止记录' : '● 记录路径';
      recBtn.classList.toggle('jp-on', run);
      demoBtn.textContent = '▶ 一键夹取演示';
      demoBtn.classList.remove('jp-on');
    },
    /** 路径信息行（点数/回放态） */
    setPathInfo(text: string, active: boolean): void {
      pathInfo.textContent = text;
      pathInfo.style.color = active ? 'var(--jp-acc-warn)' : 'var(--color-fog)';
    },
  };
}
