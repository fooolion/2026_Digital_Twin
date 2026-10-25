/**
 * 共享模块：顶部导航（工作流步骤式：总览/①零件库/②仿真规划/③遥操作/④运行监控/关于）
 *           + 跨页组件（toast / renderStatList 状态列表）。
 * Rev: R2 · 2026-10-25
 */

const LINKS = [
  { href: 'index.html', key: 'home', label: '总览', no: null },
  { href: 'workshop.html', key: 'workshop', label: '零件库', no: 1 },
  { href: 'sim.html', key: 'sim', label: '仿真规划', no: 2 },
  { href: 'teleop.html', key: 'teleop', label: '遥操作', no: 3 },
  { href: 'live.html', key: 'live', label: '运行监控', no: 4 },
  { href: 'about.html', key: 'about', label: '关于', no: null },
];

export function mountNav(active: string): void {
  const host = document.getElementById('site-nav');
  if (!host) return;
  host.innerHTML = `
    <nav class="site-nav">
      <a class="brand" href="index.html"><i></i>报废电池拆解数字孪生</a>
      <div class="links">
        ${LINKS.map((l) => {
          const no = l.no !== null ? `<span class="step-no">${l.no}</span>` : '';
          return `<a href="${l.href}" class="${l.key === active ? 'active' : ''}">${no}${l.label}</a>`;
        }).join('')}
      </div>
      <a class="gh" href="https://github.com/fooolion/2026_Digital_Twin" target="_blank" rel="noreferrer">GitHub ↗</a>
    </nav>`;

  const el = host.firstElementChild as HTMLElement;
  const onScroll = (): void => {
    el.classList.toggle('scrolled', scrollY > 8);
  };
  addEventListener('scroll', onScroll, { passive: true });
  onScroll();
}

/** 轻提示（2.4s 自动消失） */
export function toast(msg: string): void {
  let t = document.getElementById('toast');
  if (!t) {
    t = document.createElement('div');
    t.id = 'toast';
    document.body.appendChild(t);
  }
  t.textContent = msg;
  t.classList.add('show');
  window.setTimeout(() => t!.classList.remove('show'), 2400);
}

/** 状态列表渲染（跨页组件：总览/监控/遥操作；样式 .stat-list 在 theme.css）。
 *  P5 接入真链路后，三页只需把 STATS 换成轮询数据再调本函数。 */
export interface StatItem { name: string; lamp: 'ok' | 'warn' | 'off'; state: string }
const STAT_LAMP_CLS: Record<string, string> = { ok: '', warn: 'lamp-warn', off: 'lamp-alarm' };
const STAT_VAL_CLS: Record<string, string> = { ok: 'on', warn: 'warn', off: '' };
export function renderStatList(el: HTMLElement, items: StatItem[]): void {
  el.innerHTML = items.map((s) => `
    <li><span class="lamp ${STAT_LAMP_CLS[s.lamp]}"></span><b>${s.name}</b><span class="val ${STAT_VAL_CLS[s.lamp]}">${s.state}</span></li>`).join('');
}
