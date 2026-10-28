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

/** 页面 key → href 单源（各页流程条/跳转按钮复用，勿再手写文件名） */
export const PAGE_HREF: Record<string, string> = Object.fromEntries(LINKS.map((l) => [l.key, l.href]));

export function mountNav(active: string): void {
  const host = document.getElementById('site-nav');
  if (!host) return;
  host.innerHTML = `
    <nav class="site-nav">
      <a class="brand" href="index.html"><svg class="brand-ico" viewBox="0 0 512 512" aria-hidden="true"><path fill="#2ee584" d="M329.78 20.344c-.842.003-1.687.005-2.53.03-25.534.782-51.27 11.864-71.53 32.064-40.637 40.512-45.503 99.74-12.75 162.156 7.468-7.77 17.948-12.625 29.5-12.625 1.63 0 3.228.123 4.81.31-7.385-12.674-12.023-25.99-13.467-39.686-2.692-25.526 5.91-51.74 25.906-75.594 17.222-20.545 42.123-29.6 66.967-32.313 15.924-1.738 31.875-.833 46.72 1.25-21.29-24.748-47.497-35.683-73.626-35.593zm127.94 89.75c2.057 14.84 2.9 30.782 1.06 46.687-2.86 24.738-12.102 49.482-32.686 66.626-23.92 19.92-50.076 28.506-75.563 25.813-12.917-1.366-25.496-5.572-37.53-12.25.288 1.948.438 3.942.438 5.968 0 11.155-4.536 21.308-11.844 28.718C363.07 302.49 421.276 296.65 461.28 256.78c23.21-23.127 34.594-52.96 31.97-81.81-2.11-23.183-13.242-45.94-35.53-64.876zM272.47 220.656c-12.417 0-22.283 9.866-22.283 22.28 0 12.417 9.866 22.283 22.282 22.283 12.414 0 22.28-9.867 22.28-22.283 0-12.415-9.866-22.28-22.28-22.28zm-39.72 12.25C161.344 259.196 83.694 294.76 19.094 348.03v82.376c67.566-58.922 143.974-114.322 208.72-145.25-31.175 64.465-87.04 140.404-146.158 207.688h78.72c52.763-63.457 90.673-139.32 118-209.375-1.93.28-3.903.436-5.907.436-22.517 0-40.97-18.452-40.97-40.97 0-3.46.43-6.812 1.25-10.03z"/></svg>报废电池拆解数字孪生</a>
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
