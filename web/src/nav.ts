/**
 * 共享顶部导航：每个页面放 <header id="site-nav"></header>，再调用 mountNav(activeKey)。
 * 2026-10-25 R2 重组：按指导老师工作流改为步骤式导航（总览/①零件库/②仿真规划/③遥操作/④运行监控/关于）。
 * 文件名保持不变（sim/live/teleop/workshop）以兼容已发布的外链，仅导航文案与顺序变化。
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

/** 滚动显现：给 .reveal 元素挂 IntersectionObserver（各页复用） */
export function bindReveal(): void {
  const io = new IntersectionObserver(
    (es) => es.forEach((e) => {
      if (e.isIntersecting) { e.target.classList.add('in'); io.unobserve(e.target); }
    }),
    { threshold: 0.12 },
  );
  document.querySelectorAll('.reveal').forEach((el) => io.observe(el));
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
