/**
 * 共享顶部导航：每个页面放 <header id="site-nav"></header>，再调用 mountNav(activeKey)。
 * 滚动时底色加深（.scrolled），当前页链接高亮。
 */

const LINKS = [
  { href: 'index.html', key: 'home', label: '首页' },
  { href: 'live.html', key: 'live', label: '直播' },
  { href: 'sim.html', key: 'sim', label: '仿真' },
  { href: 'teleop.html', key: 'teleop', label: '遥操作' },
  { href: 'workshop.html', key: 'workshop', label: '创意工坊' },
];

export function mountNav(active: string): void {
  const host = document.getElementById('site-nav');
  if (!host) return;
  host.innerHTML = `
    <nav class="site-nav">
      <a class="brand" href="index.html"><i></i>报废电池拆解数字孪生</a>
      <div class="links">
        ${LINKS.map((l) => `<a href="${l.href}" class="${l.key === active ? 'active' : ''}">${l.label}</a>`).join('')}
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
