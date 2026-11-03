/**
 * 关于页（R7，2026-10-27）：Hero / 痛点 / 四环节 / 系统构成 / 亮点 / 系统实拍 / CTA。
 * 设计原则：不用二级标签与冗余副标，标题 + 排版自说明。
 * 动效：GSAP + Lenis 滚动叙事；reduced-motion 时内容直接可见。
 */
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';
import Lenis from 'lenis';
import { mountNav } from './nav';
import './theme.css';
import './landing.css';

mountNav('about');
gsap.registerPlugin(ScrollTrigger);

// 系统关动画（无障碍）：跳过全部动效，内容直接可见
if (matchMedia('(prefers-reduced-motion: reduce)').matches) {
  document.querySelectorAll<HTMLElement>('.flow-line-fill').forEach((e) => (e.style.transform = 'scaleX(1)'));
} else {
  // Lenis 惯性平滑滚动，与 ScrollTrigger 同帧驱动
  const lenis = new Lenis({ lerp: 0.1 });
  lenis.on('scroll', ScrollTrigger.update);
  gsap.ticker.add((time) => lenis.raf(time * 1000));
  gsap.ticker.lagSmoothing(0);

  // Hero 入场：徽章→标题→副标→按钮→数据卡 依次浮入
  gsap.from('.hero [data-hero]', {
    y: 26, opacity: 0, duration: 0.9, stagger: 0.12, ease: 'power3.out', delay: 0.15,
  });

  // 通用：带 data-rise 的元素入视口上浮
  gsap.utils.toArray<HTMLElement>('[data-rise]').forEach((el) => {
    gsap.from(el, {
      y: 34, opacity: 0, duration: 0.8, ease: 'power3.out',
      scrollTrigger: { trigger: el, start: 'top 84%' },
    });
  });

  // 卡片组：组内子元素依次浮入
  gsap.utils.toArray<HTMLElement>('[data-stagger]').forEach((wrap) => {
    gsap.from(wrap.children, {
      y: 30, opacity: 0, duration: 0.7, stagger: 0.1, ease: 'power3.out',
      scrollTrigger: { trigger: wrap, start: 'top 84%' },
    });
  });

  // 四环节流程：连线生长 + 步骤依次点亮
  gsap.from('.flow-line-fill', {
    scaleX: 0, transformOrigin: 'left center', duration: 1.3, ease: 'power2.inOut',
    scrollTrigger: { trigger: '#flow', start: 'top 68%' },
  });
  gsap.from('.flow-step', {
    y: 24, opacity: 0, duration: 0.7, stagger: 0.24, ease: 'power3.out',
    scrollTrigger: { trigger: '#flow', start: 'top 68%' },
  });

}

/* ===== 系统实拍视频：data-clip → BASE_URL/videos/ 下取片，进视口才加载播放 ===== */

const VID_BASE = `${import.meta.env.BASE_URL}videos/`;
const clips = Array.from(document.querySelectorAll<HTMLVideoElement>('.mat-vid video'));

if (matchMedia('(prefers-reduced-motion: reduce)').matches) {
  // 系统关动画：只加载首帧（preload="metadata"），不自动播放
  clips.forEach((v) => (v.src = VID_BASE + v.dataset.clip));
} else {
  const inView = new WeakSet<HTMLVideoElement>();
  const io = new IntersectionObserver(
    (entries) => {
      for (const e of entries) {
        const v = e.target as HTMLVideoElement;
        if (e.isIntersecting) {
          inView.add(v);
          if (!v.src) v.src = VID_BASE + v.dataset.clip; // 懒加载：滚到才拉流
          void v.play().catch(() => { /* 自动播放拦截等：静默 */ });
        } else {
          inView.delete(v);
          v.pause();
        }
      }
    },
    { threshold: 0.25 },
  );
  clips.forEach((v) => io.observe(v));

  // 页面隐藏统一暂停、回来只恢复视口内的（省后台解码，与 live 页同策略）
  document.addEventListener('visibilitychange', () => {
    clips.forEach((v) => (document.hidden || !inView.has(v) ? v.pause() : v.play().catch(() => {})));
  });
}
