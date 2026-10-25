/**
 * 「关于」页（宣传内容页）：2026-10-25 R2 从首页迁出。
 * Hero 入场 + GSAP/Lenis 滚动叙事 + 流程连线动画。
 */
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';
import Lenis from 'lenis';
import { mountNav } from './nav';
import './landing.css';
import './theme.css';

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

  // 素材占位：轻微视差（交替上下）
  gsap.utils.toArray<HTMLElement>('.mat-ph').forEach((el, i) => {
    gsap.to(el, {
      y: i % 2 ? 16 : -16, ease: 'none',
      scrollTrigger: { trigger: el, scrub: 1.2 },
    });
  });
}
