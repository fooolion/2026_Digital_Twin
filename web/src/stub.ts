import { mountNav, bindReveal, toast } from './nav';
import './theme.css';

// data-page: live / teleop / workshop
mountNav(document.body.dataset.page ?? '');
bindReveal();

// 创意工坊的导入按钮（占位）：真实导入在后续阶段接入
document.getElementById('import-btn')?.addEventListener('click', () => {
  toast('创意工坊施工中 · 模型导入即将上线');
});
