/**
 * 伴生服务探测（P5-A）：网页启动时探测本机 FastAPI 伴生服务（web/server/main.py），
 * 探测到 → 实时模式（状态灯转绿、后续接 WS 桥）；探测不到 → 演示模式（保持占位态）。
 * Rev: P5-A · 2026-10-25
 */
export interface CompanionHealth {
  ok: boolean;
  uptime_s: number;
  bridge_clients: number;
  dist: boolean;
}

export const COMPANION_BASE = 'http://127.0.0.1:8800';

/** 探测伴生服务（800ms 超时；失败返回 null = 演示模式） */
export async function probeCompanion(): Promise<CompanionHealth | null> {
  try {
    const ctrl = new AbortController();
    const t = window.setTimeout(() => ctrl.abort(), 800);
    const res = await fetch(`${COMPANION_BASE}/api/health`, { signal: ctrl.signal });
    window.clearTimeout(t);
    if (!res.ok) return null;
    return (await res.json()) as CompanionHealth;
  } catch {
    return null; // 服务未启动 / 断网 / 跨域被拦——一律演示模式
  }
}

/** 连接 WS 桥（P5-D 遥操作下行用；骨架阶段只做 hello+收 state 打日志验证） */
export function connectBridge(
  onState: (payload: unknown, src: string) => void,
): WebSocket | null {
  try {
    const ws = new WebSocket(`ws://127.0.0.1:8800/ws`.replace('http', 'ws'));
    ws.addEventListener('open', () => ws.send(JSON.stringify({ type: 'hello', role: 'web' })));
    ws.addEventListener('message', (e) => {
      try {
        const msg = JSON.parse(e.data as string) as { type: string; src?: string; payload?: unknown };
        if (msg.type === 'state') onState(msg.payload, msg.src ?? '');
      } catch { /* 非 JSON 忽略 */ }
    });
    return ws;
  } catch {
    return null;
  }
}
