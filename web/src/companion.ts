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

// M1：由伴生服务本体托管时（http + 8800 端口）必须用同源 origin——
// 局域网设备直开 http://本机IP:8800/ 时，硬编码 127.0.0.1 会解析到设备自身（永远探测失败）。
// GitHub Pages（https）部署时才回落 127.0.0.1（比赛 PC 本机）。
export const COMPANION_BASE =
  location.protocol === 'http:' && location.port === '8800' ? location.origin : 'http://127.0.0.1:8800';

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
  onState: (payload: unknown, src: string, ts: number) => void,
  onDown?: (reason: string) => void, // 连接失败/关闭回调（调用方可做重连）
  onUp?: () => void,                 // 握手完成（welcome 收到）——状态灯"转绿"应等此回调
): WebSocket | null {
  try {
    // 探测与桥共用同一 base（改端口只动 COMPANION_BASE）
    const ws = new WebSocket(COMPANION_BASE.replace(/^http/, 'ws') + '/ws');
    ws.addEventListener('open', () => ws.send(JSON.stringify({ type: 'hello', role: 'web' })));
    ws.addEventListener('message', (e) => {
      try {
        const msg = JSON.parse(e.data as string) as { type: string; src?: string; payload?: unknown; ts?: number };
        if (msg.type === 'welcome') onUp?.();
        if (msg.type === 'state') onState(msg.payload, msg.src ?? '', msg.ts ?? Date.now() / 1000);
      } catch { /* 非 JSON 忽略 */ }
    });
    ws.addEventListener('close', () => onDown?.('closed'));
    ws.addEventListener('error', () => onDown?.('error'));
    return ws;
  } catch {
    return null;
  }
}

/** 周期重探（watch 模式）：服务后启动也能转绿；结果变化才回调（省重渲染）。
 *  返回停止函数。interval 默认 10s；online 事件即时重探。 */
export function watchCompanion(
  onChange: (h: CompanionHealth | null) => void,
  intervalMs = 10_000,
): () => void {
  let stopped = false;
  const probeOnce = async (): Promise<void> => {
    if (stopped) return;
    const h = await probeCompanion();
    const ok = !!h?.ok;
    if (ok !== lastProbeOk) {
      lastProbeOk = ok;
      onChange(h);   // 状态翻转才回调（null→ok 的首次也算）
    }
  };
  let lastProbeOk: boolean | null = null;
  const timer = window.setInterval(() => void probeOnce(), intervalMs);
  const onOnline = (): void => void probeOnce();
  addEventListener('online', onOnline);
  void probeOnce();
  return () => { // 停止（单页常驻，骨架期不调用）
    stopped = true;
    window.clearInterval(timer);
    removeEventListener('online', onOnline);
  };
}
