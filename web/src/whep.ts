/**
 * WHEP 拉流（P5-C）：MediaMTX WebRTC 标准播放协议。
 * OBS → MediaMTX（推流）→ 本模块 WHEP 拉流 → <video>。视频面直连 MediaMTX，不经伴生服务。
 * Rev: P5-C · 2026-10-25
 */

export interface WhepSession {
  pc: RTCPeerConnection;
  close(): void;
}

/**
 * 发起 WHEP 拉流并绑定到 video 元素。
 * @param whepUrl MediaMTX 的 WHEP 端点（如 http://192.168.1.100:8889/pico/whep）
 * @param video 目标 video 元素（srcObject 由 ontrack 绑定）
 * @throws 网络/SDP 协商失败抛错（调用方降级到预录视频/占位层）
 */
export async function pullWhep(whepUrl: string, video: HTMLVideoElement): Promise<WhepSession> {
  const pc = new RTCPeerConnection();
  // 收流方向：只收视频+音频（MediaMTX 按收发器协商）
  pc.addTransceiver('video', { direction: 'recvonly' });
  pc.addTransceiver('audio', { direction: 'recvonly' });
  pc.ontrack = (e): void => {
    video.srcObject = e.streams[0];
    void video.play().catch(() => { /* autoplay 被拦时静默，用户点击全屏后可恢复 */ });
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);
  // MediaMTX 非 trickle ICE：等收集完成再发 offer（含 candidates）
  if (pc.iceGatheringState !== 'complete') {
    await new Promise<void>((res) => {
      const done = (): void => {
        if (pc.iceGatheringState === 'complete') {
          pc.removeEventListener('icegatheringstatechange', done);
          res();
        }
      };
      pc.addEventListener('icegatheringstatechange', done);
      window.setTimeout(done, 1500); // 兜底：收集慢就带已有 candidates 发
    });
  }

  const res = await fetch(whepUrl, {
    method: 'POST',
    headers: { 'Content-Type': 'application/sdp' },
    body: pc.localDescription?.sdp ?? '',
  });
  if (!res.ok) throw new Error(`WHEP ${res.status}: ${await res.text().catch(() => '')}`);
  const answerSdp = await res.text();
  await pc.setRemoteDescription({ type: 'answer', sdp: answerSdp });

  return {
    pc,
    close(): void {
      pc.close();
      video.srcObject = null;
    },
  };
}
