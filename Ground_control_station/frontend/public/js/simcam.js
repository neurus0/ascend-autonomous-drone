/**
 * simcam.js  –  Simulated camera canvas
 * Renders a realistic-looking "offline" camera feed with:
 *   • scanline animation
 *   • moving targeting reticle
 *   • noise grain
 *   • grid overlay
 * Shown only when the real MJPEG stream fails.
 */

const SimCam = (() => {
  let _canvas, _ctx;
  let _t = 0;
  let _active = false;

  function init() {
    _canvas = document.getElementById('sim-cam-canvas');
    if (!_canvas) return;
    _ctx = _canvas.getContext('2d');
    _resize();
    window.addEventListener('resize', _resize);
    _active = true;
    _loop();
  }

  function _resize() {
    if (!_canvas) return;
    const parent = _canvas.parentElement;
    _canvas.width  = parent.offsetWidth  * devicePixelRatio;
    _canvas.height = parent.offsetHeight * devicePixelRatio;
  }

  function _loop() {
    if (_active) {
      _draw();
      requestAnimationFrame(_loop);
    }
  }

  function _draw() {
    const ctx = _ctx;
    if (!ctx) return;
    const W = _canvas.width;
    const H = _canvas.height;
    _t += 0.03;
    const t = _t;

    // Clear
    ctx.fillStyle = '#040a0e';
    ctx.fillRect(0, 0, W, H);

    // ── Subtle grid ───────────────────────────────────────
    const gridSize = 30 * devicePixelRatio;
    ctx.strokeStyle = 'rgba(0,232,122,0.05)';
    ctx.lineWidth   = 0.5;
    for (let x = 0; x < W; x += gridSize) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }
    for (let y = 0; y < H; y += gridSize) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }

    // ── Noise grain ───────────────────────────────────────
    const imgData = ctx.getImageData(0, 0, W, H);
    const d = imgData.data;
    for (let i = 0; i < d.length; i += 16) {
      const n = Math.random() * 15;
      d[i] += n; d[i+1] += n; d[i+2] += n;
    }
    ctx.putImageData(imgData, 0, 0);

    // ── Scan line ─────────────────────────────────────────
    const sl = ((t * 40) % H);
    const grad = ctx.createLinearGradient(0, sl - 8, 0, sl + 8);
    grad.addColorStop(0,   'rgba(0,232,122,0)');
    grad.addColorStop(0.5, 'rgba(0,232,122,0.12)');
    grad.addColorStop(1,   'rgba(0,232,122,0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, sl - 8, W, 16);

    // ── Moving reticle ────────────────────────────────────
    const rx = W / 2 + Math.sin(t * 0.4) * W * 0.2;
    const ry = H / 2 + Math.cos(t * 0.25) * H * 0.2;
    const rs = 20 * devicePixelRatio;
    const rArm = 14 * devicePixelRatio;
    const rGap = 7  * devicePixelRatio;

    ctx.strokeStyle = '#ffb300';
    ctx.lineWidth   = 1.2 * devicePixelRatio;

    // Main circle
    ctx.beginPath();
    ctx.arc(rx, ry, rs, 0, Math.PI * 2);
    ctx.stroke();

    // Cross arms
    ctx.beginPath();
    ctx.moveTo(rx - rArm - rGap, ry);
    ctx.lineTo(rx - rGap,        ry);
    ctx.moveTo(rx + rGap,        ry);
    ctx.lineTo(rx + rArm + rGap, ry);
    ctx.moveTo(rx, ry - rArm - rGap);
    ctx.lineTo(rx, ry - rGap);
    ctx.moveTo(rx, ry + rGap);
    ctx.lineTo(rx, ry + rArm + rGap);
    ctx.stroke();

    // ── Corner brackets ───────────────────────────────────
    const bLen = 18 * devicePixelRatio;
    const bOff = 14 * devicePixelRatio;
    ctx.strokeStyle = 'rgba(0,232,122,0.4)';
    ctx.lineWidth   = 1 * devicePixelRatio;
    [
      [bOff,     bOff,     1,  1],
      [W-bOff,   bOff,    -1,  1],
      [bOff,     H-bOff,   1, -1],
      [W-bOff,   H-bOff,  -1, -1],
    ].forEach(([x, y, sx, sy]) => {
      ctx.beginPath();
      ctx.moveTo(x + sx * bLen, y);
      ctx.lineTo(x, y);
      ctx.lineTo(x, y + sy * bLen);
      ctx.stroke();
    });

    // ── HUD overlays ──────────────────────────────────────
    ctx.fillStyle = 'rgba(0,232,122,0.25)';
    ctx.font = `${8 * devicePixelRatio}px JetBrains Mono`;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('● SIM FEED', bOff, bOff + bLen + 6 * devicePixelRatio);

    ctx.textAlign = 'right';
    ctx.fillText(new Date().toLocaleTimeString(), W - bOff, bOff + bLen + 6 * devicePixelRatio);
  }

  return { init };
})();
