/**
 * Artificial Horizon Indicator (AHI) + Compass Strip
 * Draws roll/pitch as a physical horizon on canvas
 */

const AHI = (() => {
  const COLORS = {
    sky:     '#0a1e3a',
    skyHigh: '#071428',
    gnd:     '#3a1800',
    gndLow:  '#220f00',
    horizon: '#00dc8a',
    ladder:  'rgba(0,220,138,0.5)',
    cross:   '#f59e0b',
    border:  'rgba(0,220,138,0.3)',
    text:    'rgba(0,220,138,0.8)',
  };

  function drawAHI(canvas, roll, pitch, yaw) {
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const cx = W / 2, cy = H / 2;
    const R = Math.min(W, H) * 0.44;

    ctx.clearRect(0, 0, W, H);
    ctx.save();

    // Clip to circle
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.clip();

    // Rotate for roll
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(roll * Math.PI / 180);

    const pitchPx = (pitch / 90) * R;

    // Sky gradient
    const skyGrad = ctx.createLinearGradient(0, -R, 0, pitchPx);
    skyGrad.addColorStop(0, COLORS.skyHigh);
    skyGrad.addColorStop(1, COLORS.sky);
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-R * 1.5, -R * 1.5, R * 3, R * 1.5 + pitchPx);

    // Ground gradient
    const gndGrad = ctx.createLinearGradient(0, pitchPx, 0, R);
    gndGrad.addColorStop(0, COLORS.gnd);
    gndGrad.addColorStop(1, COLORS.gndLow);
    ctx.fillStyle = gndGrad;
    ctx.fillRect(-R * 1.5, pitchPx, R * 3, R * 1.5);

    // Horizon line
    ctx.strokeStyle = COLORS.horizon;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-R, pitchPx);
    ctx.lineTo(R, pitchPx);
    ctx.stroke();

    // Pitch ladder
    ctx.strokeStyle = COLORS.ladder;
    ctx.lineWidth = 0.8;
    ctx.fillStyle = COLORS.text;
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.textAlign = 'center';

    for (let deg = -30; deg <= 30; deg += 10) {
      if (deg === 0) continue;
      const yOff = pitchPx + (deg / 90) * R;
      const len = Math.abs(deg) === 30 ? 30 : (Math.abs(deg) === 20 ? 24 : 16);
      ctx.beginPath();
      ctx.moveTo(-len, yOff);
      ctx.lineTo(len, yOff);
      ctx.stroke();
      ctx.fillText(`${Math.abs(deg)}`, len + 8, yOff + 3);
      ctx.fillText(`${Math.abs(deg)}`, -len - 8, yOff + 3);
    }

    // Roll indicator marks at top
    ctx.strokeStyle = COLORS.horizon;
    ctx.lineWidth = 1;
    [30, 45, 60].forEach(a => {
      [-1, 1].forEach(s => {
        const rad = (a * s) * Math.PI / 180;
        const x1 = Math.sin(rad) * (R - 6), y1 = -Math.cos(rad) * (R - 6);
        const x2 = Math.sin(rad) * (R - 14), y2 = -Math.cos(rad) * (R - 14);
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
      });
    });

    ctx.restore();

    // Fixed aircraft crosshair (not rotated)
    ctx.strokeStyle = COLORS.cross;
    ctx.lineWidth = 2;
    const cLen = 36, cGap = 10;
    ctx.beginPath();
    ctx.moveTo(cx - cLen - cGap, cy); ctx.lineTo(cx - cGap, cy);
    ctx.moveTo(cx + cGap, cy);        ctx.lineTo(cx + cLen + cGap, cy);
    ctx.moveTo(cx, cy - 8);           ctx.lineTo(cx, cy + 8);
    ctx.stroke();
    // Center dot
    ctx.fillStyle = COLORS.cross;
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fill();

    // Roll indicator pointer at top
    const rollRad = roll * Math.PI / 180;
    const px = cx + Math.sin(rollRad) * (R - 4);
    const py = cy - Math.cos(rollRad) * (R - 4);
    ctx.fillStyle = COLORS.cross;
    ctx.beginPath();
    ctx.arc(px, py, 3, 0, Math.PI * 2);
    ctx.fill();

    // Circle border
    ctx.restore();
    ctx.strokeStyle = COLORS.border;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.stroke();
  }

  function drawCompass(canvas, yaw) {
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    // Background
    ctx.fillStyle = 'rgba(12,18,32,0.95)';
    ctx.beginPath();
    ctx.roundRect(0, 0, W, H, 4);
    ctx.fill();
    ctx.strokeStyle = 'rgba(0,220,138,0.2)';
    ctx.lineWidth = 0.5;
    ctx.stroke();

    const DIRS = [
      { deg:0,lbl:'N' }, { deg:45,lbl:'NE' }, { deg:90,lbl:'E' },
      { deg:135,lbl:'SE'}, { deg:180,lbl:'S' }, { deg:225,lbl:'SW' },
      { deg:270,lbl:'W' }, { deg:315,lbl:'NW' }
    ];

    const scale = W / 180;

    DIRS.forEach(d => {
      let offset = ((d.deg - yaw + 180 + 360) % 360) - 180;
      const x = W / 2 + offset * scale;
      if (x < -10 || x > W + 10) return;
      const isCardinal = d.lbl.length === 1;
      ctx.fillStyle = d.lbl === 'N' ? '#ef4444' : isCardinal ? 'rgba(0,220,138,0.9)' : 'rgba(0,220,138,0.45)';
      ctx.font = `${isCardinal ? 500 : 400} ${isCardinal ? 11 : 9}px Rajdhani, sans-serif`;
      ctx.textAlign = 'center';
      ctx.fillText(d.lbl, x, H / 2 + 4);
    });

    // Center tick
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(W / 2, 2);
    ctx.lineTo(W / 2, 10);
    ctx.stroke();

    // Degree label
    ctx.fillStyle = 'rgba(0,220,138,0.7)';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(Math.round(yaw) + '°', W / 2, H - 4);
  }

  return { drawAHI, drawCompass };
})();
