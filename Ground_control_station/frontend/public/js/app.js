/**
 * app.js  –  Drone GCS Frontend Application (TCP Version)
 * =========================================================
 * IRoC-U 2026 — GPS-Denied Autonomous Flight
 * 
 * CHANGES FROM ORIGINAL:
 *   - Added disarmDrone() function
 *   - LiDAR altitude displayed from telemetry.lidar_alt
 *   - Jetson TCP connection status indicator
 *   - VIO status badge
 *   - Flight state badge
 *   - Jetson log messages displayed in system log
 */

/* ── DOM helpers ──────────────────────────────────────────────── */
const $ = (id) => document.getElementById(id);
const set = (id, val) => { const el = $(id); if (el) el.textContent = val; };

/* ── State ─────────────────────────────────────────────────────── */
let socket = null;
let packetCount = 0;
let lastPingTime = 0;
let flightTimeSec = 0;
let prevArmed = false;
let chartPushTick = 0;
let _camLive = false;
let _archiveRows = [];
let _pingInterval = null;
let _statusInterval = null;

/* ── Initialise ────────────────────────────────────────────────── */
document.addEventListener('DOMContentLoaded', () => {
  if (typeof AHI !== 'undefined') AHI.init();
  if (typeof Charts !== 'undefined') Charts.init();
  if (typeof SimCam !== 'undefined') SimCam.init();
  _startClocks();
  connect();
  _loadMissionArchive();
});

/* ── Clock ─────────────────────────────────────────────────────── */
function _startClocks() {
  setInterval(() => {
    const c = $('uptime-label');
    if (c) c.textContent = new Date().toLocaleTimeString('en-GB', { hour12: false });
  }, 1000);
}

/* ══════════════════════════════════════════════════════════════
   TASK 1 — START / KILL / DISARM MISSION
   Commands sent to backend → backend forwards via TCP to Jetson
   ══════════════════════════════════════════════════════════════ */
function startMission() {
  if (!socket || !socket.connected) {
    alert('Not connected to GCS server. Press CONNECT first.');
    return;
  }
  if (!confirm('Send START MISSION to Jetson?\nDrone will arm → takeoff → hover → land.')) return;

  socket.emit('command', { cmd: 'START_MISSION', params: {} });
  _syslog('MISSION START command sent → Jetson', 'ok');

  const btn = $('start-btn');
  if (btn) {
    btn.textContent = '✓ SENT';
    btn.classList.add('sent');
    btn.disabled = true;
    setTimeout(() => {
      btn.textContent = '▶ START';
      btn.classList.remove('sent');
      btn.disabled = false;
    }, 5000);
  }
}

function killMission() {
  if (!socket || !socket.connected) {
    alert('Not connected. Press CONNECT first.');
    return;
  }
  // No confirm for KILL — it's an emergency button
  socket.emit('command', { cmd: 'KILL', params: {} });
  _syslog('⚠ KILL SWITCH ACTIVATED — Emergency descent commanded', 'failure');

  const btn = $('kill-btn');
  if (btn) {
    btn.textContent = '✓ KILL SENT';
    setTimeout(() => { btn.textContent = '✕ KILL'; }, 4000);
  }
}

function disarmDrone() {
  if (!socket || !socket.connected) {
    alert('Not connected. Press CONNECT first.');
    return;
  }
  if (!confirm('⚡ DISARM the drone?\n\nThis will immediately cut all motors!\nOnly use this as a LAST RESORT when the drone is on the ground.')) return;

  socket.emit('command', { cmd: 'DISARM', params: {} });
  _syslog('⚡ DISARM command sent — hardware emergency stop', 'failure');

  const btn = $('disarm-btn');
  if (btn) {
    btn.textContent = '⚡ DISARMING...';
    btn.disabled = true;
    setTimeout(() => {
      btn.textContent = '⚡ DISARM';
      btn.disabled = false;
    }, 5000);
  }
}

/* ── Send generic command ─────────────────────────────────────── */
function sendCommand(cmd, params = {}) {
  if (socket && socket.connected) {
    socket.emit('command', { cmd, params });
    _syslog(`CMD: ${cmd}`, 'info');
  } else {
    alert('Not connected — press CONNECT first.');
  }
}

/* ── Toggle Connect ───────────────────────────────────────────── */
function toggleConnect() {
  if (socket && socket.connected) {
    socket.disconnect();
  } else {
    connect();
  }
}

/* ══════════════════════════════════════════════════════════════
   CAMERA
   ══════════════════════════════════════════════════════════════ */
function toggleCamMode() {
  _camLive = !_camLive;
  const simCanvas = $('sim-cam');
  const liveImg = $('live-cam');
  if (!simCanvas || !liveImg) return;

  if (_camLive) {
    const rawUrl = ($('cam-url-input') || {}).value || '';
    if (!rawUrl) {
      alert('Type the camera URL first (e.g. http://192.168.70.108:5000/)');
      _camLive = false;
      return;
    }
    const proxyUrl = '/camera-proxy?url=' + encodeURIComponent(rawUrl);
    liveImg.src = proxyUrl;
    liveImg.style.display = 'block';
    simCanvas.style.display = 'none';
    if (typeof SimCam !== 'undefined' && SimCam.stop) SimCam.stop();

    let _fpsTimer = null;
    let _frameCount = 0;
    liveImg.onload = () => { _frameCount++; };
    clearInterval(_fpsTimer);
    _fpsTimer = setInterval(() => {
      set('cam-fps-label', _frameCount + ' fps');
      _frameCount = 0;
    }, 1000);
    liveImg.onerror = () => { set('cam-fps-label', 'ERR – check URL'); };
  } else {
    liveImg.src = '';
    liveImg.style.display = 'none';
    simCanvas.style.display = 'block';
    if (typeof SimCam !== 'undefined' && SimCam.init) SimCam.init();
    set('cam-fps-label', '-- fps');
  }
}

/* ══════════════════════════════════════════════════════════════
   CAMERA LOGS — always visible table, rows appended live
   ══════════════════════════════════════════════════════════════ */
async function _loadMissionArchive() {
  try {
    const res = await fetch('/api/mission-data');
    if (!res.ok) return;
    const rows = await res.json();
    rows.forEach(r => _appendArchiveRow(r));
  } catch (e) { /* server not yet running */ }
}

function _appendArchiveRow(row) {
  _archiveRows.push(row);
  const count = _archiveRows.length;
  set('archive-count', count + ' record' + (count !== 1 ? 's' : ''));

  const tbody = $('archive-tbody');
  if (!tbody) return;
  const tr = document.createElement('tr');
  const imgCell = row.image_url
    ? `<img class="thumb" src="${row.image_url}" alt="img" title="${row.image_file || ''}" onclick="openLightbox('${row.image_url}')" style="cursor:pointer;"/>`
    : `<span class="no-img">—</span>`;
  const locStr = row.location
    ? `${Number(row.location.x).toFixed(2)},${Number(row.location.y).toFixed(2)}`
    : '—';
  const velStr = row.velocity !== undefined ? Number(row.velocity).toFixed(1) : '—';
  tr.innerHTML = `
    <td>${row.date || '—'}</td>
    <td>${row.time || '—'}</td>
    <td>${imgCell}</td>
    <td class="loc">${locStr}</td>
    <td>${velStr}</td>`;
  tbody.insertBefore(tr, tbody.firstChild);

  const wrap = $('archive-table-wrap');
  if (wrap) wrap.scrollTop = 0;
}

/* ══════════════════════════════════════════════════════════════
   SYSTEM LOG & FAILURES
   ══════════════════════════════════════════════════════════════ */
function _syslog(msg, level = 'info') {
  const el = $('syslog-scroll');
  if (!el) return;
  const ts = new Date().toLocaleTimeString('en-GB', { hour12: false });
  const line = document.createElement('div');
  line.className = 'log-line ' + level;

  if (level === 'failure') {
    line.innerHTML = `<span class="log-ts">${ts}</span><span class="log-fail-tag">FAIL</span>${msg}`;
  } else {
    line.innerHTML = `<span class="log-ts">${ts}</span>${msg}`;
  }
  el.prepend(line);
  while (el.children.length > 200) el.removeChild(el.lastChild);
  el.scrollTop = 0;
}

function clearSysLog() {
  const el = $('syslog-scroll');
  if (el) el.innerHTML = '';
}

/* ── Socket.io connection ─────────────────────────────────────── */
function connect() {
  const wsInput = $('ws-url') || $('ws-input');
  const url = wsInput ? wsInput.value.trim() : 'ws://127.0.0.1:3000';

  if (socket) { socket.disconnect(); socket = null; }
  if (_pingInterval) { clearInterval(_pingInterval); _pingInterval = null; }
  if (_statusInterval) { clearInterval(_statusInterval); _statusInterval = null; }
  _setStatus('connecting', 'CONNECTING…');

  socket = io(url, {
    transports: ['websocket'],
    reconnectionDelay: 2000,
    timeout: 5000,
  });

  socket.on('connect', () => {
    _setStatus('ok', 'CONNECTED · LIVE');
    socket.emit('get_history', 60);
    _ping();
    if (_pingInterval) clearInterval(_pingInterval);
    _pingInterval = setInterval(_ping, 5000);
    _fetchStatus();
    if (_statusInterval) clearInterval(_statusInterval);
    _statusInterval = setInterval(_fetchStatus, 10000);
    _syslog('WebSocket connected to ' + url, 'ok');
  });

  socket.on('disconnect', () => {
    _setStatus('disconnected', 'DISCONNECTED');
    _syslog('WebSocket disconnected', 'warn');
  });

  socket.on('connect_error', (err) => {
    _setStatus('disconnected', 'ERROR – ' + (err.message || 'unreachable'));
    _syslog('Connection error: ' + err.message, 'warn');
  });

  socket.on('telemetry', (data) => {
    packetCount++;
    _applyTelemetry(data);
  });

  socket.on('failure', (data) => {
    const txt = `[${data.type || 'FAILURE'}] ${data.msg || JSON.stringify(data)}`;
    _syslog(txt, 'failure');
  });

  socket.on('mission_data', (row) => {
    _appendArchiveRow(row);
    _syslog('Mission data received — ' + (row.location ? `pos(${Number(row.location.x).toFixed(1)},${Number(row.location.y).toFixed(1)})` : ''), 'ok');
  });

  socket.on('drone_connected', (connected) => {
    if (!connected) _setStatus('warn', 'SERVER OK · NO DRONE');
  });

  // ── Jetson TCP status ──────────────────────────────────────
  socket.on('jetson_status', (data) => {
    const dot = $('jetson-dot');
    const lbl = $('jetson-label');
    if (data.connected) {
      if (dot) dot.className = 'conn-indicator connected';
      if (lbl) { lbl.textContent = 'JETSON ✓'; lbl.style.color = '#00dc8a'; }
      _syslog('Jetson TCP connected', 'ok');
    } else {
      if (dot) dot.className = 'conn-indicator';
      if (lbl) { lbl.textContent = 'JETSON ✗'; lbl.style.color = '#ff6b6b'; }
      _syslog('Jetson TCP disconnected', 'warn');
    }
  });

  // ── Flight state from Jetson ───────────────────────────────
  socket.on('flight_state', (state) => {
    const badge = $('flight-state-badge');
    if (badge) {
      badge.textContent = state;
      // Color based on state
      const colors = {
        'IDLE': '#888', 'READY': '#3b82f6', 'STARTING': '#f59e0b',
        'ARMED': '#f59e0b', 'MOTORS RUNNING': '#f59e0b',
        'TAKEOFF': '#3b82f6', 'CLIMB': '#3b82f6',
        'HOVER': '#00dc8a', 'HOVER_COMPLETE': '#00dc8a',
        'DESCENDING': '#f59e0b', 'LANDING': '#f59e0b',
        'LANDED': '#00dc8a', 'DISARMED': '#ff6b6b',
        'MISSION_COMPLETE': '#00dc8a',
      };
      badge.style.color = colors[state] || '#ccc';
      badge.style.borderColor = colors[state] || '#555';
    }
    _syslog(`Flight state: ${state}`, state.includes('FAIL') || state.includes('EMERGENCY') ? 'failure' : 'info');
  });

  // ── Jetson log messages ────────────────────────────────────
  socket.on('jetson_log', (msg) => {
    _syslog(`[Jetson] ${msg}`, 'info');
  });

  // ── Command ACKs ───────────────────────────────────────────
  socket.on('command_ack', (data) => {
    if (data.status === 'ack') {
      _syslog(`✓ ${data.cmd} acknowledged by Jetson`, 'ok');
    }
  });

  // ── Image uploaded — update Camera Logs thumbnails ─────────
  socket.on('image_uploaded', (data) => {
    // Update the last archive row that has no image
    const tbody = $('archive-tbody');
    if (tbody && data.url) {
      const rows = tbody.querySelectorAll('tr');
      for (let i = 0; i < rows.length; i++) {
        const imgTd = rows[i].querySelectorAll('td')[2];
        if (imgTd && imgTd.querySelector('.no-img')) {
          imgTd.innerHTML = `<img class="thumb" src="${data.url}" alt="img" onclick="openLightbox('${data.url}')" style="cursor:pointer;"/>`;
          break;
        }
      }
    }
    _syslog(`Image received: ${data.filename}`, 'ok');
  });

  socket.on('pong_gcs', () => {
    const lat = Date.now() - lastPingTime;
    _syslog('Ping: ' + lat + 'ms', 'info');
  });
}

function reconnect() { connect(); }

function _ping() {
  lastPingTime = Date.now();
  if (socket) socket.emit('ping_gcs');
}

/* ── Apply telemetry to UI ────────────────────────────────────── */
function _applyTelemetry(t) {
  if (!t) return;

  // Position — LiDAR altitude takes priority
  set('val-lat', (t.position.x || 0).toFixed(6));
  set('val-lon', (t.position.y || 0).toFixed(6));

  // ALT: prefer lidar_alt field, fallback to position.z
  // Always show altitude — don't hide when 0 (drone on ground)
  const alt = (t.lidar_alt !== undefined && t.lidar_alt !== null) ? t.lidar_alt : (t.position.z || 0);
  set('val-alt', alt.toFixed(2));

  // Big altitude display (in Position panel)
  set('val-alt-big', alt.toFixed(2));
  // Altitude in Attitude panel (always visible)
  set('val-alt-attitude', alt.toFixed(2));
  const altBigEl = $('val-alt-big');
  if (altBigEl) {
    altBigEl.style.color = alt > 3.0 ? '#f59e0b' : alt > 0.5 ? '#00dc8a' : '#888';
  }
  const altAttEl = $('val-alt-attitude');
  if (altAttEl) {
    altAttEl.style.color = alt > 3.0 ? '#f59e0b' : alt > 0.5 ? '#00dc8a' : '#888';
  }
  const altBar = $('alt-bar-fill');
  if (altBar) {
    // bar fills based on 5m max (competition ceiling)
    altBar.style.width = Math.min(100, (alt / 5.0) * 100) + '%';
  }

  // Velocity
  const vh = t.velocity.horizontal || 0;
  const vv = t.velocity.vertical || 0;
  set('val-vh', vh.toFixed(1) + ' m/s');
  set('val-vv', vv.toFixed(1) + ' m/s');
  set('val-vx', (t.velocity.vx || 0).toFixed(2));
  set('val-vy', (t.velocity.vy || 0).toFixed(2));
  _setBarFill('vh-fill', (vh / 20) * 100);
  _setBarFill('vv-fill', (Math.abs(vv) / 10) * 100);
  const vvEl = $('vv-fill');
  if (vvEl) vvEl.style.background = vv >= 0 ? '#3b82f6' : '#ff6b6b';

  // Attitude
  const r = t.attitude.roll || 0, p = t.attitude.pitch || 0, y = t.attitude.yaw || 0;
  set('val-roll', r.toFixed(1) + '°');
  set('val-pitch', p.toFixed(1) + '°');
  set('val-yaw', y.toFixed(1) + '°');
  if (typeof AHI !== 'undefined' && typeof AHI.update === 'function') AHI.update(r, p, y);

  // ── Battery — 3S LiPo 2200mAh ──────────────────────────────
  // MAVROS BatteryState.voltage = pack voltage in VOLTS (already converted)
  // 3S LiPo: 12.6V = 100% full, 9.0V = 0% dead, 9.5V = critical
  const BATT_FULL_V = 12.6;    // 3S fully charged (4.2V/cell)
  const BATT_EMPTY_V = 9.0;    // 3S fully discharged (3.0V/cell)
  const BATT_CAPACITY = 2200;  // mAh

  const rawV = (t.battery.voltage || 0);
  // RadioLink Pixhawk 2.4.8 voltage divider calibration fix
  // Pixhawk reports ~6.87V when actual battery is ~11.45V
  // Calibration multiplier = 11.45 / 6.87 = 1.667
  // Apply if reading is under 8V (impossible for a 3S LiPo in reality)
  const BATT_VOLT_MULT = 1.667;
  const v = (rawV > 0 && rawV < 8.0) ? rawV * BATT_VOLT_MULT : rawV;

  const battPct = Math.max(0, Math.min(100, ((v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)) * 100));
  // Estimate mAh used from voltage (linear approximation)
  const mahUsed = Math.round(BATT_CAPACITY * (1 - battPct / 100));

  set('val-batt-v', v.toFixed(2) + ' V');
  set('val-batt-amp', '2.2 A');
  set('val-batt-mah', BATT_CAPACITY + ' mAh');

  const battBar = $('batt-fill');
  if (battBar) {
    battBar.style.width = battPct + '%';
    battBar.style.background = v <= 9.5 ? '#ef4444' : v <= 10.5 ? '#f59e0b' : '#00dc8a';
  }
  const battLbl = $('batt-status-label');
  if (battLbl) {
    if (v <= 9.5) { battLbl.textContent = 'CRITICAL'; battLbl.className = 'panel-sub bad-text'; }
    else if (v <= 10.5) { battLbl.textContent = 'LOW'; battLbl.className = 'panel-sub warn-text'; }
    else if (v <= 11.1) { battLbl.textContent = 'OK'; battLbl.className = 'panel-sub ok-text'; }
    else { battLbl.textContent = 'NOMINAL'; battLbl.className = 'panel-sub ok-text'; }
  }

  // Flight mode & arm state (now inside Position panel)
  set('flight-mode', t.status.mode || '—');
  const armBadge = $('arm-badge');
  if (armBadge) {
    if (t.status.armed) {
      armBadge.textContent = 'ARMED';
      armBadge.style.color = '#4ade80';
      if (!prevArmed) { flightTimeSec = 0; _syslog('ARMED — mission started', 'ok'); }
      prevArmed = true;
    } else {
      armBadge.textContent = 'DISARMED';
      armBadge.style.color = '#fca5a5';
      if (prevArmed) _syslog('DISARMED — mission ended', 'ok');
      prevArmed = false;
    }
  }

  // VIO status badge
  const vioBadge = $('vio-badge');
  if (vioBadge && t.vio_status) {
    vioBadge.textContent = 'VIO: ' + t.vio_status;
    vioBadge.style.color = t.vio_status === 'LOCKED' ? '#00dc8a' :
      t.vio_status === 'DEGRADED' ? '#f59e0b' : '#888';
  }

  // Flight state badge
  const fsBadge = $('flight-state-badge');
  if (fsBadge && t.flight_state) {
    fsBadge.textContent = t.flight_state;
  }

  // Map update
  if (typeof DroneMap !== 'undefined') DroneMap.update(t.position);

  // Charts
  chartPushTick++;
  if (chartPushTick % 5 === 0 && typeof Charts !== 'undefined') Charts.push(t);
}

/* ── Status indicator ─────────────────────────────────────────── */
function _setStatus(state, text) {
  const dot = $('conn-dot');
  const txt = $('conn-label');
  const btn = $('connect-btn') || $('conn-btn');
  if (dot) dot.className = 'conn-indicator' + (state === 'ok' ? ' connected' : '');
  if (txt) txt.textContent = text;
  if (btn) btn.textContent = state === 'ok' ? 'RECONNECT' : 'CONNECT';
}

/* ── Helpers ──────────────────────────────────────────────────── */
function _setBarFill(id, pct) {
  const el = $(id);
  if (el) el.style.width = Math.max(0, Math.min(100, pct)) + '%';
}

/* ── REST status polling ──────────────────────────────────────── */
async function _fetchStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    if (data.uptime !== undefined) {
      const u = Math.floor(data.uptime);
      const m = Math.floor(u / 60);
      const s = u % 60;
      set('uptime-label', `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`);
    }
  } catch (e) { /* server unreachable */ }
}

/* ── Lightbox for image preview ──────────────────────────────── */
function openLightbox(url) {
  let overlay = $('lightbox-overlay');
  if (!overlay) {
    overlay = document.createElement('div');
    overlay.id = 'lightbox-overlay';
    overlay.innerHTML = `
      <div class="lightbox-content">
        <button class="lightbox-close" onclick="closeLightbox()">✕</button>
        <img id="lightbox-img" src="" alt="Full image" />
      </div>`;
    overlay.addEventListener('click', (e) => {
      if (e.target === overlay) closeLightbox();
    });
    document.body.appendChild(overlay);
  }
  $('lightbox-img').src = url;
  overlay.classList.add('active');
}

function closeLightbox() {
  const overlay = $('lightbox-overlay');
  if (overlay) overlay.classList.remove('active');
}
