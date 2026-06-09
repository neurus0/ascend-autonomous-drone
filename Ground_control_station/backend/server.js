/**
 * Drone GCS - Main Backend Server (TCP Version)
 * ================================================
 * IRoC-U 2026 — GPS-Denied Autonomous Flight
 * 
 * CHANGES FROM ORIGINAL:
 *   - Replaced UDP telemetry listener with TCP CLIENT that connects to Jetson
 *   - Commands (START, KILL, DISARM) sent over TCP, not UDP
 *   - LiDAR altitude field added to telemetry state
 *   - DISARM command handler added
 *   - WebSocket/Socket.io broadcast to browser UNCHANGED
 *   - Excel logging UNCHANGED
 *   - Camera proxy UNCHANGED
 *   - Mission data archive UNCHANGED
 * 
 * Transport: TCP → Jetson:5005 (bidirectional)
 */

require('dotenv').config();
const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const cors = require('cors');
const path = require('path');
const net = require('net');
const fs = require('fs');
const XLSX = require('xlsx');
const multer = require('multer');
const logger = require('./logger');
const mavparser = require('./mavparser');
const simulator = require('./simulator');

// ─── Config ────────────────────────────────────────────────────────────────
const PORT = process.env.SERVER_PORT || 3000;
const CAMERA_URL = process.env.CAMERA_URL || 'http://10.94.30.39:5000';
const JETSON_IP = process.env.JETSON_IP || '10.94.30.39';
const JETSON_TCP_PORT = parseInt(process.env.JETSON_TCP_PORT || '5005');
const SIMULATE = process.argv.includes('--simulate') || process.env.SIMULATE === 'true';

// ─── Excel Flight Log ───────────────────────────────────────────────────────
const LOGS_DIR = path.join(__dirname, 'logs');
const UPLOADS_DIR = path.join(__dirname, 'uploads');
const EXCEL_PATH = path.join(LOGS_DIR, 'flight_log.xlsx');

if (!fs.existsSync(LOGS_DIR)) fs.mkdirSync(LOGS_DIR, { recursive: true });
if (!fs.existsSync(UPLOADS_DIR)) fs.mkdirSync(UPLOADS_DIR, { recursive: true });

// ─── Multer config (save uploaded images) ───────────────────────────────────
const storage = multer.diskStorage({
  destination: (req, file, cb) => cb(null, UPLOADS_DIR),
  filename: (req, file, cb) => {
    const ts = new Date().toISOString().replace(/[:.]/g, '-');
    const ext = path.extname(file.originalname) || '.jpg';
    cb(null, `capture_${ts}${ext}`);
  }
});
const upload = multer({ storage, limits: { fileSize: 20 * 1024 * 1024 } }); // 20MB max

function _loadOrCreateWorkbook() {
  if (fs.existsSync(EXCEL_PATH)) {
    try { return XLSX.readFile(EXCEL_PATH); } catch (_) { }
  }
  const wb = XLSX.utils.book_new();
  const ws = XLSX.utils.aoa_to_sheet([['Date', 'Time', 'Level', 'Failure', 'Message']]);
  XLSX.utils.book_append_sheet(wb, ws, 'FlightLog');
  XLSX.writeFile(wb, EXCEL_PATH);
  return wb;
}

let _wb = _loadOrCreateWorkbook();

function appendExcelLog(level, failure, message) {
  try {
    const now = new Date();
    const date = now.toLocaleDateString('en-GB');
    const time = now.toLocaleTimeString('en-GB', { hour12: false });
    _wb = _loadOrCreateWorkbook();
    const ws = _wb.Sheets['FlightLog'];
    XLSX.utils.sheet_add_aoa(ws, [[date, time, level, failure || '', message]], { origin: -1 });
    XLSX.writeFile(_wb, EXCEL_PATH);
  } catch (e) {
    logger.warn('Excel write error: ' + e.message);
  }
}

logger.info(`Excel flight log: ${EXCEL_PATH}`);

// ─── Mission Data Archive (in-memory + REST) ────────────────────────────────
const missionData = [];

// ─── Express + Socket.io setup ─────────────────────────────────────────────
const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: '*', methods: ['GET', 'POST'] }
});

app.use(cors());
app.use(express.json());
app.use(express.static(path.join(__dirname, '../frontend/public')));
app.use('/uploads', express.static(UPLOADS_DIR));

// ─── Shared telemetry state ────────────────────────────────────────────────
const telemetry = {
  position: { x: 0.0, y: 0.0, z: 0.0 },
  velocity: { vertical: 0.0, horizontal: 0.0, vx: 0.0, vy: 0.0, vz: 0.0 },
  attitude: { roll: 0.0, pitch: 0.0, yaw: 0.0 },
  battery: { voltage: 0.0, current: 0.0, remaining_mah: 0 },
  sensors: { gps_fix: 0, satellites: 0, temperature: 0.0, pressure: 0.0 },
  status: { armed: false, mode: 'STABILIZE', connected: false },
  lidar_alt: 0.0,
  vio_status: 'SEARCHING',
  flight_state: 'IDLE',
  timestamp: Date.now()
};

// ─── REST API ──────────────────────────────────────────────────────────────
app.get('/api/telemetry', (req, res) => res.json(telemetry));

app.get('/api/status', (req, res) => res.json({
  ok: true,
  simulate: SIMULATE,
  uptime: process.uptime(),
  jetson: `${JETSON_IP}:${JETSON_TCP_PORT}`,
  jetson_connected: jetsonConnected
}));

app.get('/api/mission-data', (req, res) => res.json(missionData));

app.get('/api/download-log', (req, res) => {
  if (!fs.existsSync(EXCEL_PATH)) {
    return res.status(404).json({ error: 'No log file yet' });
  }
  res.download(EXCEL_PATH, 'flight_log.xlsx');
});

app.post('/api/mission-data', (req, res) => {
  const row = req.body;
  if (!row) return res.status(400).json({ error: 'Body required' });
  const now = new Date();
  row.date = row.date || now.toLocaleDateString('en-GB');
  row.time = row.time || now.toLocaleTimeString('en-GB', { hour12: false });

  // Save base64 image to disk if present
  if (row.image_base64) {
    const imagesDir = path.join(__dirname, 'mission_images');
    if (!fs.existsSync(imagesDir)) fs.mkdirSync(imagesDir, { recursive: true });
    const fname = row.image_file ? path.basename(row.image_file) : ('capture_' + Date.now() + '.jpg');
    const fpath = path.join(imagesDir, fname);
    try {
      fs.writeFileSync(fpath, Buffer.from(row.image_base64, 'base64'));
      logger.info('Saved image: ' + fname);
      row.image_url = '/mission_images/' + fname;
    } catch (e) {
      logger.warn('Image save error: ' + e.message);
    }
  }

  missionData.push(row);
  io.emit('mission_data', row);
  res.json({ ok: true, count: missionData.length });
});

// Upload image endpoint — saves to backend/uploads/
app.post('/api/upload-image', upload.single('image'), (req, res) => {
  if (!req.file) {
    return res.status(400).json({ error: 'No image file received' });
  }
  const saved = req.file.filename;
  const imageUrl = `/uploads/${saved}`;
  logger.info(`Image saved: ${saved} (${(req.file.size / 1024).toFixed(1)} KB)`);

  // Associate with latest mission data record if possible
  if (missionData.length > 0) {
    const lastRecord = missionData[missionData.length - 1];
    if (!lastRecord.image_url) {
      lastRecord.image_url = imageUrl;
    }
  }

  // Notify browsers so Camera Logs can show the image
  io.emit('image_uploaded', { filename: saved, url: imageUrl, index: req.body.index });
  res.json({ ok: true, filename: saved, url: imageUrl, size: req.file.size });
});

// List all uploaded images
app.get('/api/uploads', (req, res) => {
  try {
    const files = fs.readdirSync(UPLOADS_DIR).filter(f => /\.(jpg|jpeg|png|webp)$/i.test(f));
    res.json(files.map(f => ({ filename: f, url: `/uploads/${f}` })));
  } catch (e) {
    res.json([]);
  }
});

// ─── Camera Proxy (UNCHANGED) ──────────────────────────────────────────────
app.get('/camera-proxy', (req, res) => {
  const targetUrl = req.query.url || CAMERA_URL;
  if (!targetUrl) return res.status(400).json({ error: 'No ?url= provided' });
  try {
    const http_mod = targetUrl.startsWith('https') ? require('https') : require('http');
    const upstream = http_mod.get(targetUrl, (camRes) => {
      res.setHeader('Content-Type', camRes.headers['content-type'] || 'multipart/x-mixed-replace; boundary=frame');
      res.setHeader('Cache-Control', 'no-cache');
      res.setHeader('Access-Control-Allow-Origin', '*');
      camRes.pipe(res);
      req.on('close', () => upstream.destroy());
    });
    upstream.on('error', (err) => {
      logger.warn('Camera proxy error: ' + err.message);
      if (!res.headersSent) res.status(503).json({ error: 'Camera unreachable', detail: err.message });
    });
    upstream.setTimeout(5000, () => {
      upstream.destroy();
      if (!res.headersSent) res.status(504).json({ error: 'Camera timeout' });
    });
  } catch (err) {
    res.status(500).json({ error: 'Proxy error', detail: err.message });
  }
});
app.get('/camera', (req, res) => res.redirect('/camera-proxy'));

// ═══════════════════════════════════════════════════════════════════════════
// TCP CLIENT — connects to Jetson tcp_bridge.py on port 5005
// Replaces the old UDP listener and UDP command sender
// ═══════════════════════════════════════════════════════════════════════════
let jetsonSocket = null;
let jetsonConnected = false;
let jetsonBuffer = '';
let reconnectTimer = null;

function connectToJetson() {
  if (SIMULATE) {
    logger.info('[SIM] Skipping TCP connection to Jetson');
    return;
  }

  if (jetsonSocket) {
    try { jetsonSocket.destroy(); } catch (_) { }
    jetsonSocket = null;
  }

  logger.info(`TCP: Connecting to Jetson at ${JETSON_IP}:${JETSON_TCP_PORT}...`);

  const sock = new net.Socket();
  sock.setTimeout(5000);

  sock.connect(JETSON_TCP_PORT, JETSON_IP, () => {
    jetsonConnected = true;
    jetsonBuffer = '';
    jetsonSocket = sock;
    logger.info(`TCP: Connected to Jetson ${JETSON_IP}:${JETSON_TCP_PORT}`);
    io.emit('jetson_status', { connected: true });

    // Clear reconnect timer
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  });

  sock.on('data', (data) => {
    jetsonBuffer += data.toString('utf-8');

    while (jetsonBuffer.includes('\n')) {
      const idx = jetsonBuffer.indexOf('\n');
      const line = jetsonBuffer.substring(0, idx).trim();
      jetsonBuffer = jetsonBuffer.substring(idx + 1);

      if (!line) continue;
      _handleJetsonMessage(line);
    }
  });

  sock.on('error', (err) => {
    logger.warn(`TCP: Jetson connection error: ${err.message}`);
  });

  sock.on('close', () => {
    jetsonConnected = false;
    jetsonSocket = null;
    io.emit('jetson_status', { connected: false });
    logger.info('TCP: Disconnected from Jetson. Reconnecting in 3s...');

    reconnectTimer = setTimeout(connectToJetson, 3000);
  });

  sock.on('timeout', () => {
    logger.warn('TCP: Connection timeout to Jetson.');
    sock.destroy();
  });
}

function _handleJetsonMessage(line) {
  try {
    // Try JSON first (telemetry packets)
    const parsed = JSON.parse(line);

    if (parsed.type === 'TELEMETRY') {
      // Apply telemetry to our state
      if (parsed.position) Object.assign(telemetry.position, parsed.position);
      if (parsed.velocity) Object.assign(telemetry.velocity, parsed.velocity);
      if (parsed.attitude) Object.assign(telemetry.attitude, parsed.attitude);
      if (parsed.battery) Object.assign(telemetry.battery, parsed.battery);
      if (parsed.sensors) Object.assign(telemetry.sensors, parsed.sensors);
      if (parsed.status) Object.assign(telemetry.status, parsed.status);
      if (parsed.lidar_alt !== undefined) telemetry.lidar_alt = parsed.lidar_alt;
      if (parsed.vio_status) telemetry.vio_status = parsed.vio_status;
      if (parsed.flight_state) telemetry.flight_state = parsed.flight_state;
      return;
    }

    // Mission data or failure events
    if (parsed.type === 'FAILURE') {
      logger.error(`FAILURE: ${parsed.failure} – ${parsed.msg}`);
      appendExcelLog('FAILURE', parsed.failure || 'UNKNOWN', parsed.msg || '');
      io.emit('failure', { type: parsed.failure, msg: parsed.msg });
      return;
    }
    if (parsed.type === 'MISSION_DATA') {
      const row = { date: parsed.date, time: parsed.time, image_url: parsed.image_url, location: parsed.location, velocity: parsed.velocity };
      missionData.push(row);
      io.emit('mission_data', row);
      return;
    }

    // Other JSON from Jetson — apply via mavparser
    mavparser.applyMessage(parsed, telemetry);

  } catch (e) {
    // Not JSON — it's a text protocol message (ACK_START, STATE:xxx, LOG:xxx)
    _handleJetsonTextMessage(line);
  }
}

function _handleJetsonTextMessage(msg) {
  // Forward relevant messages to browser clients
  if (msg.startsWith('STATE:')) {
    const state = msg.substring(6);
    telemetry.flight_state = state;
    io.emit('flight_state', state);
    logger.info(`Jetson state: ${state}`);

  } else if (msg.startsWith('LOG:')) {
    const logMsg = msg.substring(4);
    io.emit('jetson_log', logMsg);
    logger.info(`Jetson: ${logMsg}`);

  } else if (msg === 'ACK_START') {
    io.emit('command_ack', { cmd: 'START_MISSION', status: 'ack', ts: Date.now() });
    logger.info('Jetson: ACK_START');

  } else if (msg === 'ACK_KILL') {
    io.emit('command_ack', { cmd: 'KILL', status: 'ack', ts: Date.now() });
    logger.info('Jetson: ACK_KILL');
    appendExcelLog('FAILURE', 'KILL_SWITCH', 'Kill acknowledged by Jetson');

  } else if (msg === 'ACK_DISARM') {
    io.emit('command_ack', { cmd: 'DISARM', status: 'ack', ts: Date.now() });
    logger.info('Jetson: ACK_DISARM');
    appendExcelLog('CMD', 'DISARM', 'Disarm acknowledged by Jetson');

  } else if (msg === 'PONG') {
    // Keepalive response — ignore
  } else if (msg === 'CONN_FAIL') {
    io.emit('jetson_log', 'Command rejected by Jetson');
  }
}

function sendToJetson(cmd) {
  if (SIMULATE) {
    logger.info(`[SIM] Would send to Jetson: ${cmd}`);
    // Simulate ACK for testing
    setTimeout(() => {
      if (cmd === 'START') io.emit('command_ack', { cmd: 'START_MISSION', status: 'ack', ts: Date.now() });
      if (cmd === 'KILL') io.emit('command_ack', { cmd: 'KILL', status: 'ack', ts: Date.now() });
      if (cmd === 'DISARM') io.emit('command_ack', { cmd: 'DISARM', status: 'ack', ts: Date.now() });
    }, 500);
    return;
  }

  if (!jetsonSocket || !jetsonConnected) {
    logger.warn(`Cannot send "${cmd}" — not connected to Jetson`);
    return;
  }

  try {
    jetsonSocket.write(cmd + '\n');
    logger.info(`→ Jetson: ${cmd}`);
  } catch (e) {
    logger.error(`Send to Jetson failed: ${e.message}`);
  }
}

// Keepalive PING every 5s
setInterval(() => {
  if (jetsonConnected) sendToJetson('PING');
}, 5000);

// ─── WebSocket events ──────────────────────────────────────────────────────
io.on('connection', (socket) => {
  logger.info(`GCS client connected: ${socket.id}`);
  telemetry.status.connected = true;
  socket.emit('telemetry', telemetry);
  socket.emit('jetson_status', { connected: jetsonConnected });

  socket.on('command', (data) => {
    const { cmd, params } = data;
    logger.info(`COMMAND: ${cmd}`, params);
    appendExcelLog('CMD', null, `${cmd} ${JSON.stringify(params)}`);

    if (cmd === 'START_MISSION') {
      sendToJetson('START');
      io.emit('command_ack', { cmd, status: 'sent', ts: Date.now() });

    } else if (cmd === 'KILL') {
      sendToJetson('KILL');
      io.emit('command_ack', { cmd, status: 'sent', ts: Date.now() });
      appendExcelLog('FAILURE', 'KILL_SWITCH', 'Operator-triggered kill switch');

    } else if (cmd === 'DISARM') {
      sendToJetson('DISARM');
      io.emit('command_ack', { cmd, status: 'sent', ts: Date.now() });

    } else {
      // Generic commands: RTL, LAND, LOITER
      sendToJetson(cmd);
      io.emit('command_ack', { cmd, status: 'sent', ts: Date.now() });
    }
  });

  socket.on('report_failure', (data) => {
    logger.error(`FAILURE: ${data.type} — ${data.msg}`);
    appendExcelLog('FAILURE', data.type || 'UNKNOWN', data.msg || '');
    io.emit('failure', data);
  });

  socket.on('mission_data_push', (row) => {
    const now = new Date();
    row.date = row.date || now.toLocaleDateString('en-GB');
    row.time = row.time || now.toLocaleTimeString('en-GB', { hour12: false });
    missionData.push(row);
    appendExcelLog('DATA', null, `Image captured at (${(row.location || {}).x},${(row.location || {}).y})`);
    io.emit('mission_data', row);
  });

  socket.on('ping_gcs', () => socket.emit('pong_gcs'));

  socket.on('disconnect', () => {
    logger.info(`GCS client disconnected: ${socket.id}`);
    if (io.engine.clientsCount === 0) telemetry.status.connected = false;
  });
});

// ─── Broadcast loop (10 Hz) ────────────────────────────────────────────────
setInterval(() => {
  telemetry.timestamp = Date.now();
  io.emit('telemetry', telemetry);
}, 100);

// ─── Data sources ──────────────────────────────────────────────────────────
if (SIMULATE) {
  logger.info('Running in SIMULATION mode');
  simulator.start(telemetry);
} else {
  // TCP connection to Jetson (replaces UDP)
  connectToJetson();
}

// ─── Start server ──────────────────────────────────────────────────────────
server.listen(PORT, () => {
  logger.info(`GCS server running at http://localhost:${PORT}`);
  logger.info(`Simulation: ${SIMULATE}`);
  logger.info(`Camera URL: ${CAMERA_URL}`);
  logger.info(`Jetson: ${JETSON_IP}:${JETSON_TCP_PORT} (TCP)`);
  logger.info(`Excel log: ${EXCEL_PATH}`);
});
