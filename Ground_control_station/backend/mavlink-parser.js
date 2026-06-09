/**
 * mavlink-parser.js
 * Parses raw MAVLink JSON messages (forwarded by drone companion Pi)
 * into a clean, normalised telemetry object.
 *
 * The companion Pi runs drone-companion/bridge.py which reads real MAVLink
 * from the flight controller and sends pre-parsed JSON lines over UDP.
 */

'use strict';

/**
 * Default telemetry state – all fields initialised to safe values.
 */
function defaultTelemetry() {
  return {
    timestamp: Date.now(),
    // Position (lat/lon in decimal degrees, alt in metres AGL)
    position: { x: 0.0, y: 0.0, z: 0.0 },
    // Velocities in m/s
    velocity: { vertical: 0.0, horizontal: 0.0, vx: 0.0, vy: 0.0, vz: 0.0 },
    // Euler angles in degrees
    attitude: { roll: 0.0, pitch: 0.0, yaw: 0.0 },
    // Battery
    battery: { voltage: 0.0, current: 0.0, percent: 0, remaining_mah: 0 },
    // Sensor health flags
    sensors: {
      gps_fix: 0,       // 0=no fix, 2=2D, 3=3D
      satellites: 0,
      imu_ok: false,
      baro_ok: false,
      mag_ok: false,
      temp_celsius: 0.0,
      hdop: 99.99,
    },
    // Flight mode / armed state
    status: {
      armed: false,
      mode: 'UNKNOWN',
      ekf_ok: false,
      flight_time_s: 0,
    },
  };
}

/**
 * Apply a single parsed MAVLink message onto the telemetry object (mutates in-place).
 * @param {object} telemetry - current state
 * @param {object} msg       - parsed JSON from bridge.py (has a `type` field)
 */
function applyMessage(telemetry, msg) {
  if (!msg || !msg.type) return;

  telemetry.timestamp = Date.now();

  switch (msg.type) {

    // ── Position & velocity ─────────────────────────────────
    case 'GLOBAL_POSITION_INT':
      telemetry.position.x = msg.lat / 1e7;             // degrees
      telemetry.position.y = msg.lon / 1e7;             // degrees
      telemetry.position.z = msg.relative_alt / 1000;   // mm → m
      telemetry.velocity.vx = msg.vx / 100;             // cm/s → m/s
      telemetry.velocity.vy = msg.vy / 100;
      telemetry.velocity.vz = msg.vz / 100;             // positive = down
      telemetry.velocity.vertical   = -(msg.vz / 100);  // positive = climb
      telemetry.velocity.horizontal = Math.hypot(msg.vx, msg.vy) / 100;
      break;

    // ── Attitude ────────────────────────────────────────────
    case 'ATTITUDE':
      telemetry.attitude.roll  = _toDeg(msg.roll);
      telemetry.attitude.pitch = _toDeg(msg.pitch);
      telemetry.attitude.yaw   = (_toDeg(msg.yaw) + 360) % 360;
      break;

    // ── Battery ─────────────────────────────────────────────
    case 'SYS_STATUS':
      telemetry.battery.voltage       = msg.voltage_battery / 1000; // mV → V
      telemetry.battery.current       = msg.current_battery / 100;  // cA → A
      telemetry.battery.percent       = msg.battery_remaining;      // 0-100
      telemetry.sensors.imu_ok        = !!(msg.onboard_control_sensors_health & 0x1);
      telemetry.sensors.baro_ok       = !!(msg.onboard_control_sensors_health & 0x20);
      telemetry.sensors.mag_ok        = !!(msg.onboard_control_sensors_health & 0x2);
      break;

    // ── GPS ─────────────────────────────────────────────────
    case 'GPS_RAW_INT':
      telemetry.sensors.gps_fix    = msg.fix_type;
      telemetry.sensors.satellites = msg.satellites_visible;
      telemetry.sensors.hdop       = msg.eph / 100;
      break;

    // ── Scaled IMU (temperature) ─────────────────────────────
    case 'SCALED_IMU2':
    case 'SCALED_IMU':
      telemetry.sensors.temp_celsius = msg.temperature / 100;
      break;

    // ── Heartbeat (armed / mode) ─────────────────────────────
    case 'HEARTBEAT':
      telemetry.status.armed = !!(msg.base_mode & 128); // MAV_MODE_FLAG_SAFETY_ARMED
      telemetry.status.mode  = _decodeMode(msg.custom_mode, msg.autopilot);
      break;

    // ── EKF status ───────────────────────────────────────────
    case 'EKF_STATUS_REPORT':
      telemetry.status.ekf_ok = !!(msg.flags & 0x1F);
      break;

    // ── Flight timer ─────────────────────────────────────────
    case 'VFR_HUD':
      // VFR_HUD also gives airspeed / groundspeed / throttle
      if (!telemetry.status.armed) telemetry.status.flight_time_s = 0;
      break;

    default:
      break;
  }
}

// ── Helpers ────────────────────────────────────────────────────────────────

function _toDeg(radians) {
  return radians * (180 / Math.PI);
}

// Very simplified mode decoder (ArduCopter custom modes).
// Extend as needed for your firmware.
const ARDUCOPTER_MODES = {
  0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
  4: 'GUIDED',    5: 'LOITER', 6: 'RTL',    7: 'CIRCLE',
  9: 'LAND',     16: 'POSHOLD', 17: 'BRAKE', 20: 'THROW',
};

function _decodeMode(customMode) {
  return ARDUCOPTER_MODES[customMode] || `MODE_${customMode}`;
}

module.exports = { defaultTelemetry, applyMessage };
