/**
 * MAVLink Message Parser
 * Maps MAVLink message types to telemetry state
 * Reference: https://mavlink.io/en/messages/common.html
 */

function applyMessage(msg, telemetry) {
  if (!msg || !msg.type) return;

  switch (msg.type) {

    // ATTITUDE (#30) - Roll, pitch, yaw in radians
    case 'ATTITUDE':
      telemetry.attitude.roll  = toDeg(msg.roll  || 0);
      telemetry.attitude.pitch = toDeg(msg.pitch || 0);
      telemetry.attitude.yaw   = normYaw(toDeg(msg.yaw || 0));
      break;

    // GLOBAL_POSITION_INT (#33) - GPS position + velocity
    case 'GLOBAL_POSITION_INT':
      telemetry.position.x = (msg.lat || 0) / 1e7;
      telemetry.position.y = (msg.lon || 0) / 1e7;
      telemetry.position.z = (msg.relative_alt || 0) / 1000;   // mm → m
      telemetry.velocity.vx         = (msg.vx || 0) / 100;     // cm/s → m/s
      telemetry.velocity.vy         = (msg.vy || 0) / 100;
      telemetry.velocity.vz         = (msg.vz || 0) / 100;
      telemetry.velocity.horizontal = Math.hypot(telemetry.velocity.vx, telemetry.velocity.vy);
      telemetry.velocity.vertical   = telemetry.velocity.vz;
      break;

    // LOCAL_POSITION_NED (#32) - Local x/y/z position
    case 'LOCAL_POSITION_NED':
      if (!telemetry.position.x) {   // only use if no GPS
        telemetry.position.x = msg.x || 0;
        telemetry.position.y = msg.y || 0;
        telemetry.position.z = -(msg.z || 0);   // NED z is down
      }
      break;

    // SYS_STATUS (#1) - Battery + sensor health
    case 'SYS_STATUS':
      telemetry.battery.voltage = (msg.voltage_battery || 0) / 1000;   // mV → V
      telemetry.battery.current = (msg.current_battery || 0) / 100;    // cA → A
      telemetry.battery.percent = msg.battery_remaining >= 0 ? msg.battery_remaining : telemetry.battery.percent;
      break;

    // BATTERY_STATUS (#147) - Extended battery info
    case 'BATTERY_STATUS':
      if (msg.battery_remaining !== undefined)
        telemetry.battery.percent = msg.battery_remaining;
      if (msg.current_consumed !== undefined)
        telemetry.battery.mah = msg.current_consumed;
      break;

    // GPS_RAW_INT (#24) - GPS fix & satellite count
    case 'GPS_RAW_INT':
      telemetry.sensors.gps_fix    = msg.fix_type || 0;
      telemetry.sensors.satellites = msg.satellites_visible || 0;
      break;

    // SCALED_PRESSURE (#29) - Barometer
    case 'SCALED_PRESSURE':
      telemetry.sensors.pressure    = msg.press_abs || 0;
      telemetry.sensors.temperature = (msg.temperature || 0) / 100;   // cdegC → °C
      break;

    // VFR_HUD (#74) - Heading, airspeed, groundspeed, alt
    case 'VFR_HUD':
      telemetry.sensors.compass_heading = msg.heading || 0;
      if (!telemetry.velocity.horizontal)
        telemetry.velocity.horizontal = msg.groundspeed || 0;
      break;

    // HEARTBEAT (#0) - Armed state, flight mode
    case 'HEARTBEAT':
      telemetry.status.armed = !!(msg.base_mode & 128);
      telemetry.status.mode  = customModeToString(msg.custom_mode, msg.autopilot);
      break;

    // RC_CHANNELS_RAW (#35) - RSSI
    case 'RC_CHANNELS_RAW':
      telemetry.status.rssi = msg.rssi || 0;
      break;

    // RADIO_STATUS (#109) - Telemetry radio RSSI
    case 'RADIO_STATUS':
      telemetry.status.rssi = msg.rssi || 0;
      break;

    // TELEMETRY — unified packet from Jetson telemetry_bridge.py
    case 'TELEMETRY':
      if (msg.position) Object.assign(telemetry.position, msg.position);
      if (msg.velocity) Object.assign(telemetry.velocity, msg.velocity);
      if (msg.attitude) Object.assign(telemetry.attitude, msg.attitude);
      if (msg.battery)  Object.assign(telemetry.battery,  msg.battery);
      if (msg.sensors)  Object.assign(telemetry.sensors,  msg.sensors);
      if (msg.status)   Object.assign(telemetry.status,   msg.status);
      break;

    default:
      break;
  }
}

function toDeg(radians) {
  return parseFloat((radians * 180 / Math.PI).toFixed(2));
}

function normYaw(deg) {
  return ((deg % 360) + 360) % 360;
}

// ArduCopter custom modes mapping
const ARDUCOPTER_MODES = {
  0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
  4: 'GUIDED',    5: 'LOITER', 6: 'RTL',    7: 'CIRCLE',
  9: 'LAND',      11: 'DRIFT', 13: 'SPORT',  14: 'FLIP',
  15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE', 18: 'THROW',
  19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL'
};

function customModeToString(mode, autopilot) {
  return ARDUCOPTER_MODES[mode] || `MODE_${mode}`;
}

module.exports = { applyMessage };
