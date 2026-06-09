/**
 * Telemetry Simulator
 * Generates realistic drone flight data for testing without hardware
 */

let t = 0;
let interval = null;

// Simulated flight parameters
let simState = {
  lat: 18.5204,    // Mumbai lat
  lon: 73.8567,    // Mumbai lon
  alt: 0,
  heading: 0,
  speed: 0,
  phase: 'takeoff',   // takeoff | cruise | hover | land
  phaseTimer: 0,
  batteryV: 12.6
};

function start(telemetry) {
  interval = setInterval(() => {
    tick(telemetry);
    t += 0.05;
  }, 50);   // 20 Hz sim update
}

function stop() {
  if (interval) clearInterval(interval);
}

function tick(tel) {
  simState.phaseTimer += 0.05;

  // Phase transitions for realistic flight
  if (simState.phase === 'takeoff' && simState.alt >= 30) {
    simState.phase = 'cruise';
    simState.phaseTimer = 0;
  } else if (simState.phase === 'cruise' && simState.phaseTimer > 40) {
    simState.phase = 'hover';
    simState.phaseTimer = 0;
  } else if (simState.phase === 'hover' && simState.phaseTimer > 15) {
    simState.phase = 'land';
    simState.phaseTimer = 0;
  } else if (simState.phase === 'land' && simState.alt <= 0.1) {
    simState.phase = 'takeoff';
    simState.phaseTimer = 0;
    simState.batteryV   = 12.6;
  }

  // Altitude dynamics
  const vv = phaseVertVel();
  simState.alt = Math.max(0, simState.alt + vv * 0.05);

  // Heading & horizontal movement
  simState.heading = (simState.heading + 0.5 * Math.sin(t * 0.1)) % 360;
  const hSpeed = phaseHorizSpeed();
  simState.speed = hSpeed;
  const hdRad = simState.heading * Math.PI / 180;
  simState.lat += (hSpeed * Math.cos(hdRad) * 0.00001);
  simState.lon += (hSpeed * Math.sin(hdRad) * 0.00001);

  // Attitude — realistic banking & pitching
  const roll  = Math.sin(t * 0.4) * 12 + Math.sin(t * 1.1) * 4;
  const pitch = Math.cos(t * 0.3) * 8  + (hSpeed > 0 ? -5 : 0);
  const yaw   = simState.heading;

  // Battery drain
  simState.batteryV = Math.max(9.0, simState.batteryV - 0.005) + Math.sin(t * 0.1) * 0.02;

  // Apply to shared telemetry object
  tel.position.x  = parseFloat(simState.lat.toFixed(6));
  tel.position.y  = parseFloat(simState.lon.toFixed(6));
  tel.position.z  = parseFloat(simState.alt.toFixed(2));

  tel.velocity.vertical   = parseFloat(vv.toFixed(2));
  tel.velocity.horizontal = parseFloat((hSpeed + Math.sin(t * 0.7) * 0.3).toFixed(2));
  tel.velocity.vx = parseFloat((hSpeed * Math.cos(hdRad)).toFixed(2));
  tel.velocity.vy = parseFloat((hSpeed * Math.sin(hdRad)).toFixed(2));
  tel.velocity.vz = parseFloat(vv.toFixed(2));

  tel.attitude.roll  = parseFloat(roll.toFixed(2));
  tel.attitude.pitch = parseFloat(pitch.toFixed(2));
  tel.attitude.yaw   = parseFloat(yaw.toFixed(2));

  tel.battery.voltage = parseFloat(simState.batteryV.toFixed(2));
  tel.battery.current = parseFloat((8 + hSpeed * 0.5 + Math.random() * 0.2).toFixed(1));
  tel.battery.mah     = Math.round((12.6 - simState.batteryV) * 200);
  tel.battery.remaining_mah = tel.battery.mah;

  tel.sensors.gps_fix    = 3;
  tel.sensors.satellites = 10 + Math.round(Math.sin(t * 0.05) * 2);
  tel.sensors.temperature = parseFloat((28 + Math.sin(t * 0.02) * 3).toFixed(1));
  tel.sensors.pressure    = parseFloat((1013.25 - simState.alt * 0.12).toFixed(1));
  tel.sensors.compass_heading = Math.round(yaw);

  tel.status.armed   = simState.phase !== 'land' || simState.alt > 0.5;
  tel.status.mode    = phaseMode();
  tel.status.rssi    = 180 + Math.round(Math.sin(t * 0.1) * 20);
  tel.status.connected = true;
  tel.timestamp      = Date.now();
}

function phaseVertVel() {
  switch (simState.phase) {
    case 'takeoff': return 2.0 + Math.sin(t * 0.5) * 0.3;
    case 'cruise':  return Math.sin(t * 0.3) * 0.8;
    case 'hover':   return Math.sin(t * 0.8) * 0.2;
    case 'land':    return -1.5 - Math.random() * 0.1;
    default:        return 0;
  }
}

function phaseHorizSpeed() {
  switch (simState.phase) {
    case 'takeoff': return 0.5;
    case 'cruise':  return 5 + Math.sin(t * 0.2) * 2;
    case 'hover':   return 0.3;
    case 'land':    return 0.5;
    default:        return 0;
  }
}

function phaseMode() {
  switch (simState.phase) {
    case 'takeoff': return 'ALT_HOLD';
    case 'cruise':  return 'LOITER';
    case 'hover':   return 'POSHOLD';
    case 'land':    return 'LAND';
    default:        return 'STABILIZE';
  }
}

module.exports = { start, stop };
