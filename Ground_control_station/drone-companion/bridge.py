#!/usr/bin/env python3
"""
bridge.py  –  Drone Companion Computer Bridge
═══════════════════════════════════════════════
Runs ON the drone's Raspberry Pi / Jetson.

  1. Connects to the flight controller via serial (MAVLink)
  2. Requests all telemetry streams
  3. Parses each MAVLink message with pymavlink
  4. Converts it to a compact JSON dict
  5. Sends it to the ground laptop over UDP (WiFi dongle)

Usage:
  python3 bridge.py
  python3 bridge.py --port /dev/ttyAMA0 --baud 115200 --gcs-ip 192.168.1.100

Dependencies (install on Pi):
  pip3 install pymavlink
"""

import argparse
import json
import socket
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run:  pip3 install pymavlink")
    sys.exit(1)


# ── Argument parsing ──────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description='MAVLink → UDP/JSON bridge')
parser.add_argument('--port',    default='/dev/ttyAMA0',    help='Serial port to FC')
parser.add_argument('--baud',    default=57600, type=int,   help='Serial baud rate')
parser.add_argument('--gcs-ip',  default='192.168.1.100',   help='Ground laptop IP')
parser.add_argument('--gcs-port',default=14550, type=int,   help='Ground UDP port')
parser.add_argument('--hz',      default=20, type=int,      help='Stream rate in Hz')
args = parser.parse_args()

# ── UDP socket (sends to ground) ──────────────────────────────────────────────

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
GCS_ADDR = (args.gcs_ip, args.gcs_port)

def send(msg_dict):
    """Serialise msg_dict to JSON and fire over UDP."""
    try:
        data = json.dumps(msg_dict) + '\n'
        udp.sendto(data.encode('utf-8'), GCS_ADDR)
    except Exception as e:
        print(f'[UDP] Send error: {e}')


# ── MAVLink connection ────────────────────────────────────────────────────────

print(f'[MAV] Connecting to FC on {args.port} @ {args.baud} baud …')
try:
    mav = mavutil.mavlink_connection(args.port, baud=args.baud)
except Exception as e:
    print(f'[MAV] Cannot open port: {e}')
    sys.exit(1)

print('[MAV] Waiting for heartbeat …')
mav.wait_heartbeat(timeout=15)
print(f'[MAV] Heartbeat from system {mav.target_system} / component {mav.target_component}')

# Request all data streams at the desired Hz
stream_ids = [
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
]
for sid in stream_ids:
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        sid, args.hz, 1   # 1 = start
    )

print(f'[MAV] Streaming → GCS {args.gcs_ip}:{args.gcs_port} @ {args.hz} Hz')
print('[MAV] Press Ctrl+C to stop.\n')


# ── Message types we care about ───────────────────────────────────────────────

WANTED = {
    'ATTITUDE',
    'GLOBAL_POSITION_INT',
    'GPS_RAW_INT',
    'SYS_STATUS',
    'HEARTBEAT',
    'SCALED_IMU',
    'SCALED_IMU2',
    'EKF_STATUS_REPORT',
    'VFR_HUD',
    'BATTERY_STATUS',
    'LOCAL_POSITION_NED',
}

# ── Main loop ─────────────────────────────────────────────────────────────────

packet_count = 0
start_time = time.time()

try:
    while True:
        msg = mav.recv_match(type=list(WANTED), blocking=True, timeout=1.0)

        if msg is None:
            # Send a keepalive so GCS knows we're still alive
            send({'type': 'KEEPALIVE', 'ts': int(time.time() * 1000)})
            continue

        msg_type = msg.get_type()
        if msg_type == 'BAD_DATA':
            continue

        # Convert to dict and add type tag
        d = msg.to_dict()
        d['type'] = msg_type

        # Remove verbose / useless fields
        for key in ('mavpackettype', 'time_boot_ms'):
            d.pop(key, None)

        send(d)
        packet_count += 1

        # Print stats every 5 s
        elapsed = time.time() - start_time
        if packet_count % (args.hz * 5) == 0:
            print(f'[MAV] {packet_count} packets in {elapsed:.0f}s  '
                  f'({packet_count/elapsed:.1f} pkt/s)')

except KeyboardInterrupt:
    print('\n[MAV] Bridge stopped.')
finally:
    udp.close()
