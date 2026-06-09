#!/usr/bin/env python3
"""
Drone Bridge — runs on companion Raspberry Pi / Jetson onboard the drone.
Reads MAVLink from flight controller via UART/USB,
parses messages and forwards JSON over UDP to the ground laptop GCS.

Install:  pip install pymavlink
Run:      python3 drone_bridge.py --fc /dev/ttyAMA0 --baud 57600 --gcs 192.168.1.100 --port 14550
"""

import argparse, json, socket, time, threading
from pymavlink import mavutil

parser = argparse.ArgumentParser(description='Drone MAVLink Bridge')
parser.add_argument('--fc',   default='/dev/ttyAMA0', help='Flight controller serial port')
parser.add_argument('--baud', default=57600, type=int)
parser.add_argument('--gcs',  default='192.168.1.100', help='GCS laptop IP')
parser.add_argument('--port', default=14550, type=int, help='GCS UDP port')
parser.add_argument('--hz',   default=20, type=int, help='Telemetry rate Hz')
args = parser.parse_args()

sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
GCS_ADDR = (args.gcs, args.port)

WANTED_TYPES = {
    'HEARTBEAT','SYS_STATUS','BATTERY_STATUS','ATTITUDE',
    'GLOBAL_POSITION_INT','LOCAL_POSITION_NED','GPS_RAW_INT',
    'VFR_HUD','SCALED_PRESSURE','RC_CHANNELS_RAW','RADIO_STATUS'
}

def send_to_gcs(d):
    try:
        sock.sendto((json.dumps(d)+'\n').encode(), GCS_ADDR)
    except Exception as e:
        print(f'[UDP ERROR] {e}')

def request_streams(mav):
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, args.hz, 1)
    print(f'[INFO] Streams requested at {args.hz} Hz')

def main():
    print(f'[INFO] Connecting to FC: {args.fc} @ {args.baud}')
    mav = mavutil.mavlink_connection(args.fc, baud=args.baud)
    print('[INFO] Waiting for heartbeat...')
    mav.wait_heartbeat()
    print(f'[INFO] Heartbeat OK sysid={mav.target_system}')
    time.sleep(0.5)
    request_streams(mav)
    def refresh():
        while True:
            time.sleep(30)
            request_streams(mav)
    threading.Thread(target=refresh, daemon=True).start()
    print(f'[INFO] Forwarding → {args.gcs}:{args.port}')
    count = 0
    while True:
        msg = mav.recv_match(blocking=True, timeout=2.0)
        if not msg: continue
        t = msg.get_type()
        if t == 'BAD_DATA' or t not in WANTED_TYPES: continue
        d = msg.to_dict()
        d['type'] = t
        d.pop('mavpackettype', None)
        send_to_gcs(d)
        count += 1
        if count % 200 == 0:
            print(f'[INFO] {count} msgs sent | last: {t}')

if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: print('\n[INFO] Stopped')
