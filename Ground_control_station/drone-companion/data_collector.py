#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
data_collector.py  –  ROS1 Node 3: Camera Image Capture + Data Logger
=======================================================================
Runs ON the Jetson Nano (ROS1 Melodic / Python 2.7).

Captures one image every 20 seconds while the drone is in HOVER or TAKEOFF
state. Saves images to ~/mission_data/images/ and appends a record to
~/mission_data/records.json with:
    DATE, TIME, IMG, LOCATION (ENU x/y), VELOCITY (horizontal m/s)

Publishes each record as JSON on /drone/data_record.
Sends each record to the GCS over UDP for live display in Camera Logs panel.

Usage (inside roslaunch or standalone):
    rosrun drone_mission data_collector.py
"""

import rospy
import json
import os
import socket
import threading
import time
import math
from datetime import datetime
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

# ── Constants ──────────────────────────────────────────────────────────────────
CAPTURE_INTERVAL_SEC = 20        # seconds between image captures (was 2s)
ACTIVE_STATES        = ('HOVER', 'TAKEOFF')

# ── Globals ────────────────────────────────────────────────────────────────────
_flight_state    = 'INIT'
_local_x         = 0.0   # ENU metres
_local_y         = 0.0
_local_z         = 0.0
_horiz_vel       = 0.0   # horizontal speed m/s
_gps_lat         = 0.0
_gps_lon         = 0.0
_seq             = 0
_capturing       = False
_last_capture_t  = 0.0
_capture_thread  = None
_lock            = threading.Lock()

# ── ROS publisher ──────────────────────────────────────────────────────────────
_record_pub = None

# ── Paths ──────────────────────────────────────────────────────────────────────
DATA_DIR   = os.path.expanduser('~/mission_data')
IMG_DIR    = os.path.join(DATA_DIR, 'images')
RECORD_LOG = os.path.join(DATA_DIR, 'records.json')

# ── GCS UDP (for live Camera Logs in the browser) ──────────────────────────────
_udp_sock = None
_gcs_ip   = '192.168.70.100'
_gcs_port = 14550


def _ensure_dirs():
    if not os.path.isdir(IMG_DIR):
        os.makedirs(IMG_DIR)


def _send_to_gcs(record):
    """Forward a record as a MISSION_DATA UDP packet to the GCS."""
    if _udp_sock is None:
        return
    payload = json.dumps({'type': 'MISSION_DATA', 'data': record}) + '\n'
    try:
        _udp_sock.sendto(payload.encode('utf-8'), (_gcs_ip, _gcs_port))
    except Exception as e:
        rospy.logwarn('[data_collector] UDP send error: %s', str(e))


def _capture_image(img_path):
    """Capture one JPEG frame from /dev/video0 using OpenCV."""
    try:
        import cv2
    except ImportError:
        rospy.logerr('[data_collector] opencv-python not installed')
        return False

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr('[data_collector] Cannot open /dev/video0')
        cap.release()
        return False

    ret, frame = cap.read()
    cap.release()
    if not ret:
        rospy.logerr('[data_collector] Frame capture failed')
        return False

    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if ok:
        with open(img_path, 'wb') as f:
            f.write(buf.tobytes())
        return True
    return False


def _do_capture():
    """Capture an image + log metadata. Called from background thread."""
    global _seq

    with _lock:
        seq_num = _seq
        _seq   += 1
        x, y, z = _local_x, _local_y, _local_z
        vh      = _horiz_vel
        lat     = _gps_lat
        lon     = _gps_lon

    now       = datetime.utcnow()
    date_str  = now.strftime('%d/%m/%Y')
    time_str  = now.strftime('%H:%M:%S')
    fname     = now.strftime('%Y%m%d_%H%M%S') + '_{:03d}.jpg'.format(seq_num)
    img_path  = os.path.join(IMG_DIR, fname)
    rel_path  = 'images/' + fname

    success = _capture_image(img_path)
    if not success:
        img_path = None

    record = {
        'date'      : date_str,
        'time'      : time_str,
        'image_file': rel_path if success else None,
        'image_url' : None,
        'location'  : {'x': round(x, 3), 'y': round(y, 3)},
        'altitude'  : round(z, 2),
        'velocity'  : round(vh, 2),
        'gps_lat'   : round(lat, 7),
        'gps_lon'   : round(lon, 7),
    }

    # Append to records.json
    try:
        with open(RECORD_LOG, 'a') as f:
            f.write(json.dumps(record) + '\n')
    except Exception as e:
        rospy.logerr('[data_collector] JSON log error: %s', str(e))

    # Publish on ROS topic
    if _record_pub is not None:
        msg      = String()
        msg.data = json.dumps(record)
        _record_pub.publish(msg)

    # Send to GCS for live Camera Logs panel
    _send_to_gcs(record)

    rospy.loginfo('[data_collector] Captured %s  pos(%.2f,%.2f)  vel=%.2f m/s',
                  fname, x, y, vh)


# ── Timed capture loop ─────────────────────────────────────────────────────────

def _capture_loop():
    """Background thread: fires _do_capture() every CAPTURE_INTERVAL_SEC."""
    global _last_capture_t
    rospy.loginfo('[data_collector] Capture loop started — interval %ds',
                  CAPTURE_INTERVAL_SEC)
    while not rospy.is_shutdown() and _capturing:
        now = time.time()
        if now - _last_capture_t >= CAPTURE_INTERVAL_SEC:
            _last_capture_t = now
            _do_capture()
        time.sleep(1.0)
    rospy.loginfo('[data_collector] Capture loop stopped')


def _start_capture():
    global _capturing, _capture_thread, _last_capture_t
    if _capturing:
        return
    _capturing = True
    _last_capture_t = 0.0   # trigger immediately on first tick
    _capture_thread = threading.Thread(target=_capture_loop)
    _capture_thread.daemon = True
    _capture_thread.start()


def _stop_capture():
    global _capturing
    _capturing = False


# ── ROS Subscribers ────────────────────────────────────────────────────────────

def _cb_flight_state(msg):
    global _flight_state
    state = msg.data.strip().upper()
    prev  = _flight_state
    _flight_state = state

    if state in ACTIVE_STATES and prev not in ACTIVE_STATES:
        rospy.loginfo('[data_collector] State %s → start capturing', state)
        _start_capture()
    elif state not in ACTIVE_STATES and prev in ACTIVE_STATES:
        rospy.loginfo('[data_collector] State %s → stop capturing', state)
        _stop_capture()


def _cb_local_pos(msg):
    global _local_x, _local_y, _local_z
    with _lock:
        _local_x = msg.pose.position.x
        _local_y = msg.pose.position.y
        _local_z = msg.pose.position.z


def _cb_velocity(msg):
    global _horiz_vel
    vx = msg.twist.linear.x
    vy = msg.twist.linear.y
    with _lock:
        _horiz_vel = math.sqrt(vx * vx + vy * vy)


def _cb_gps(msg):
    global _gps_lat, _gps_lon
    with _lock:
        _gps_lat = msg.latitude
        _gps_lon = msg.longitude


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    global _record_pub, _udp_sock, _gcs_ip, _gcs_port

    rospy.init_node('data_collector', anonymous=False)

    _gcs_ip   = rospy.get_param('~gcs_ip',   '192.168.70.100')
    _gcs_port = int(rospy.get_param('~gcs_port', 14550))

    _ensure_dirs()

    _udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _record_pub = rospy.Publisher('/drone/data_record', String, queue_size=20)

    rospy.Subscriber('/drone/flight_state',
                     String, _cb_flight_state, queue_size=5)
    rospy.Subscriber('/mavros/local_position/pose',
                     PoseStamped, _cb_local_pos, queue_size=5)
    rospy.Subscriber('/mavros/local_position/velocity_local',
                     TwistStamped, _cb_velocity, queue_size=5)
    rospy.Subscriber('/mavros/global_position/global',
                     NavSatFix, _cb_gps, queue_size=5)

    rospy.loginfo('[data_collector] Ready — image every %ds when HOVER/TAKEOFF',
                  CAPTURE_INTERVAL_SEC)
    rospy.loginfo('[data_collector] GCS → %s:%d', _gcs_ip, _gcs_port)
    rospy.loginfo('[data_collector] Data dir: %s', DATA_DIR)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        _stop_capture()
        if _udp_sock:
            _udp_sock.close()
        rospy.loginfo('[data_collector] Shutdown')


if __name__ == '__main__':
    main()
