#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
photo_capture.py  —  Standalone Photo Capture + Data Logger
=============================================================
IRoC-U 2026 — GPS-Denied Autonomous Flight

Standalone script (NOT integrated into main flight stack).
Captures photos from Jetson's camera and logs metadata.

For each capture:
  - Saves JPEG to ~/mission_data/images/
  - Logs metadata to ~/mission_data/records.json:
    * timestamp
    * altitude (from /lidar/altitude)
    * roll, pitch, yaw
    * Vx, Vy, Vz
    * battery voltage
    * flight state

Usage:
  python2 photo_capture.py
  OR
  rosrun thl_drone photo_capture.py

Captures a photo every CAPTURE_INTERVAL_S seconds while armed.

Author: IRoC-U ASCEND 2026 Team
"""

import rospy
import cv2
import os
import json
import time
import threading

from sensor_msgs.msg   import Imu, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg   import State
from std_msgs.msg      import Float32, String
import math

# ─── CONFIG ───────────────────────────────────────────────────────────────────
CAPTURE_INTERVAL_S = 10.0     # seconds between captures
CAMERA_DEVICE      = 0        # /dev/video0
FRAME_W            = 640
FRAME_H            = 480
MISSION_DIR        = os.path.expanduser('~/mission_data')
IMAGES_DIR         = os.path.join(MISSION_DIR, 'images')
RECORDS_FILE       = os.path.join(MISSION_DIR, 'records.json')
# ──────────────────────────────────────────────────────────────────────────────


class PhotoCapture(object):

    def __init__(self):
        rospy.init_node('photo_capture_node', anonymous=False)

        # Ensure directories exist
        for d in [MISSION_DIR, IMAGES_DIR]:
            if not os.path.exists(d):
                os.makedirs(d)

        # State
        self._lock = threading.Lock()
        self._lidar_alt = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._voltage = 0.0
        self._armed = False
        self._flight_state = 'IDLE'
        self._capture_count = 0

        # Subscribers
        rospy.Subscriber('/lidar/altitude', Float32, self._lidar_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)
        rospy.Subscriber('/mavros/local_position/velocity_local',
                         TwistStamped, self._vel_cb)
        rospy.Subscriber('/mavros/battery', BatteryState, self._battery_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/drone/flight_state', String, self._flight_state_cb)

        # Open camera
        self._cap = self._open_camera()
        if self._cap is None:
            rospy.logerr('[PHOTO] Cannot open camera. Exiting.')
            rospy.signal_shutdown('No camera')
            return

        rospy.loginfo('[PHOTO] Photo capture ready. Interval: %.0fs', CAPTURE_INTERVAL_S)
        rospy.Timer(rospy.Duration(CAPTURE_INTERVAL_S), self._capture_timer)

    def _open_camera(self):
        try:
            # GStreamer MJPG
            gst = (
                'v4l2src device=/dev/video{dev} ! '
                'image/jpeg, width={w}, height={h}, framerate=30/1 ! '
                'jpegdec ! videoconvert ! '
                'video/x-raw, format=BGR ! '
                'appsink max-buffers=1 drop=true'
            ).format(dev=CAMERA_DEVICE, w=FRAME_W, h=FRAME_H)

            cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                time.sleep(1.0)
                ret, frame = cap.read()
                if ret and frame is not None:
                    rospy.loginfo('[PHOTO] Camera ready via GStreamer')
                    return cap
                cap.release()
        except Exception:
            pass

        # Fallback
        try:
            cap = cv2.VideoCapture(CAMERA_DEVICE)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                time.sleep(1.0)
                ret, frame = cap.read()
                if ret and frame is not None:
                    rospy.loginfo('[PHOTO] Camera ready via OpenCV')
                    return cap
                cap.release()
        except Exception:
            pass

        return None

    # ── Callbacks ─────────────────────────────────────────────────────

    def _lidar_cb(self, msg):
        with self._lock:
            self._lidar_alt = msg.data

    def _imu_cb(self, msg):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr) * 180.0 / math.pi
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1, min(1, sinp))) * 180.0 / math.pi
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy) * 180.0 / math.pi
        if yaw < 0:
            yaw += 360.0
        with self._lock:
            self._roll = round(roll, 2)
            self._pitch = round(pitch, 2)
            self._yaw = round(yaw, 2)

    def _vel_cb(self, msg):
        with self._lock:
            self._vx = round(msg.twist.linear.x, 3)
            self._vy = round(msg.twist.linear.y, 3)
            self._vz = round(msg.twist.linear.z, 3)

    def _battery_cb(self, msg):
        with self._lock:
            self._voltage = round(msg.voltage, 2)

    def _state_cb(self, msg):
        with self._lock:
            self._armed = msg.armed

    def _flight_state_cb(self, msg):
        with self._lock:
            self._flight_state = msg.data

    # ── Capture timer ─────────────────────────────────────────────────

    def _capture_timer(self, event):
        with self._lock:
            armed = self._armed

        if not armed:
            rospy.loginfo_throttle(10, '[PHOTO] Not armed. Skipping capture.')
            return

        if self._cap is None or not self._cap.isOpened():
            rospy.logwarn('[PHOTO] Camera not available.')
            return

        try:
            ret, frame = self._cap.read()
            if not ret or frame is None:
                rospy.logwarn('[PHOTO] Empty frame.')
                return

            self._capture_count += 1
            ts = time.strftime('%Y%m%d_%H%M%S')
            filename = 'capture_{:04d}_{}.jpg'.format(self._capture_count, ts)
            filepath = os.path.join(IMAGES_DIR, filename)

            cv2.imwrite(filepath, frame)

            # Build metadata record
            with self._lock:
                record = {
                    'capture_id': self._capture_count,
                    'timestamp': ts,
                    'date': time.strftime('%d/%m/%Y'),
                    'time': time.strftime('%H:%M:%S'),
                    'image_file': 'images/' + filename,
                    'altitude_m': self._lidar_alt,
                    'roll_deg': self._roll,
                    'pitch_deg': self._pitch,
                    'yaw_deg': self._yaw,
                    'vx_ms': self._vx,
                    'vy_ms': self._vy,
                    'vz_ms': self._vz,
                    'battery_v': self._voltage,
                    'flight_state': self._flight_state,
                    'location': {
                        'x': 0.0,
                        'y': 0.0,
                    },
                    'velocity': round(math.sqrt(self._vx**2 + self._vy**2 + self._vz**2), 3),
                }

            # Append to records.json
            self._append_record(record)

            rospy.loginfo('[PHOTO] Captured #%d: %s (alt=%.2fm)',
                          self._capture_count, filename, record['altitude_m'])

        except Exception as e:
            rospy.logwarn('[PHOTO] Capture error: %s', str(e))

    def _append_record(self, record):
        try:
            records = []
            if os.path.isfile(RECORDS_FILE):
                with open(RECORDS_FILE, 'r') as f:
                    content = f.read().strip()
                    if content.startswith('['):
                        records = json.loads(content)
            records.append(record)
            with open(RECORDS_FILE, 'w') as f:
                json.dump(records, f, indent=2)
        except Exception as e:
            rospy.logwarn('[PHOTO] Record write error: %s', str(e))

    def shutdown(self):
        if self._cap:
            self._cap.release()
        rospy.loginfo('[PHOTO] Shutdown complete.')


if __name__ == '__main__':
    node = PhotoCapture()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
