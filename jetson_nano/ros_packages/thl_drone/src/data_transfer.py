#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
data_transfer.py  -  Post-Landing Mission Data Transfer
=========================================================
IRoC-U 2026 — GPS-Denied Autonomous Flight

Runs ON the Jetson Nano alongside the flight stack.

What it does:
  1. Monitors /mavros/extended_state and /lidar/altitude
     to detect when the drone has landed.
  2. Listens to /drone/flight_state for LANDED/MISSION_COMPLETE signals.
  3. Upload starts when landing is confirmed + delay.
  4. After conditions met, scans ~/mission_data/ for:
       - images/  (JPEG frames captured by photo_capture.py)
       - records.json  (metadata: timestamp, position, velocity per image)
  5. Uploads everything to the GCS laptop over WiFi via HTTP:
       POST /api/mission-data   -> each metadata record
       POST /api/upload-image   -> each image (multipart)
  6. After uploading, logs the result to ~/mission_data/transfer_log.txt

CHANGED FROM ORIGINAL:
  - Removed UDP sending (tcp_bridge handles telemetry now)
  - Uses LiDAR altitude instead of local_position for landing detect
  - GCS IP updated to 192.168.70.198
  - Python 2.7 compatible (no f-strings)

Usage:
  Launched by unified_launch.launch (delays until after landing)
  Or manually: python2 data_transfer.py

Author: IRoC-U ASCEND 2026 Team
"""

import rospy
import os
import json
import time
import threading
import datetime

try:
    import requests
except ImportError:
    requests = None

from mavros_msgs.msg import ExtendedState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Float32

# ── File paths ────────────────────────────────────────────────────────────────
MISSION_DIR   = os.path.expanduser('~/mission_data')
RECORDS_FILE  = os.path.join(MISSION_DIR, 'records.json')
IMAGES_DIR    = os.path.join(MISSION_DIR, 'images')
TRANSFER_LOG  = os.path.join(MISSION_DIR, 'transfer_log.txt')

# Battery power_supply_status values
POWER_SUPPLY_STATUS_CHARGING = 1


class DataTransferNode(object):

    def __init__(self):
        rospy.init_node('data_transfer_node', anonymous=True)

        # ROS params (override with _gcs_ip:=x.x.x.x at launch)
        self.gcs_ip        = rospy.get_param('~gcs_ip',        '192.168.70.198')
        self.gcs_http_port = rospy.get_param('~gcs_http_port', 3000)
        self.upload_delay  = rospy.get_param('~upload_delay',  5.0)
        self.retry_count   = rospy.get_param('~retry_count',   3)

        # State
        self._landed          = False
        self._upload_started  = False
        self._conditions_met_time = None
        self._lidar_alt       = 0.0

        # ROS subscribers
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self._extended_state_cb)
        rospy.Subscriber('/lidar/altitude', Float32, self._lidar_cb)
        rospy.Subscriber('/mavros/battery', BatteryState, self._battery_cb)
        rospy.Subscriber('/drone/flight_state', String, self._flight_state_cb)

        # Publisher to signal upload progress
        self._status_pub = rospy.Publisher('/drone/upload_status', String, queue_size=5)

        if requests is None:
            rospy.logwarn('[DataTransfer] requests not installed. Run: pip2 install requests')

        rospy.loginfo('=' * 60)
        rospy.loginfo('DataTransferNode ready')
        rospy.loginfo('GCS: %s:%d (HTTP)', self.gcs_ip, self.gcs_http_port)
        rospy.loginfo('Mission dir: %s', MISSION_DIR)
        rospy.loginfo('Waiting for landing...')
        rospy.loginfo('=' * 60)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _extended_state_cb(self, msg):
        if msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            self._mark_landed()

    def _lidar_cb(self, msg):
        self._lidar_alt = msg.data
        # Landing detection: LiDAR altitude < 0.15m
        if msg.data < 0.15 and msg.data > 0:
            self._mark_landed()

    def _battery_cb(self, msg):
        pass  # Battery monitoring kept for future use

    def _flight_state_cb(self, msg):
        state = msg.data.strip().upper()
        if state in ('LANDED', 'MISSION_COMPLETE', 'DISARMED'):
            self._mark_landed()

    def _mark_landed(self):
        if not self._landed:
            self._landed = True
            self._conditions_met_time = time.time()
            rospy.loginfo('[DataTransfer] Landing detected! Upload in %.0fs...', self.upload_delay)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if (self._conditions_met_time is not None
                    and not self._upload_started
                    and (time.time() - self._conditions_met_time) >= self.upload_delay
                    and self._landed):
                self._upload_started = True
                self._publish_status('UPLOADING')
                t = threading.Thread(target=self._do_transfer)
                t.daemon = True
                t.start()
            rate.sleep()

    # ── Transfer logic ────────────────────────────────────────────────────────

    def _do_transfer(self):
        rospy.loginfo('[DataTransfer] Starting post-landing data transfer...')

        records = self._load_records()
        if not records:
            rospy.logwarn('[DataTransfer] No records to transfer.')
            self._log_transfer('NO_RECORDS', 0, 0)
            self._publish_status('UPLOAD_COMPLETE')
            return

        rospy.loginfo('[DataTransfer] Found %d records to upload', len(records))

        ok_count   = 0
        fail_count = 0

        for i, record in enumerate(records):
            # POST metadata to GCS HTTP API
            if requests is not None:
                success = self._http_post(record)
            else:
                success = False
                rospy.logwarn('[DataTransfer] requests not available, skipping record %d', i + 1)

            # Upload image file if it exists
            img_file = record.get('image_file', '')
            if img_file:
                full_img_path = os.path.join(MISSION_DIR, img_file)
                if os.path.isfile(full_img_path) and requests is not None:
                    self._http_upload_image(full_img_path, record, i)

            if success:
                ok_count += 1
                rospy.loginfo('[DataTransfer] [%d/%d] Record uploaded OK', i + 1, len(records))
            else:
                fail_count += 1
                rospy.logwarn('[DataTransfer] [%d/%d] Record upload FAILED', i + 1, len(records))

            time.sleep(0.1)

        rospy.loginfo('[DataTransfer] Transfer complete. OK=%d FAIL=%d', ok_count, fail_count)
        self._log_transfer('COMPLETE', ok_count, fail_count)
        self._publish_status('UPLOAD_COMPLETE')

    # ── Record loading ─────────────────────────────────────────────────────────

    def _load_records(self):
        if not os.path.isfile(RECORDS_FILE):
            rospy.logwarn('[DataTransfer] records.json not found at %s', RECORDS_FILE)
            return []
        try:
            with open(RECORDS_FILE, 'r') as f:
                content = f.read().strip()

            if content.startswith('['):
                return json.loads(content)

            # JSON-lines format
            records = []
            for line in content.split('\n'):
                line = line.strip()
                if line:
                    try:
                        records.append(json.loads(line))
                    except Exception:
                        pass
            return records
        except Exception as e:
            rospy.logerr('[DataTransfer] Failed to read records.json: %s', str(e))
            return []

    # ── HTTP helpers ───────────────────────────────────────────────────────────

    def _http_post(self, record):
        url = 'http://{}:{}/api/mission-data'.format(self.gcs_ip, self.gcs_http_port)
        for attempt in range(1, self.retry_count + 1):
            try:
                resp = requests.post(url, json=record, timeout=5)
                if resp.status_code == 200:
                    return True
                rospy.logwarn('[DataTransfer] HTTP %d on attempt %d', resp.status_code, attempt)
            except Exception as e:
                rospy.logwarn('[DataTransfer] HTTP error attempt %d: %s', attempt, str(e))
            time.sleep(1.0)
        return False

    def _http_upload_image(self, filepath, record, index):
        url = 'http://{}:{}/api/upload-image'.format(self.gcs_ip, self.gcs_http_port)
        for attempt in range(1, self.retry_count + 1):
            try:
                with open(filepath, 'rb') as f:
                    files = {'image': (os.path.basename(filepath), f, 'image/jpeg')}
                    data  = {'index': str(index)}
                    resp  = requests.post(url, files=files, data=data, timeout=10)
                if resp.status_code == 200:
                    rospy.loginfo('[DataTransfer] Image %s uploaded OK', os.path.basename(filepath))
                    return True
                rospy.logwarn('[DataTransfer] Image upload HTTP %d', resp.status_code)
            except Exception as e:
                rospy.logwarn('[DataTransfer] Image upload error attempt %d: %s', attempt, str(e))
            time.sleep(1.0)
        return False

    # ── Status publishing ──────────────────────────────────────────────────────

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        rospy.loginfo('[DataTransfer] Status: %s', status)

    # ── Logging ────────────────────────────────────────────────────────────────

    def _log_transfer(self, status, ok, fail):
        try:
            if not os.path.isdir(MISSION_DIR):
                os.makedirs(MISSION_DIR)
            ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            line = '{} | {} | ok={} fail={}\n'.format(ts, status, ok, fail)
            with open(TRANSFER_LOG, 'a') as f:
                f.write(line)
        except Exception as e:
            rospy.logwarn('[DataTransfer] Log write error: %s', str(e))


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    node = DataTransferNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
