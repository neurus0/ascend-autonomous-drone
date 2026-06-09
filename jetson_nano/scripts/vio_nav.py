#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
vio_nav.py
==========
VIO Navigation Node — IRoC-U 2026 Flight Stack
================================================
Based on: vio.py (tested, working)
Runs with: offboard_hover.py + lidar_alt.py (parallel)

Hardware : Jetson Nano + USB Webcam (MJPG, 640x480, 30fps)
Stack    : ROS 1 Melodic / Python 2.7

This node does everything vio.py does PLUS:
  - Publishes vision pose to /mavros/vision_pose/pose so PX4's EKF
    can use optical-flow-based position for GPS-denied OFFBOARD flight.
  - Subscribes to /lidar/altitude from lidar_alt node for accurate
    scale estimation (replaces the fixed 1.0 m assumption).
  - Publishes /vio/status (String) for mission-level health monitoring.

Topics Published:
    /vio/camera_feed            -- annotated camera frames (Image)
    /vio/drift                  -- [drift_px_x, drift_px_y, drift_metres]
    /vio/origin_locked          -- True once optical flow origin is locked
    /vio/status                 -- "SEARCHING" | "LOCKED" | "DEGRADED"
    /mavros/vision_pose/pose    -- vision pose for PX4 EKF fusion

Topics Subscribed:
    /lidar/altitude             -- Float32, metres AGL from lidar_alt node
    /mavros/local_position/pose -- for initial position reference

Run:
    python2 vio_nav.py
"""

import rospy
import cv2
import numpy as np
import threading
import time
import math
import os

from sensor_msgs.msg   import Imu, Image
from std_msgs.msg      import Bool, Float32, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_OK = True
except ImportError:
    _CV_BRIDGE_OK = False

# =============================================================================
# CONFIG
# =============================================================================

CAMERA_DEVICE         = 0         # /dev/video0
FRAME_W               = 640
FRAME_H               = 480
FRAME_FPS             = 30

MAX_FEATURES          = 300
FEATURE_QUALITY       = 0.01
FEATURE_MIN_DIST      = 10
LK_WIN_SIZE           = (21, 21)
LK_MAX_LEVEL          = 3
LK_CRITERIA           = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                          30, 0.01)
REDETECT_INTERVAL     = 30
MIN_FEATURES_FOR_LOCK = 20

# Approximate focal length for a generic 640x480 USB webcam
FOCAL_PX              = 600.0

# Vision pose publishing rate limit (Hz) — do not flood EKF
VISION_POSE_MAX_HZ    = 10

_DISPLAY_AVAILABLE    = bool(os.environ.get('DISPLAY', ''))


# =============================================================================
# OPEN CAMERA  (identical to proven vio.py)
# =============================================================================

def _open_camera():
    """
    Open USB webcam using MJPG format.
    MJPG is natively supported by OpenCV on Jetson and avoids
    the YUYV pixel format error.
    """
    rospy.loginfo('[VIO-NAV] Opening /dev/video%d (MJPG %dx%d %dfps) ...',
                  CAMERA_DEVICE, FRAME_W, FRAME_H, FRAME_FPS)

    # ── GStreamer MJPG pipeline (primary) ─────────────────────────────
    gst = (
        'v4l2src device=/dev/video{dev} ! '
        'image/jpeg, width={w}, height={h}, framerate={fps}/1 ! '
        'jpegdec ! '
        'videoconvert ! '
        'video/x-raw, format=BGR ! '
        'appsink max-buffers=1 drop=true'
    ).format(dev=CAMERA_DEVICE, w=FRAME_W, h=FRAME_H, fps=FRAME_FPS)

    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        time.sleep(1.0)
        for _ in range(5):
            try:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    rospy.loginfo('[VIO-NAV] Camera ready via GStreamer MJPG')
                    return cap
            except Exception:
                pass
            time.sleep(0.2)
        cap.release()

    # ── Fallback: plain OpenCV with MJPG fourcc ──────────────────────
    rospy.logwarn('[VIO-NAV] GStreamer failed, trying OpenCV direct ...')
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        cap.set(cv2.CAP_PROP_FPS,          FRAME_FPS)
        time.sleep(1.0)
        for _ in range(10):
            try:
                cap.read()
            except Exception:
                pass
            time.sleep(0.05)
        for _ in range(5):
            try:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    rospy.loginfo('[VIO-NAV] Camera ready via OpenCV direct')
                    return cap
            except Exception:
                pass
            time.sleep(0.2)
        cap.release()

    return None


# =============================================================================
# VIO ENGINE  (from proven vio.py, enhanced with vision-pose output)
# =============================================================================

class VIOEngine(object):
    """
    Optical-flow-based Visual-Inertial Odometry engine.
    Tracks pixel-level drift from a locked origin and converts to
    metric displacement using altitude from LiDAR.
    """

    def __init__(self):
        self._lock          = threading.Lock()
        self._prev_gray     = None
        self._prev_pts      = None
        self._frame_count   = 0
        self._origin_locked = False
        self._drift_px_x    = 0.0
        self._drift_px_y    = 0.0
        self.altitude_m     = 1.0        # updated by lidar
        self._feature_count = 0

    def set_altitude(self, alt_m):
        """Update altitude from LiDAR for metric scale."""
        with self._lock:
            self.altitude_m = max(0.1, alt_m)

    def is_locked(self):
        with self._lock:
            return self._origin_locked

    def get_drift(self):
        """Return (drift_px_x, drift_px_y, drift_metres)."""
        with self._lock:
            dx    = self._drift_px_x
            dy    = self._drift_px_y
            scale = self.altitude_m / FOCAL_PX
            dm    = math.sqrt(dx**2 + dy**2) * scale
        return dx, dy, dm

    def get_metric_xy(self):
        """Return (x_metres, y_metres) displacement from origin."""
        with self._lock:
            scale = self.altitude_m / FOCAL_PX
            mx = self._drift_px_x * scale
            my = self._drift_px_y * scale
        return mx, my

    def get_feature_count(self):
        with self._lock:
            return self._feature_count

    def reset_drift(self):
        with self._lock:
            self._drift_px_x = 0.0
            self._drift_px_y = 0.0

    def process_frame(self, frame_bgr):
        """
        Process one BGR frame through Lucas-Kanade optical flow.
        Returns annotated frame.
        """
        gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        annotated = frame_bgr.copy()
        h, w      = gray.shape
        cx, cy    = w // 2, h // 2

        with self._lock:
            self._frame_count += 1

            need_detect = (
                self._prev_pts is None or
                self._prev_gray is None or
                self._frame_count % REDETECT_INTERVAL == 0 or
                (self._prev_pts is not None and
                 len(self._prev_pts) < MIN_FEATURES_FOR_LOCK // 2)
            )

            if need_detect:
                pts = cv2.goodFeaturesToTrack(
                    gray,
                    maxCorners   = MAX_FEATURES,
                    qualityLevel = FEATURE_QUALITY,
                    minDistance   = FEATURE_MIN_DIST,
                    blockSize    = 7
                )
                self._prev_pts  = pts
                self._prev_gray = gray.copy()
                self._feature_count = len(pts) if pts is not None else 0
                cv2.drawMarker(annotated, (cx, cy),
                               (0, 255, 0), cv2.MARKER_CROSS, 40, 2)
                cv2.putText(annotated, 'VIO: SEARCHING',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 220, 255), 2)
                return annotated

            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, gray,
                self._prev_pts, None,
                winSize  = LK_WIN_SIZE,
                maxLevel = LK_MAX_LEVEL,
                criteria = LK_CRITERIA
            )

            good_prev = self._prev_pts[status == 1]
            good_next = next_pts[status == 1]
            self._feature_count = len(good_next)

            if len(good_next) >= 4:
                mean_flow = (good_next - good_prev).mean(axis=0)
            else:
                mean_flow = np.array([0.0, 0.0])

            if not self._origin_locked:
                if len(good_next) >= MIN_FEATURES_FOR_LOCK:
                    self._origin_locked = True
                    self._drift_px_x    = 0.0
                    self._drift_px_y    = 0.0
                    rospy.loginfo('[VIO-NAV] Origin LOCKED at centre (%d, %d)',
                                  cx, cy)

            if self._origin_locked:
                self._drift_px_x += mean_flow[0]
                self._drift_px_y += mean_flow[1]

            self._prev_pts  = good_next.reshape(-1, 1, 2)
            self._prev_gray = gray.copy()

            dx     = self._drift_px_x
            dy     = self._drift_px_y
            locked = self._origin_locked
            alt    = self.altitude_m

        # ── annotate outside lock ────────────────────────────────────
        for pt in good_next:
            cv2.circle(annotated, tuple(pt.astype(int)), 3, (0, 255, 100), -1)
        for p, n in zip(good_prev, good_next):
            cv2.arrowedLine(annotated,
                            tuple(p.astype(int)), tuple(n.astype(int)),
                            (200, 200, 0), 1, tipLength=0.4)

        # centre crosshair
        cv2.drawMarker(annotated, (cx, cy),
                       (0, 255, 0), cv2.MARKER_CROSS, 40, 2)

        # drift arrow
        if locked and (abs(dx) > 1 or abs(dy) > 1):
            cur_x = max(5, min(w - 5, int(cx + dx)))
            cur_y = max(5, min(h - 5, int(cy + dy)))
            cv2.arrowedLine(annotated, (cx, cy), (cur_x, cur_y),
                            (0, 0, 255), 2, tipLength=0.15)
            cv2.circle(annotated, (cur_x, cur_y), 6, (0, 0, 255), 2)

        status_str = 'VIO: LOCKED' if locked else 'VIO: SEARCHING'
        cv2.putText(annotated, status_str,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
        cv2.putText(annotated, 'FEATURES: %d' % len(good_next),
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 255, 100), 2)

        if locked:
            dm     = math.sqrt(dx**2 + dy**2) * (alt / FOCAL_PX)
            colour = (0, 80, 255) if dm > 0.05 else (0, 220, 0)
            cv2.putText(annotated,
                        'DRIFT px=(%.1f,%.1f) m=%.3f' % (dx, dy, dm),
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
            cv2.putText(annotated,
                        'ALT=%.2f m' % alt,
                        (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 200, 0), 2)

        return annotated


# =============================================================================
# ROS NODE
# =============================================================================

class VIONavNode(object):
    """
    VIO navigation node that runs alongside offboard_hover.py.
    Publishes vision pose for PX4 EKF fusion and drift data for
    the flight controller to monitor.
    """

    def __init__(self):
        self._vio    = VIOEngine()
        self._bridge = CvBridge() if _CV_BRIDGE_OK else None

        # ── Published topics ─────────────────────────────────────────
        self._cam_pub    = rospy.Publisher('/vio/camera_feed',
                                           Image,             queue_size=1)
        self._drift_pub  = rospy.Publisher('/vio/drift',
                                           Float32MultiArray, queue_size=5)
        self._locked_pub = rospy.Publisher('/vio/origin_locked',
                                           Bool,              queue_size=5,
                                           latch=True)
        self._status_pub = rospy.Publisher('/vio/status',
                                           String,            queue_size=5,
                                           latch=True)

        # ── Vision pose for PX4 EKF fusion ───────────────────────────
        # PX4 uses this for position estimation in GPS-denied mode.
        # Requires EKF2_AID_MASK bit 3 set and EKF2_HGT_MODE=Vision.
        self._vision_pub = rospy.Publisher('/mavros/vision_pose/pose',
                                           PoseStamped,       queue_size=5)

        # ── Subscribed topics ────────────────────────────────────────
        rospy.Subscriber('/mavros/imu/data',        Imu,     self._imu_cb)
        rospy.Subscriber('/lidar/altitude',         Float32, self._lidar_cb)
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self._local_pose_cb)

        # ── Internal state ───────────────────────────────────────────
        self._local_pose       = None    # latest mavros local pose
        self._last_vision_time = 0.0     # rate-limiter for vision pose
	self._node_start_time  = rospy.Time.now()
        self._vision_period    = 1.0 / VISION_POSE_MAX_HZ

        # ── Open camera ──────────────────────────────────────────────
        self._cap = _open_camera()
        if self._cap is None:
            rospy.logerr('[VIO-NAV] Could not open camera -- exiting')
            rospy.signal_shutdown('No camera')
            return

        rospy.loginfo('[VIO-NAV] Display=%s',
                      'ON' if _DISPLAY_AVAILABLE else
                      'OFF (headless) -- view /vio/camera_feed in rqt_image_view')

        # ── Timers ───────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(1.0 / FRAME_FPS), self._camera_loop)
        rospy.Timer(rospy.Duration(0.2),              self._drift_loop)
        rospy.Timer(rospy.Duration(1.0),              self._status_loop)

        rospy.loginfo('[VIO-NAV] Running — waiting for origin lock ...')

    # ── Callbacks ────────────────────────────────────────────────────

    def _imu_cb(self, msg):
        """Reserved for future IMU fusion."""
        pass

    def _lidar_cb(self, msg):
        """Update VIO engine altitude from LiDAR for metric scaling."""
        if msg.data > 0.05:
            self._vio.set_altitude(msg.data)

    def _local_pose_cb(self, msg):
        """Cache latest local pose as reference frame."""
        self._local_pose = msg

    # ── Camera loop ──────────────────────────────────────────────────

    def _camera_loop(self, _event):
        """Grab frame, run optical flow, publish annotated + vision pose."""
        if self._cap is None:
            return
        try:
            ret, frame = self._cap.read()
            if not ret or frame is None or frame.size == 0:
                rospy.logwarn_throttle(5, '[VIO-NAV] Empty frame skipped')
                return

            annotated = self._vio.process_frame(frame)

            # ── Display if GUI available ─────────────────────────────
            if _DISPLAY_AVAILABLE:
                cv2.imshow('VIO Nav', annotated)
                cv2.waitKey(1)

            # ── Publish annotated frame ──────────────────────────────
            if _CV_BRIDGE_OK and self._bridge is not None:
                msg = self._bridge.cv2_to_imgmsg(annotated, 'bgr8')
                msg.header.stamp = rospy.Time.now()
                self._cam_pub.publish(msg)

            # ── Publish origin lock status ───────────────────────────
            lmsg      = Bool()
            lmsg.data = self._vio.is_locked()
            self._locked_pub.publish(lmsg)

            # ── Publish vision pose for PX4 EKF ─────────────────────
            now = time.time()
            if (self._vio.is_locked() and
                    now - self._last_vision_time >= self._vision_period):
                self._last_vision_time = now
                self._publish_vision_pose()

        except Exception as exc:
            rospy.logwarn_throttle(5, '[VIO-NAV] Frame error: %s', str(exc))

	def _publish_vision_pose(self):
		dx, dy, dm = self._vio.get_drift()
		if dm > 0.5:
		    return
		# Don't publish for first 5 seconds — let EKF settle on baro/compass
		if (rospy.Time.now() - self._node_start_time).to_sec() < 5.0:
		    return
		mx, my = self._vio.get_metric_xy()
		vp = PoseStamped()
		vp.header.stamp    = rospy.Time.now()
		vp.header.frame_id = "map"
		vp.pose.position.x = mx
		vp.pose.position.y = my
		vp.pose.position.z = 0.0
		vp.pose.orientation.x = 0.0
		vp.pose.orientation.y = 0.0
		vp.pose.orientation.z = 0.0
		vp.pose.orientation.w = 1.0
		self._vision_pub.publish(vp)

    # ── Drift loop ───────────────────────────────────────────────────

    def _drift_loop(self, _event):
        """Publish VIO drift at 5 Hz."""
        dx, dy, dm = self._vio.get_drift()
        dmsg       = Float32MultiArray()
        dmsg.data  = [float(dx), float(dy), float(dm)]
        self._drift_pub.publish(dmsg)
        if self._vio.is_locked():
            rospy.loginfo_throttle(2,
                '[VIO-NAV] drift px=(%.1f, %.1f)  metric=%.4f m', dx, dy, dm)
            if dm > 0.1:
                rospy.logwarn('[VIO-NAV] DRIFT > 0.1m: %.4f m', dm)

    # ── Status loop ──────────────────────────────────────────────────

    def _status_loop(self, _event):
        """Publish human-readable VIO status at 1 Hz."""
        smsg = String()
        if not self._vio.is_locked():
            smsg.data = "SEARCHING"
        elif self._vio.get_feature_count() < MIN_FEATURES_FOR_LOCK:
            smsg.data = "DEGRADED"
        else:
            smsg.data = "LOCKED"
        self._status_pub.publish(smsg)

    # ── Shutdown ─────────────────────────────────────────────────────

    def shutdown(self):
        if self._cap is not None:
            self._cap.release()
        if _DISPLAY_AVAILABLE:
            cv2.destroyAllWindows()
        rospy.loginfo('[VIO-NAV] Shutdown complete')


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == '__main__':
    rospy.init_node('vio_nav_node')
    rospy.loginfo('=' * 60)
    rospy.loginfo('vio_nav.py — VIO Navigation Node (IRoC-U 2026)')
    rospy.loginfo('Publishes: /vio/camera_feed  /vio/drift  /vio/origin_locked')
    rospy.loginfo('           /vio/status  /mavros/vision_pose/pose')
    rospy.loginfo('Subscribes: /lidar/altitude  /mavros/imu/data')
    rospy.loginfo('=' * 60)
    node = VIONavNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
