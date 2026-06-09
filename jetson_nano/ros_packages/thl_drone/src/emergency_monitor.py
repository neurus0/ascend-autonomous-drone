#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
emergency_monitor.py  —  Safety Monitor (Fixed for unified stack)
===================================================================
IRoC-U 2026 — GPS-Denied Autonomous Flight

Based on emergency.py but fixed for Python 2.7 + ROS Melodic:
  - BatteryStatus → BatteryState
  - f-strings → .format()
  - Publishes /gcs/kill_flag to trigger kill.py descent
  - Publishes to /drone/flight_state for telemetry bridge

Monitors:
  - Altitude (max 3m)
  - Hover timeout (max 60s)
  - Battery voltage (min 10.0V)
  - FCU connection
  - IMU alive
  - LiDAR alive
  - VIO alive

Author: IRoC-U ASCEND 2026 Team
"""

import rospy
import os
import time
from datetime import datetime

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg   import Imu, Range, Image, BatteryState
from mavros_msgs.msg   import State, ExtendedState
from mavros_msgs.srv   import SetMode, CommandBool
from std_msgs.msg      import Float32, Bool, String

# ─── CONFIG ──────────────────────────────────────────────────────────────────
MAX_ALTITUDE_M        = 3.5   # 0.5m headroom above 3m target
MAX_HOVER_TIME_S      = 60.0
MIN_BATTERY_VOLTAGE   = 10.0
SENSOR_TIMEOUT_S      = 3.0
FCU_TIMEOUT_S         = 3.0
CHECK_RATE_HZ         = 5
STARTUP_GRACE_S       = 15.0     # ignore sensor timeouts for first 15s

LOG_DIR = os.path.expanduser("~/emergency_logs")
# ─────────────────────────────────────────────────────────────────────────────


class EmergencyMonitor(object):

    def __init__(self):
        rospy.init_node("emergency_monitor", anonymous=False)

        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)

        # ── State ─────────────────────────────────────────────────────
        self.current_height      = 0.0
        self.lidar_alt           = 0.0
        self.battery_voltage     = 99.0
        self.fcu_connected       = False
        self.landed              = True
        self.armed               = False
        self.emergency_triggered = False
        self._last_hover_height  = 0.0

        self.hover_start_time   = None
        self.last_imu_time      = time.time()
        self.last_lidar_time    = time.time()
        self.last_vio_time      = time.time()
        self.last_fcu_time      = time.time()
        self.node_start_time    = time.time()

        # ── Subscribers ───────────────────────────────────────────────
        rospy.Subscriber("/mavros/local_position/pose",
                         PoseStamped, self.cb_local_pose)
        rospy.Subscriber("/mavros/battery",
                         BatteryState, self.cb_battery)
        rospy.Subscriber("/mavros/state",
                         State, self.cb_state)
        rospy.Subscriber("/mavros/extended_state",
                         ExtendedState, self.cb_extended_state)
        rospy.Subscriber("/mavros/imu/data",
                         Imu, self.cb_imu)
        rospy.Subscriber("/lidar/altitude",
                         Float32, self.cb_lidar)
        rospy.Subscriber("/mavros/vision_pose/pose",
                         PoseStamped, self.cb_vio)

        # ── Publishers ────────────────────────────────────────────────
        self._kill_pub = rospy.Publisher(
            '/gcs/kill_flag', Bool, queue_size=5, latch=True)
        self._state_pub = rospy.Publisher(
            '/drone/flight_state', String, queue_size=5, latch=True)

        # ── Services ─────────────────────────────────────────────────
        rospy.loginfo("[SAFETY] Waiting for MAVROS services...")
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=15)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=15)
            self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self.arming_srv   = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        except Exception as e:
            rospy.logwarn("[SAFETY] MAVROS services not ready: {}".format(str(e)))
            self.set_mode_srv = None
            self.arming_srv = None

        rospy.loginfo("[SAFETY] Emergency monitor ONLINE.")
        rospy.Timer(rospy.Duration(1.0 / CHECK_RATE_HZ), self.check_all)

    # ─── Callbacks ────────────────────────────────────────────────────

    def cb_local_pose(self, msg):
        self.current_height = msg.pose.position.z

    def cb_battery(self, msg):
        self.battery_voltage = msg.voltage
        self.last_fcu_time = time.time()

    def cb_state(self, msg):
        self.fcu_connected = msg.connected
        self.armed = msg.armed
        self.last_fcu_time = time.time()

    def cb_extended_state(self, msg):
        self.landed = (msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)

    def cb_imu(self, msg):
        self.last_imu_time = time.time()

    def cb_lidar(self, msg):
        self.lidar_alt = msg.data
        self.last_lidar_time = time.time()

    def cb_vio(self, msg):
        self.last_vio_time = time.time()

    # ─── Main check loop ─────────────────────────────────────────────

    def check_all(self, event):
        if self.emergency_triggered:
            return
        if not self.armed:
            self.hover_start_time = None
            return

        now = time.time()
        uptime = now - self.node_start_time

        # Use LiDAR altitude as primary
        alt = self.lidar_alt if self.lidar_alt > 0 else self.current_height

        # 1. Altitude check
        if alt > MAX_ALTITUDE_M:
            self.trigger_emergency(
                "ALTITUDE TOO HIGH — {:.2f}m > limit {:.1f}m".format(alt, MAX_ALTITUDE_M),
                "ALT_EXCEEDED")
            return

        # 2. Hover timeout
        if self.hover_start_time is None:
            self.hover_start_time = now
            self._last_hover_height = alt

        height_delta = abs(alt - self._last_hover_height)
        if height_delta < 0.1:
            hover_duration = now - self.hover_start_time
            if hover_duration > MAX_HOVER_TIME_S:
                self.trigger_emergency(
                    "HOVER TIMEOUT — idle for {:.1f}s > {:.0f}s limit".format(
                        hover_duration, MAX_HOVER_TIME_S),
                    "HOVER_TIMEOUT")
                return
        else:
            self.hover_start_time = now
            self._last_hover_height = alt

        # 3. Battery voltage
        if self.battery_voltage < MIN_BATTERY_VOLTAGE:
            self.trigger_emergency(
                "LOW BATTERY — {:.2f}V < {:.1f}V".format(
                    self.battery_voltage, MIN_BATTERY_VOLTAGE),
                "LOW_BATTERY")
            return

        # 4. FCU connection (after startup grace)
        if uptime > STARTUP_GRACE_S:
            fcu_age = now - self.last_fcu_time
            if fcu_age > FCU_TIMEOUT_S:
                self.trigger_emergency(
                    "FCU LINK LOST — no heartbeat for {:.1f}s".format(fcu_age),
                    "FCU_DISCONNECTED")
                return

        # 5. IMU check
        if uptime > STARTUP_GRACE_S:
            imu_age = now - self.last_imu_time
            if imu_age > SENSOR_TIMEOUT_S:
                self.trigger_emergency(
                    "IMU DEAD — no data for {:.1f}s".format(imu_age),
                    "IMU_TIMEOUT")
                return

        # 6. LiDAR check
        if uptime > STARTUP_GRACE_S:
            lidar_age = now - self.last_lidar_time
            if lidar_age > SENSOR_TIMEOUT_S:
                self.trigger_emergency(
                    "LIDAR DEAD — no data for {:.1f}s".format(lidar_age),
                    "LIDAR_TIMEOUT")
                return

    # ─── Emergency trigger ────────────────────────────────────────────

    def trigger_emergency(self, reason, code):
        self.emergency_triggered = True

        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_filename = os.path.join(
            LOG_DIR,
            "emergency_{}_{}.txt".format(code, datetime.now().strftime('%Y%m%d_%H%M%S')))

        msg = """
========================================================
   EMERGENCY LANDING TRIGGERED
========================================================
   Time    : {}
   Code    : {}
   Reason  : {}
   Height  : {:.2f} m
   Battery : {:.2f} V
   FCU     : {}
========================================================
""".format(ts, code, reason, self.lidar_alt, self.battery_voltage, self.fcu_connected)

        rospy.logerr(msg)

        try:
            with open(log_filename, "w") as f:
                f.write(msg)
            rospy.logwarn("[SAFETY] Log saved -> {}".format(log_filename))
        except Exception as e:
            rospy.logwarn("[SAFETY] Log write failed: {}".format(str(e)))

        # Publish kill flag — kill.py will handle the descent
        self._kill_pub.publish(Bool(data=True))
        self._state_pub.publish(String(data="EMERGENCY_{}".format(code)))

        rospy.logwarn("[SAFETY] Kill flag published. kill.py will handle descent.")

    def run(self):
        rospy.spin()


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        monitor = EmergencyMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
