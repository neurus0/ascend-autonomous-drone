#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
throttle_hover.py  —  LiDAR-Based Throttle Hover
==================================================
IRoC-U 2026 — GPS-Denied Autonomous Flight

BYPASS STRATEGY: Pixhawk barometer is not calibrated correctly, so we
do NOT use GUIDED mode or Pixhawk's altitude controller. Instead:
  - STABILIZE mode (no EKF position needed, just gyro stabilisation)
  - RC override for throttle (same as offboard_hover(1).py which WORKS)
  - LiDAR for altitude feedback (this works correctly)
  - We control altitude ourselves via simple throttle adjustments

Phases:
  1. WAIT_FCU      — wait for Pixhawk connection
  2. WAIT_LIDAR    — wait for valid LiDAR readings
  3. WAIT_START    — wait for START command from GCS via /gcs/flight_command
  4. SET_STABILIZE — switch to STABILIZE mode
  5. ARM           — arm the motors
  6. RAMP_UP       — increase throttle from BASE_THROTTLE until LiDAR increases
  7. CLIMB         — continue increasing throttle until TARGET_ALT reached
  8. HOVER         — hold altitude at TARGET_ALT for HOVER_DURATION
  9. DESCEND       — trigger kill.py for graceful descent
  10. DONE          — mission complete

Configurable parameters: TARGET_ALT_M, HOVER_DURATION_S, BASE_THROTTLE_PCT

Usage:
  Launched by unified_launch.launch — waits for START from tcp_bridge.py

Author: IRoC-U ASCEND 2026 Team
"""

import rospy
import sys
import time
import threading

from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg    import Float32, Bool, String

# Import kill controller
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kill import KillController

# =============================================================================
# CONFIGURATION — ADJUSTABLE PARAMETERS
# =============================================================================

TARGET_ALT_M       = 1.0      # metres — target hover altitude
HOVER_DURATION_S   = 5.0      # seconds — how long to hover at target
BASE_THROTTLE_PCT  = 0       # starting throttle percentage
MAX_THROTTLE_PCT   = 80       # hard cap — never exceed this during flight
THROTTLE_INCREMENT = 5        # PWM units to increase per step during ramp
ALT_TOLERANCE_M    = 0.10     # ±metres considered "at target"
ALT_INCREASE_THRESH = 0.05   # metres — altitude increase to detect lift
RAMP_INTERVAL_S    = 0.5      # seconds between throttle increases during ramp
HOVER_ADJUST_RATE  = 1        # PWM units to adjust during hover hold
RATE_HZ            = 10       # main loop rate
NO_OVERRIDE        = 0        # MAVROS "no override"

# Derived
BASE_THROTTLE_PWM  = 1000 + int((BASE_THROTTLE_PCT / 100.0) * 1000)
MAX_THROTTLE_PWM   = 1000 + int((MAX_THROTTLE_PCT  / 100.0) * 1000)  # 1800


# =============================================================================
# FLIGHT PHASES
# =============================================================================

class Phase(object):
    WAIT_FCU       = "WAIT_FCU"
    WAIT_LIDAR     = "WAIT_LIDAR"
    WAIT_START     = "WAIT_START"
    SET_STABILIZE  = "SET_STABILIZE"
    ARM            = "ARM"
    RAMP_UP        = "RAMP_UP"
    CLIMB          = "CLIMB"
    HOVER          = "HOVER"
    DESCEND        = "DESCEND"
    DONE           = "DONE"


# =============================================================================
# NODE
# =============================================================================

class ThrottleHoverNode(object):

    def __init__(self):
        rospy.init_node('throttle_hover_node', anonymous=False)

        rospy.loginfo("=" * 60)
        rospy.loginfo("IRoC-U 2026 | LiDAR Throttle Hover  [THL Mission]")
        rospy.loginfo("Target: %.1f m | Hover: %.0f s | Base: %d%% | Max: %d%%",
                       TARGET_ALT_M, HOVER_DURATION_S, BASE_THROTTLE_PCT, MAX_THROTTLE_PCT)
        rospy.loginfo("=" * 60)

        # ── State ──────────────────────────────────────────────────
        self._phase          = Phase.WAIT_FCU
        self._fcu_state      = State()
        self._lidar_alt      = 0.0
        self._lidar_stamp    = 0.0
        self._lidar_valid    = False
        self._throttle_pwm   = BASE_THROTTLE_PWM
        self._hover_start    = None
        self._ramp_start_alt = 0.0
        self._last_ramp_time = 0.0
        self._retry_count    = 0
        self._start_received = False
        self._kill_requested = False
        self._lift_throttle  = BASE_THROTTLE_PWM  # throttle at which drone started lifting

        # ── Kill controller ────────────────────────────────────────
        self._kill_ctrl = KillController(standalone=False)

        # ── Publishers ─────────────────────────────────────────────
        self._rc_pub = rospy.Publisher(
            '/mavros/rc/override', OverrideRCIn, queue_size=10)
        self._state_pub = rospy.Publisher(
            '/drone/flight_state', String, queue_size=5, latch=True)

        # ── Subscribers ────────────────────────────────────────────
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/lidar/altitude', Float32, self._lidar_cb)
        rospy.Subscriber('/gcs/flight_command', String, self._cmd_cb)
        rospy.Subscriber('/gcs/kill_flag', Bool, self._kill_cb)

        # ── Services ──────────────────────────────────────────────
        rospy.loginfo("[HOVER] Waiting for MAVROS services...")
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        rospy.wait_for_service('/mavros/set_mode', timeout=30)

        self._arm_srv  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo("[HOVER] Services ready.")

    # ── Callbacks ──────────────────────────────────────────────────

    def _state_cb(self, msg):
        self._fcu_state = msg

    def _lidar_cb(self, msg):
        self._lidar_alt = msg.data
        self._lidar_stamp = time.time()
        if msg.data > 0.01:
            self._lidar_valid = True

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "START":
            rospy.loginfo("[HOVER] START command received from GCS.")
            self._start_received = True
        elif cmd == "KILL":
            rospy.logwarn("[HOVER] KILL command received from GCS.")
            self._kill_requested = True

    def _kill_cb(self, msg):
        if msg.data:
            rospy.logwarn("[HOVER] Kill flag set.")
            self._kill_requested = True

    # ── Helpers ────────────────────────────────────────────────────

    def _lidar_fresh(self):
        return (time.time() - self._lidar_stamp) < 1.0

    def _set_stabilize(self):
        try:
            req = SetModeRequest()
            req.custom_mode = "STABILIZE"
            resp = self._mode_srv(req)
            return resp.mode_sent
        except rospy.ServiceException:
            return False

    def _arm(self):
        try:
            req = CommandBoolRequest()
            req.value = True
            resp = self._arm_srv(req)
            return resp.success
        except rospy.ServiceException:
            return False

    def _send_rc(self, throttle_pwm):
        msg = OverrideRCIn()
        msg.channels = [
            NO_OVERRIDE,     # ch1 roll
            NO_OVERRIDE,     # ch2 pitch
            throttle_pwm,    # ch3 throttle
            NO_OVERRIDE,     # ch4 yaw
            NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE,
            NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE,
            NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE,
            NO_OVERRIDE, NO_OVERRIDE,
        ]
        self._rc_pub.publish(msg)

    def _publish_state(self, state):
        self._state_pub.publish(String(data=state))

    # ── State Machine ──────────────────────────────────────────────

    def _tick(self):
        # ── Check for kill at any time ─────────────────────────────
        if self._kill_requested and self._phase not in (Phase.DESCEND, Phase.DONE,
                                                         Phase.WAIT_FCU, Phase.WAIT_LIDAR,
                                                         Phase.WAIT_START):
            rospy.logwarn("[HOVER] Kill requested! Starting descent.")
            self._phase = Phase.DESCEND

        # ── If descending via kill controller, keep sending rc ─────
        if self._kill_ctrl.is_descending():
            return
        if self._kill_ctrl.is_complete() and self._phase != Phase.DONE:
            self._phase = Phase.DONE

        # ── WAIT_FCU ──────────────────────────────────────────────
        if self._phase == Phase.WAIT_FCU:
            if self._fcu_state.connected:
                rospy.loginfo("[HOVER] FCU connected. mode=%s",
                              self._fcu_state.mode)
                self._phase = Phase.WAIT_LIDAR
            else:
                rospy.loginfo_throttle(2, "[HOVER] Waiting for FCU...")

        # ── WAIT_LIDAR ────────────────────────────────────────────
        elif self._phase == Phase.WAIT_LIDAR:
            if self._lidar_valid and self._lidar_fresh():
                rospy.loginfo("[HOVER] LiDAR OK. Alt: %.2fm", self._lidar_alt)
                self._phase = Phase.WAIT_START
                self._publish_state("READY")
            else:
                rospy.loginfo_throttle(2, "[HOVER] Waiting for LiDAR...")

        # ── WAIT_START ────────────────────────────────────────────
        elif self._phase == Phase.WAIT_START:
            if self._start_received:
                rospy.loginfo("[HOVER] START received. Beginning mission.")
                self._publish_state("STARTING")
                self._phase = Phase.SET_STABILIZE
                self._retry_count = 0
            else:
                rospy.loginfo_throttle(3, "[HOVER] Waiting for START from GCS...")

        # ── SET_STABILIZE ─────────────────────────────────────────
        elif self._phase == Phase.SET_STABILIZE:
            if self._fcu_state.mode == "STABILIZE":
                rospy.loginfo("[HOVER] STABILIZE mode confirmed.")
                self._phase = Phase.ARM
                self._retry_count = 0
            else:
                if self._set_stabilize():
                    rospy.loginfo("[HOVER] STABILIZE requested...")
                rospy.sleep(1.0)
                self._retry_count += 1
                if self._retry_count > 10:
                    rospy.logerr("[HOVER] Cannot set STABILIZE after 10 tries.")
                    self._publish_state("MODE_FAIL")
                    self._phase = Phase.DONE

        # ── ARM ───────────────────────────────────────────────────
        elif self._phase == Phase.ARM:
            if self._fcu_state.armed:
                rospy.loginfo("[HOVER] ARMED.")
                self._publish_state("ARMED")
                self._throttle_pwm = BASE_THROTTLE_PWM
                self._ramp_start_alt = self._lidar_alt
                self._last_ramp_time = time.time()
                self._phase = Phase.RAMP_UP
            else:
                if self._arm():
                    rospy.loginfo("[HOVER] Arm command sent...")
                else:
                    rospy.logwarn("[HOVER] Arm rejected, retrying...")
                rospy.sleep(1.0)

        # ── RAMP_UP — increase throttle until drone starts lifting ──
        elif self._phase == Phase.RAMP_UP:
            self._send_rc(self._throttle_pwm)
            self._publish_state("MOTORS RUNNING")

            now = time.time()
            if now - self._last_ramp_time >= RAMP_INTERVAL_S:
                self._last_ramp_time = now

                # Check if LiDAR shows altitude increase
                alt_change = self._lidar_alt - self._ramp_start_alt
                if alt_change > ALT_INCREASE_THRESH:
                    self._lift_throttle = self._throttle_pwm
                    rospy.loginfo("[HOVER] LIFT DETECTED at throttle %d PWM (alt change: +%.2fm)",
                                  self._throttle_pwm, alt_change)
                    self._publish_state("TAKEOFF")
                    self._phase = Phase.CLIMB
                else:
                    # Increase throttle
                    self._throttle_pwm += THROTTLE_INCREMENT
                    if self._throttle_pwm > MAX_THROTTLE_PWM:
                        rospy.logerr("[HOVER] Throttle at %d%% cap — no lift detected.",
                                     MAX_THROTTLE_PCT)
                        self._publish_state("TAKEOFF_FAIL")
                        self._phase = Phase.DESCEND

                rospy.loginfo_throttle(0.5,
                    "[HOVER] RAMP — throttle: %d PWM | alt: %.2fm | change: %.3fm",
                    self._throttle_pwm, self._lidar_alt,
                    self._lidar_alt - self._ramp_start_alt)

            if not self._fcu_state.armed:
                rospy.logwarn("[HOVER] Disarmed during ramp!")
                self._phase = Phase.DONE

        # ── CLIMB — continue to target altitude ────────────────────
        elif self._phase == Phase.CLIMB:
            alt = self._lidar_alt

            # Simple proportional adjustment
            error = TARGET_ALT_M - alt
            if error > ALT_TOLERANCE_M:
                # Below target — increase throttle slightly
                self._throttle_pwm += 1
            elif error < -ALT_TOLERANCE_M:
                # Above target — decrease throttle slightly
                self._throttle_pwm -= 1

            # Clamp throttle
            self._throttle_pwm = max(self._lift_throttle - 20,
                                     min(MAX_THROTTLE_PWM, self._throttle_pwm))

            self._send_rc(self._throttle_pwm)

            rospy.loginfo_throttle(1,
                "[HOVER] CLIMBING — alt: %.2fm / target: %.1fm | throttle: %d",
                alt, TARGET_ALT_M, self._throttle_pwm)

            if abs(error) <= ALT_TOLERANCE_M:
                rospy.loginfo("[HOVER] TARGET ALTITUDE REACHED: %.2fm", alt)
                self._hover_start = time.time()
                self._publish_state("HOVER")
                self._phase = Phase.HOVER

            if not self._fcu_state.armed:
                rospy.logwarn("[HOVER] Disarmed during climb!")
                self._phase = Phase.DONE

        # ── HOVER — hold altitude for HOVER_DURATION ───────────────
        elif self._phase == Phase.HOVER:
            alt = self._lidar_alt
            elapsed = time.time() - self._hover_start

            # Simple altitude hold — adjust throttle to maintain target
            error = TARGET_ALT_M - alt
            if error > ALT_TOLERANCE_M:
                self._throttle_pwm += HOVER_ADJUST_RATE
            elif error < -ALT_TOLERANCE_M:
                self._throttle_pwm -= HOVER_ADJUST_RATE

            self._throttle_pwm = max(self._lift_throttle - 30,
                                     min(MAX_THROTTLE_PWM, self._throttle_pwm))

            self._send_rc(self._throttle_pwm)

            rospy.loginfo_throttle(2,
                "[HOVER] HOVERING — alt: %.2fm | %.1f / %.0f s | throttle: %d",
                alt, elapsed, HOVER_DURATION_S, self._throttle_pwm)

            if elapsed >= HOVER_DURATION_S:
                rospy.loginfo("=" * 60)
                rospy.loginfo("[HOVER] HOVER COMPLETE — %.0fs at ~%.1fm",
                              HOVER_DURATION_S, TARGET_ALT_M)
                rospy.loginfo("[HOVER] TASK-1 (Takeoff)  : PASS")
                rospy.loginfo("[HOVER] TASK-2 (Hover)    : PASS")
                rospy.loginfo("=" * 60)
                self._publish_state("HOVER_COMPLETE")
                self._phase = Phase.DESCEND

            if not self._fcu_state.armed:
                rospy.logwarn("[HOVER] Disarmed during hover!")
                self._phase = Phase.DONE

        # ── DESCEND — use kill controller ──────────────────────────
        elif self._phase == Phase.DESCEND:
            rospy.loginfo("[HOVER] Starting descent via KillController.")
            self._publish_state("DESCENDING")
            self._kill_ctrl.begin_descent(self._throttle_pwm)
            # Don't change phase — the kill controller will handle it
            # Check in the next tick via is_complete()

        # ── DONE ──────────────────────────────────────────────────
        elif self._phase == Phase.DONE:
            rospy.loginfo_throttle(5, "[HOVER] MISSION COMPLETE.")
            self._publish_state("MISSION_COMPLETE")
            # Reset for next mission
            self._phase = Phase.WAIT_START
            self._start_received = False
            self._kill_requested = False
            self._hover_start = None
            self._throttle_pwm = BASE_THROTTLE_PWM
            rospy.loginfo("[HOVER] Ready for next mission. Waiting for START...")

    # ── Main loop ──────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        rospy.loginfo("[HOVER] Loop starting at %d Hz.", RATE_HZ)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()
        rospy.loginfo("[HOVER] Shutdown.")


# =============================================================================
# ENTRY
# =============================================================================

if __name__ == '__main__':
    try:
        node = ThrottleHoverNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[HOVER] ROS interrupt. Exiting.")
    except Exception as e:
        rospy.logerr("[HOVER] Fatal: %s", str(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)
