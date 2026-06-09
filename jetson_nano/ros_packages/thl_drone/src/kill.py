#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
kill.py  —  Graceful Throttle-Based Descent
=============================================
IRoC-U 2026 — GPS-Denied Autonomous Flight

This module handles graceful descent by reducing throttle while monitoring
LiDAR altitude. It does NOT use MAVLink LAND mode (barometer is unreliable).

Can be used in 3 ways:
  1. Imported by throttle_hover.py:  kill_instance.begin_descent(current_throttle)
  2. Triggered via ROS topic:        /drone/kill_trigger (Bool)
  3. Called from emergency_monitor.py

Logic:
  - Decrease throttle by DESCENT_RATE PWM units per second
  - Monitor LiDAR: if altitude unchanged (±0.05m) for GROUND_DETECT_S → on ground
  - Once on ground: set throttle to 1000 (zero), wait 0.5s, then DISARM via MAVLink
  - Publish state to /drone/flight_state for telemetry

Author: IRoC-U ASCEND 2026 Team
"""

import rospy
import time
import threading

from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg   import Float32, Bool, String

# ─── CONFIG ───────────────────────────────────────────────────────────────────
DESCENT_RATE        = 2       # PWM units to decrease per second
GROUND_DETECT_S     = 5.0     # seconds of unchanged LiDAR = on ground
GROUND_ALT_THRESH   = 0.05    # metres — altitude change below this = "unchanged"
MIN_THROTTLE_PWM    = 1000    # absolute minimum (motors off)
DESCENT_CHECK_HZ    = 10      # how fast the descent loop runs
NO_OVERRIDE         = 0       # MAVROS "no override" value
DISARM_RETRY        = 5       # number of disarm attempts
# ──────────────────────────────────────────────────────────────────────────────


class KillController(object):
    """
    Graceful descent controller.
    Can be instantiated by throttle_hover.py or run standalone.
    """

    def __init__(self, standalone=False):
        if standalone:
            rospy.init_node('kill_node', anonymous=False)

        self._lock = threading.Lock()

        # State
        self._descending      = False
        self._current_throttle = MIN_THROTTLE_PWM
        self._lidar_alt       = 0.0
        self._lidar_stamp     = 0.0
        self._ground_start    = None    # when we first detected "no change"
        self._last_alt_check  = 0.0     # altitude at last check
        self._fcu_state       = State()
        self._descent_complete = False

        # Publishers
        self._rc_pub    = rospy.Publisher('/mavros/rc/override',
                                         OverrideRCIn, queue_size=10)
        self._state_pub = rospy.Publisher('/drone/flight_state',
                                         String, queue_size=5, latch=True)
        self._kill_active_pub = rospy.Publisher('/drone/kill_active',
                                                Bool, queue_size=5, latch=True)

        # Subscribers
        rospy.Subscriber('/lidar/altitude', Float32, self._lidar_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)

        if standalone:
            rospy.Subscriber('/drone/kill_trigger', Bool, self._trigger_cb)

        # Services (wait only if they exist)
        self._arm_srv  = None
        self._mode_srv = None
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
            rospy.wait_for_service('/mavros/set_mode', timeout=10)
            self._arm_srv  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self._mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        except Exception as e:
            rospy.logwarn('[KILL] MAVROS services not available: %s', str(e))

        rospy.loginfo('[KILL] KillController initialised.')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _lidar_cb(self, msg):
        with self._lock:
            self._lidar_alt = msg.data
            self._lidar_stamp = time.time()

    def _state_cb(self, msg):
        self._fcu_state = msg

    def _trigger_cb(self, msg):
        if msg.data and not self._descending:
            rospy.logwarn('[KILL] Kill triggered via /drone/kill_trigger')
            # We don't know current throttle from outside, use a safe default
            self.begin_descent(1200)

    # ── Public API ─────────────────────────────────────────────────────────

    def begin_descent(self, current_throttle_pwm):
        """
        Start the descent sequence.
        current_throttle_pwm: the throttle PWM value to start descending from.
        """
        if self._descending:
            rospy.logwarn('[KILL] Already descending. Ignoring.')
            return

        with self._lock:
            self._descending = True
            self._descent_complete = False
            self._current_throttle = current_throttle_pwm
            self._last_alt_check = self._lidar_alt
            self._ground_start = None

        self._kill_active_pub.publish(Bool(data=True))
        self._state_pub.publish(String(data='DESCENDING'))
        rospy.logwarn('[KILL] Descent started. Throttle: %d PWM', current_throttle_pwm)

        # Run descent in a thread so it doesn't block the caller
        t = threading.Thread(target=self._descent_loop)
        t.daemon = True
        t.start()

    def is_descending(self):
        return self._descending

    def is_complete(self):
        return self._descent_complete

    # ── Descent loop ───────────────────────────────────────────────────────

    def _descent_loop(self):
        rate_sleep = 1.0 / DESCENT_CHECK_HZ
        pwm_decrement_per_tick = float(DESCENT_RATE) / DESCENT_CHECK_HZ

        rospy.logwarn('[KILL] Descent loop running at %d Hz', DESCENT_CHECK_HZ)

        while not rospy.is_shutdown() and self._descending:
            with self._lock:
                current_alt = self._lidar_alt
                lidar_age = time.time() - self._lidar_stamp

            # ── Decrease throttle ──────────────────────────────────────
            self._current_throttle -= pwm_decrement_per_tick
            if self._current_throttle < MIN_THROTTLE_PWM:
                self._current_throttle = MIN_THROTTLE_PWM

            # Send RC override with current throttle
            self._send_rc(int(self._current_throttle))

            # ── Ground detection via LiDAR ─────────────────────────────
            if lidar_age < 1.0:  # LiDAR data is fresh
                alt_change = abs(current_alt - self._last_alt_check)

                if alt_change < GROUND_ALT_THRESH:
                    # Altitude not changing — might be on ground
                    if self._ground_start is None:
                        self._ground_start = time.time()
                        rospy.loginfo('[KILL] Ground detect started. Alt: %.2fm', current_alt)
                    else:
                        ground_time = time.time() - self._ground_start
                        if ground_time >= GROUND_DETECT_S:
                            rospy.logwarn('[KILL] Ground confirmed (%.1fs at %.2fm). Disarming.',
                                         ground_time, current_alt)
                            self._on_ground()
                            return
                else:
                    # Still descending — reset ground timer
                    self._ground_start = None
                    self._last_alt_check = current_alt
            else:
                # LiDAR data is stale — keep descending blindly
                rospy.logwarn_throttle(2, '[KILL] LiDAR stale (%.1fs). Descending blind.', lidar_age)

            # ── Safety: if throttle is at minimum for too long, assume ground ─
            if self._current_throttle <= MIN_THROTTLE_PWM:
                if self._ground_start is None:
                    self._ground_start = time.time()
                elif time.time() - self._ground_start >= GROUND_DETECT_S:
                    rospy.logwarn('[KILL] Throttle at minimum for %ds. Assuming ground.', GROUND_DETECT_S)
                    self._on_ground()
                    return

            rospy.loginfo_throttle(1,
                '[KILL] DESCENDING — throttle: %d | alt: %.2fm | ground_timer: %s',
                int(self._current_throttle), current_alt,
                '%.1fs' % (time.time() - self._ground_start) if self._ground_start else 'N/A')

            time.sleep(rate_sleep)

    def _on_ground(self):
        """Called when the drone is confirmed on the ground."""
        # Zero throttle
        for _ in range(10):
            self._send_rc(MIN_THROTTLE_PWM)
            time.sleep(0.05)

        time.sleep(0.5)

        # Disarm
        self._disarm()

        # Update state
        self._state_pub.publish(String(data='LANDED'))
        self._kill_active_pub.publish(Bool(data=False))
        self._descending = False
        self._descent_complete = True

        rospy.logwarn('[KILL] ===== DESCENT COMPLETE. DRONE DISARMED. =====')

    def _disarm(self):
        """Disarm the motors via MAVLink."""
        if self._arm_srv is None:
            rospy.logerr('[KILL] No arming service available. Cannot disarm.')
            return

        for attempt in range(DISARM_RETRY):
            try:
                req = CommandBoolRequest()
                req.value = False
                resp = self._arm_srv(req)
                if resp.success:
                    rospy.logwarn('[KILL] Disarm successful (attempt %d).', attempt + 1)
                    self._state_pub.publish(String(data='DISARMED'))
                    return
                else:
                    rospy.logwarn('[KILL] Disarm rejected (attempt %d). Retrying...', attempt + 1)
            except Exception as e:
                rospy.logerr('[KILL] Disarm error (attempt %d): %s', attempt + 1, str(e))
            time.sleep(0.3)

        # Force disarm via MAVROS service (safe - no serial conflict)
        rospy.logwarn("[KILL] Trying force-disarm via MAVROS...")
        try:
            for _ in range(3):
                req = CommandBoolRequest()
                req.value = False
                self._arm_srv(req)
                time.sleep(0.3)
            rospy.logwarn("[KILL] Force-disarm attempts complete.")
        except Exception as e:
            rospy.logerr("[KILL] Force-disarm failed: %s", str(e))

    def _send_rc(self, throttle_pwm):
        """Send RC override with given throttle."""
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


# ── Standalone entry point ────────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        controller = KillController(standalone=True)
        rospy.loginfo('[KILL] Running standalone. Waiting for /drone/kill_trigger...')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
