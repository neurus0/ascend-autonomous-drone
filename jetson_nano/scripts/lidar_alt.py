#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
lidar_alt.py
============
LiDAR Altitude Node — IRoC-U 2026 Flight Stack
================================================
Based on: lidar.py (tested, working TFMini-S reader)
Runs with: offboard_hover.py + vio_nav.py (parallel)

Hardware : Jetson Nano + TFMini-S LiDAR on /dev/ttyUSB0
Stack    : ROS 1 Melodic / Python 2.7

This node does everything lidar.py does PLUS:
  - Wraps the serial reader in a proper ROS node
  - Publishes /lidar/altitude  (Float32, metres)
  - Publishes /lidar/strength  (Float32, signal quality)
  - Publishes /lidar/status    (String) for health monitoring
  - Publishes /mavros/distance_sensor/rangefinder_sub
    (sensor_msgs/Range) for potential PX4 EKF altitude fusion
  - Applies a simple moving-average filter for noise reduction
  - Handles serial errors gracefully with auto-reconnect

Topics Published:
    /lidar/altitude       -- Float32, altitude in metres
    /lidar/strength       -- Float32, signal strength
    /lidar/status         -- String: "OK" | "WEAK" | "DISCONNECTED"
    /lidar/raw_cm         -- Float32, raw distance in cm (debug)
    /mavros/distance_sensor/rangefinder_sub -- Range msg for EKF

Run:
    python2 lidar_alt.py
"""

import rospy
import serial
import time
import threading
import sys
import collections

from std_msgs.msg    import Float32, String
from sensor_msgs.msg import Range


# =============================================================================
# CONFIG
# =============================================================================

SERIAL_PORT       = '/dev/ttyUSB0'
SERIAL_BAUD       = 115200
SERIAL_TIMEOUT    = 1.0          # seconds

# TFMini-S specs
MIN_RANGE_M       = 0.10         # 10 cm minimum
MAX_RANGE_M       = 12.0         # 12 m maximum
FIELD_OF_VIEW     = 0.04         # ~2.3° FOV in radians

# Signal quality threshold
MIN_STRENGTH      = 100          # readings below this are unreliable

# Moving-average filter window
FILTER_WINDOW     = 5            # last N valid readings

# Publishing rate
PUBLISH_RATE_HZ   = 20           # match offboard_hover setpoint rate

# Reconnect
RECONNECT_DELAY_S = 2.0          # wait before retry on serial error


# =============================================================================
# TFMini-S READER (from proven lidar.py, enhanced)
# =============================================================================

class TFMiniReader(object):
    """
    Thread-safe serial reader for TFMini-S LiDAR.
    Reads 9-byte frames, validates checksum, applies filter.
    """

    def __init__(self):
        self._lock       = threading.Lock()
        self._ser        = None
        self._connected  = False
        self._distance_m = 0.0
        self._strength   = 0
        self._valid      = False
        self._filter_buf = collections.deque(maxlen=FILTER_WINDOW)
        self._filtered_m = 0.0
        self._frame_count = 0
        self._error_count = 0

    def connect(self):
        """Open serial port to TFMini-S."""
        try:
            self._ser = serial.Serial(
                SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            time.sleep(0.5)
            # flush stale data
            self._ser.reset_input_buffer()
            self._connected = True
            rospy.loginfo('[LIDAR] Connected to %s at %d baud',
                          SERIAL_PORT, SERIAL_BAUD)
            return True
        except serial.SerialException as e:
            rospy.logerr('[LIDAR] Cannot open %s: %s', SERIAL_PORT, str(e))
            self._connected = False
            return False

    def disconnect(self):
        """Close serial port."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._connected = False

    def is_connected(self):
        return self._connected

    def read_one_frame(self):
        """
        Read one TFMini-S frame (9 bytes: 0x59 0x59 + 7 data bytes).
        Returns True if a valid frame was read, False otherwise.

        Frame format:
          Byte 0-1: 0x59 0x59 (header)
          Byte 2:   distance low byte
          Byte 3:   distance high byte
          Byte 4:   strength low byte
          Byte 5:   strength high byte
          Byte 6:   reserved
          Byte 7:   raw signal quality
          Byte 8:   checksum (sum of bytes 0-7, mod 256)
        """
        if not self._connected or self._ser is None:
            return False

        try:
            # Wait for the 2-byte frame header 0x59 0x59
            b1 = self._ser.read(1)
            if b1 != b'\x59':
                return False
            b2 = self._ser.read(1)
            if b2 != b'\x59':
                return False

            # Read remaining 7 bytes
            frame = self._ser.read(7)
            if len(frame) < 7:
                return False

            dist_low  = frame[0] if isinstance(frame[0], int) else ord(frame[0])
            dist_high = frame[1] if isinstance(frame[1], int) else ord(frame[1])
            str_low   = frame[2] if isinstance(frame[2], int) else ord(frame[2])
            str_high  = frame[3] if isinstance(frame[3], int) else ord(frame[3])
            checksum  = frame[6] if isinstance(frame[6], int) else ord(frame[6])

            distance_cm = dist_low + dist_high * 256
            strength    = str_low  + str_high  * 256

            # Validate checksum (sum of all 9 bytes mod 256 = 0,
            # or equivalently sum of first 8 = byte 9)
            raw_bytes = bytearray([0x59, 0x59])
            for b in frame:
                raw_bytes.append(b if isinstance(b, int) else ord(b))
            calc_checksum = sum(raw_bytes[:8]) & 0xFF

            if calc_checksum != checksum:
                self._error_count += 1
                return False

            self._frame_count += 1

            with self._lock:
                self._strength = strength

                if strength > MIN_STRENGTH and 0 < distance_cm < (MAX_RANGE_M * 100):
                    self._distance_m = distance_cm / 100.0
                    self._valid      = True
                    # Update moving-average filter
                    self._filter_buf.append(self._distance_m)
                    self._filtered_m = sum(self._filter_buf) / len(self._filter_buf)
                else:
                    self._valid = False

            return True

        except serial.SerialException as e:
            rospy.logerr('[LIDAR] Serial error: %s', str(e))
            self._connected = False
            return False
        except Exception as e:
            rospy.logwarn_throttle(5, '[LIDAR] Read error: %s', str(e))
            return False

    def get_data(self):
        """
        Return (altitude_m, filtered_m, strength, valid).
        Thread-safe.
        """
        with self._lock:
            return (self._distance_m, self._filtered_m,
                    self._strength, self._valid)

    def get_stats(self):
        """Return (frame_count, error_count)."""
        return self._frame_count, self._error_count


# =============================================================================
# ROS NODE
# =============================================================================

class LidarAltNode(object):
    """
    ROS node that reads TFMini-S LiDAR and publishes altitude data
    for use by offboard_hover.py and vio_nav.py.
    """

    def __init__(self):
        self._reader = TFMiniReader()

        # ── Publishers ───────────────────────────────────────────────
        self._alt_pub      = rospy.Publisher('/lidar/altitude',
                                             Float32, queue_size=10)
        self._strength_pub = rospy.Publisher('/lidar/strength',
                                             Float32, queue_size=10)
        self._status_pub   = rospy.Publisher('/lidar/status',
                                             String,  queue_size=5,
                                             latch=True)
        self._raw_pub      = rospy.Publisher('/lidar/raw_cm',
                                             Float32, queue_size=10)

        # Range message for MAVROS distance sensor (optional EKF fusion)
        self._range_pub    = rospy.Publisher(
            '/mavros/distance_sensor/rangefinder_sub',
            Range, queue_size=10)

        # ── Connect to serial ────────────────────────────────────────
        self._connect_with_retry()

        # ── Start reader thread ──────────────────────────────────────
        self._reader_thread = threading.Thread(target=self._reader_loop)
        self._reader_thread.daemon = True
        self._reader_thread.start()

        # ── Publish timer ────────────────────────────────────────────
        period = 1.0 / PUBLISH_RATE_HZ
        rospy.Timer(rospy.Duration(period), self._publish_loop)

        # ── Status timer (1 Hz) ──────────────────────────────────────
        rospy.Timer(rospy.Duration(1.0), self._status_loop)

        rospy.loginfo('[LIDAR] Node ready — publishing at %d Hz', PUBLISH_RATE_HZ)

    def _connect_with_retry(self):
        """Keep trying to connect until success or shutdown."""
        while not rospy.is_shutdown():
            if self._reader.connect():
                return
            rospy.logwarn('[LIDAR] Retrying in %.1f s ...', RECONNECT_DELAY_S)
            time.sleep(RECONNECT_DELAY_S)

    # ── Reader thread (runs as fast as serial data arrives) ──────────

    def _reader_loop(self):
        """Continuously read TFMini-S frames in background thread."""
        while not rospy.is_shutdown():
            if not self._reader.is_connected():
                rospy.logwarn_throttle(5, '[LIDAR] Disconnected — reconnecting ...')
                time.sleep(RECONNECT_DELAY_S)
                self._reader.connect()
                continue

            self._reader.read_one_frame()

    # ── Publish loop ─────────────────────────────────────────────────

    def _publish_loop(self, _event):
        """Publish LiDAR data at fixed rate."""
        raw_m, filtered_m, strength, valid = self._reader.get_data()

        # Always publish filtered altitude (other nodes depend on it)
        alt_msg      = Float32()
        alt_msg.data = filtered_m if valid else 0.0
        self._alt_pub.publish(alt_msg)

        # Strength
        str_msg      = Float32()
        str_msg.data = float(strength)
        self._strength_pub.publish(str_msg)

        # Raw distance in cm (debug)
        raw_msg      = Float32()
        raw_msg.data = raw_m * 100.0
        self._raw_pub.publish(raw_msg)

        # MAVROS Range message for EKF fusion
        if valid:
            rmsg = Range()
            rmsg.header.stamp    = rospy.Time.now()
            rmsg.header.frame_id = "lidar_link"
            rmsg.radiation_type  = Range.INFRARED
            rmsg.field_of_view   = FIELD_OF_VIEW
            rmsg.min_range       = MIN_RANGE_M
            rmsg.max_range       = MAX_RANGE_M
            rmsg.range           = filtered_m
            self._range_pub.publish(rmsg)

        # Throttled console log
        if valid:
            rospy.loginfo_throttle(2,
                '[LIDAR] ALT: %.2f m (filtered: %.2f m) | STR: %d',
                raw_m, filtered_m, strength)
        else:
            rospy.logwarn_throttle(2,
                '[LIDAR] Weak/invalid reading — STR: %d', strength)

    # ── Status loop ──────────────────────────────────────────────────

    def _status_loop(self, _event):
        """Publish health status at 1 Hz."""
        smsg = String()
        if not self._reader.is_connected():
            smsg.data = "DISCONNECTED"
        else:
            _, _, strength, valid = self._reader.get_data()
            if valid:
                smsg.data = "OK"
            else:
                smsg.data = "WEAK"
        self._status_pub.publish(smsg)

        frames, errors = self._reader.get_stats()
        rospy.loginfo_throttle(10,
            '[LIDAR] Stats: %d frames, %d errors', frames, errors)

    # ── Shutdown ─────────────────────────────────────────────────────

    def shutdown(self):
        self._reader.disconnect()
        rospy.loginfo('[LIDAR] Shutdown complete')


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == '__main__':
    rospy.init_node('lidar_alt_node')
    rospy.loginfo('=' * 60)
    rospy.loginfo('lidar_alt.py — LiDAR Altitude Node (IRoC-U 2026)')
    rospy.loginfo('Serial     : %s @ %d baud', SERIAL_PORT, SERIAL_BAUD)
    rospy.loginfo('Publishes  : /lidar/altitude  /lidar/strength  /lidar/status')
    rospy.loginfo('             /mavros/distance_sensor/rangefinder_sub')
    rospy.loginfo('=' * 60)
    node = LidarAltNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
