#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
tcp_bridge.py  —  Unified TCP Telemetry + Command Bridge
==========================================================
IRoC-U 2026 — GPS-Denied Autonomous Flight

Runs on Jetson Nano. Single TCP server that:
  1. Accepts connection from GCS laptop
  2. Streams telemetry at 10Hz (JSON):
     - roll, pitch, yaw
     - Vx, Vy, Vz
     - LiDAR altitude
     - battery voltage, current
     - armed state, flight mode
     - VIO status, emergency state
     - flight state (IDLE/TAKEOFF/HOVER/LANDING/etc)
  3. Receives commands: START, KILL, DISARM, PING
  4. Forwards commands to appropriate ROS topics
  5. DISARM: direct MAVLink to Pixhawk

Transport: TCP on port 5005 — no firewall issues over WiFi.

Author: IRoC-U ASCEND 2026 Team
"""

import rospy
import os
import sys
import json
import time
import math
import socket
import signal
import threading
import datetime

# ── Python 2 UTF-8 fix ───────────────────────────────────────────────────────
import codecs
if hasattr(sys.stdout, 'buffer'):
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, errors='replace')
elif hasattr(sys.stdout, 'fileno'):
    try:
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout, errors='replace')
    except Exception:
        pass

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg   import BatteryState, Imu
from mavros_msgs.msg   import State
from std_msgs.msg      import Float32, String, Bool

# pymavlink for direct DISARM
try:
    from pymavlink import mavutil
    MAV_AVAILABLE = True
except ImportError:
    MAV_AVAILABLE = False

# ─── CONFIG ───────────────────────────────────────────────────────────────────
LISTEN_HOST      = "0.0.0.0"
LISTEN_PORT      = 5005
TELEMETRY_HZ     = 10         # telemetry push rate
WATCHDOG_TIMEOUT = 15.0       # seconds without PING → failsafe
PIXHAWK_PORT     = "/dev/ttyACM0"
PIXHAWK_BAUD     = 115200
# ──────────────────────────────────────────────────────────────────────────────


def log(msg, level="INFO"):
    try:
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        line = u"[{}] [{}] {}".format(ts, level, msg)
        rospy.loginfo(line) if level == "INFO" else rospy.logwarn(line)
    except Exception:
        pass


class TCPBridge(object):

    def __init__(self):
        rospy.init_node('tcp_bridge_node', anonymous=False)

        # ── Telemetry state ───────────────────────────────────────────
        self._lock = threading.Lock()
        self._telem = {
            'position':  {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity':  {'vertical': 0.0, 'horizontal': 0.0,
                          'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'attitude':  {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'battery':   {'voltage': 0.0, 'current': 0.0, 'remaining_mah': 0},
            'sensors':   {'gps_fix': 0, 'satellites': 0,
                          'temperature': 0.0, 'pressure': 0.0},
            'status':    {'armed': False, 'mode': 'STABILIZE', 'connected': False},
            'lidar_alt': 0.0,
            'vio_status': 'SEARCHING',
            'flight_state': 'IDLE',
            'emergency': '',
            'timestamp': 0,
        }

        # ── TCP server state ──────────────────────────────────────────
        self._client_sock   = None
        self._server_sock   = None
        self._send_lock     = threading.Lock()
        self._last_cmd_time = time.time()
        self._shutdown      = threading.Event()
        self._mission_active = False

        # Kill/disarm flags
        self._action_lock        = threading.Lock()
        self._kill_in_progress   = False
        self._disarm_in_progress = False

        # ── ROS Subscribers ───────────────────────────────────────────
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/local_position/velocity_local',
                         TwistStamped, self._vel_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)
        rospy.Subscriber('/mavros/battery', BatteryState, self._battery_cb)
        rospy.Subscriber('/lidar/altitude', Float32, self._lidar_cb)
        rospy.Subscriber('/vio/status', String, self._vio_status_cb)
        rospy.Subscriber('/drone/flight_state', String, self._flight_state_cb)

        # ── ROS Publishers (forward commands) ─────────────────────────
        self._flight_cmd_pub = rospy.Publisher(
            '/gcs/flight_command', String, queue_size=5, latch=True)
        self._kill_flag_pub = rospy.Publisher(
            '/gcs/kill_flag', Bool, queue_size=5, latch=True)

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        log("TCP Bridge initialised. Port: {}".format(LISTEN_PORT))

    # ── ROS Callbacks ─────────────────────────────────────────────────

    def _state_cb(self, msg):
        with self._lock:
            self._telem['status']['armed'] = msg.armed
            self._telem['status']['mode'] = msg.mode
            self._telem['status']['connected'] = msg.connected

    def _pose_cb(self, msg):
        with self._lock:
            self._telem['position']['x'] = msg.pose.position.x
            self._telem['position']['y'] = msg.pose.position.y
            self._telem['position']['z'] = msg.pose.position.z

    def _vel_cb(self, msg):
        with self._lock:
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            vz = msg.twist.linear.z
            self._telem['velocity']['vx'] = vx
            self._telem['velocity']['vy'] = vy
            self._telem['velocity']['vz'] = vz
            self._telem['velocity']['horizontal'] = math.sqrt(vx**2 + vy**2)
            self._telem['velocity']['vertical'] = vz

    def _imu_cb(self, msg):
        with self._lock:
            # Convert quaternion to euler (simplified)
            q = msg.orientation
            # Roll
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr, cosr) * 180.0 / math.pi
            # Pitch
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            if abs(sinp) >= 1:
                pitch = math.copysign(90.0, sinp)
            else:
                pitch = math.asin(sinp) * 180.0 / math.pi
            # Yaw
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy) * 180.0 / math.pi
            if yaw < 0:
                yaw += 360.0

            self._telem['attitude']['roll'] = round(roll, 2)
            self._telem['attitude']['pitch'] = round(pitch, 2)
            self._telem['attitude']['yaw'] = round(yaw, 2)

    def _battery_cb(self, msg):
        with self._lock:
            self._telem['battery']['voltage'] = round(msg.voltage, 2)
            self._telem['battery']['current'] = round(msg.current, 2) if msg.current >= 0 else 0.0

    def _lidar_cb(self, msg):
        with self._lock:
            self._telem['lidar_alt'] = round(msg.data, 3)
            # Also update position.z with LiDAR (authoritative altitude)
            self._telem['position']['z'] = round(msg.data, 3)

    def _vio_status_cb(self, msg):
        with self._lock:
            self._telem['vio_status'] = msg.data

    def _flight_state_cb(self, msg):
        with self._lock:
            self._telem['flight_state'] = msg.data
        if msg.data in ('MISSION_COMPLETE', 'LANDED', 'DISARMED'):
            self._mission_active = False

    # ── TCP Server ────────────────────────────────────────────────────

    def start(self):
        try:
            self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_sock.bind((LISTEN_HOST, LISTEN_PORT))
            self._server_sock.listen(1)
            self._server_sock.settimeout(1.0)
            log("TCP server listening on port {}.".format(LISTEN_PORT))
        except Exception as e:
            log("Server bind failed: {}".format(e), "ERROR")
            sys.exit(1)

        # Start watchdog
        wdog = threading.Thread(target=self._watchdog_loop)
        wdog.daemon = True
        wdog.start()

        # Start telemetry streamer
        telem_thread = threading.Thread(target=self._telemetry_loop)
        telem_thread.daemon = True
        telem_thread.start()

        self._accept_loop()

    def _accept_loop(self):
        log("Waiting for GCS connection...")
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            try:
                conn, addr = self._server_sock.accept()
                self._client_sock = conn
                self._client_sock.settimeout(1.0)
                self._last_cmd_time = time.time()
                log("GCS connected from {}:{}.".format(addr[0], addr[1]))
                self._client_loop()
                old_sock, self._client_sock = self._client_sock, None
                try:
                    old_sock.close()
                except Exception:
                    pass
                log("GCS disconnected.")
                self._trigger_failsafe_on_disconnect()
            except socket.timeout:
                continue
            except Exception as e:
                if not self._shutdown.is_set():
                    log("Accept error: {}".format(e), "ERROR")
        self._cleanup()

    def _client_loop(self):
        buf = u""
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            try:
                data = self._client_sock.recv(4096)
                if not data:
                    break
                buf += data.decode("utf-8", errors="ignore")
                while u"\n" in buf:
                    line, buf = buf.split(u"\n", 1)
                    line = line.strip().upper()
                    if not line:
                        continue
                    self._last_cmd_time = time.time()
                    self._dispatch(line)
            except socket.timeout:
                continue
            except Exception as e:
                log("Client recv error: {}".format(e), "ERROR")
                break

    # ── Command Dispatch ──────────────────────────────────────────────

    def _dispatch(self, cmd):
        log("Received: '{}'".format(cmd))

        if cmd == "PING":
            self._send("PONG")
            return

        if cmd == "START":
            t = threading.Thread(target=self._handle_start)
            t.daemon = True
            t.start()
        elif cmd == "KILL":
            t = threading.Thread(target=self._handle_kill)
            t.daemon = True
            t.start()
        elif cmd == "DISARM":
            t = threading.Thread(target=self._handle_disarm)
            t.daemon = True
            t.start()
        else:
            log("Unknown command: '{}'".format(cmd), "WARN")
            self._send("CONN_FAIL")

    def _handle_start(self):
        if self._mission_active:
            log("Mission already active.", "WARN")
            self._send("LOG:Mission already running.")
            return

        log("START validated.")
        self._mission_active = True

        # Publish START to ROS so throttle_hover.py picks it up
        self._flight_cmd_pub.publish(String(data="START"))
        self._send("ACK_START")
        self._send("STATE:TAKEOFF")
        log("ACK_START sent. Mission started.")

    def _handle_kill(self):
        with self._action_lock:
            if self._kill_in_progress or self._disarm_in_progress:
                self._send("ACK_KILL")
                return
            self._kill_in_progress = True

        # ACK immediately
        self._send("ACK_KILL")
        log("KILL received. Publishing kill flag.", "WARN")

        self._mission_active = False

        # Publish KILL to ROS
        self._kill_flag_pub.publish(Bool(data=True))
        self._flight_cmd_pub.publish(String(data="KILL"))

        self._send("STATE:DESCENDING")
        self._send("LOG:KILL received. Initiating controlled descent via LiDAR.")

        # Monitor until flight state becomes LANDED or DISARMED
        deadline = time.time() + 120.0  # 2 min max
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                fstate = self._telem.get('flight_state', '')
            if fstate in ('LANDED', 'DISARMED', 'MISSION_COMPLETE'):
                self._send("STATE:LANDED")
                self._send("LOG:Descent complete. Drone landed.")
                log("Kill descent complete.")
                break
            time.sleep(1.0)
            remaining = int(deadline - time.time())
            if remaining % 10 == 0:
                self._send("LOG:Descent in progress... {}s until timeout.".format(remaining))
        else:
            self._send("STATE:KILL / FAILSAFE")
            self._send("LOG:Kill timeout. Check drone manually.")
            log("Kill timeout!", "WARN")

        with self._action_lock:
            self._kill_in_progress = False

    def _handle_disarm(self):
        """Direct MAVLink disarm — LOGIC INTENTIONALLY MATCHES comm4.py"""
        with self._action_lock:
            if self._disarm_in_progress:
                self._send("ACK_DISARM")
                return
            self._disarm_in_progress = True
            self._kill_in_progress = True

        log("DISARM received. Hardware emergency stop.", "WARN")
        self._mission_active = False

        # Publish kill flag
        self._kill_flag_pub.publish(Bool(data=True))
        # Disarm via MAVROS service (safe - no serial conflict)
        disarm_success = False
        try:
            from mavros_msgs.srv import CommandBool, CommandBoolRequest
            import rospy
            arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            req = CommandBoolRequest()
            req.value = False
            resp = arm_srv(req)
            disarm_success = resp.success
            log("MAVROS disarm result: {}".format(disarm_success))
        except Exception as e:
            log("MAVROS disarm error: {}".format(str(e)), "ERROR")

        if disarm_success:
            self._send("LOG:Hardware DISARM sent to Pixhawk successfully.")
            log("Hardware DISARM complete.")
        else:
            self._send("LOG:WARNING - MAVLink DISARM may have failed.")
            log("MAVLink disarm failed.", "ERROR")

        with self._action_lock:
            self._kill_in_progress = False
            self._disarm_in_progress = False

    def _mavlink_disarm(self):
        """Direct serial MAVLink disarm — same logic as comm4.py"""
        try:
            log("Opening MAVLink on {}...".format(PIXHAWK_PORT))
            mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
            hb = mav.wait_heartbeat(timeout=5)
            if hb is None:
                log("No Pixhawk heartbeat.", "ERROR")
                mav.close()
                return False

            log("Heartbeat OK. Sending DISARM.")

            def send_disarm(param2=0):
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, param2, 0, 0, 0, 0, 0
                )

            for _ in range(3):
                send_disarm(param2=0)
                time.sleep(0.05)

            ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
            if ack and ack.result == 0:
                log("Disarm accepted.")
                mav.close()
                return True

            # Force disarm
            log("Normal disarm failed. Trying force-disarm.", "WARN")
            for _ in range(3):
                send_disarm(param2=21196)
                time.sleep(0.05)

            ack2 = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
            mav.close()

            if ack2 and ack2.result == 0:
                return True
            return ack2 is not None  # assume success if any ACK came

        except Exception as e:
            log("MAVLink disarm error: {}".format(e), "ERROR")
            return False

    # ── Telemetry Streamer ────────────────────────────────────────────

    def _telemetry_loop(self):
        period = 1.0 / TELEMETRY_HZ
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            if self._client_sock is not None:
                with self._lock:
                    telem = dict(self._telem)
                    telem['timestamp'] = int(time.time() * 1000)
                    telem['type'] = 'TELEMETRY'
                try:
                    payload = json.dumps(telem) + "\n"
                    with self._send_lock:
                        if self._client_sock:
                            self._client_sock.sendall(payload.encode('utf-8'))
                except Exception:
                    pass  # client disconnected
            time.sleep(period)

    # ── Watchdog ──────────────────────────────────────────────────────

    def _watchdog_loop(self):
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            time.sleep(1.0)
            if self._client_sock is None:
                continue
            if not self._mission_active:
                continue
            with self._action_lock:
                busy = self._kill_in_progress or self._disarm_in_progress
            if busy:
                continue
            elapsed = time.time() - self._last_cmd_time
            if elapsed > WATCHDOG_TIMEOUT:
                log("Watchdog: no packet for {:.1f}s. Auto-kill.".format(elapsed), "WARN")
                self._send("LOG:Watchdog triggered. Auto-kill initiated.")
                t = threading.Thread(target=self._handle_kill)
                t.daemon = True
                t.start()
                self._last_cmd_time = time.time()

    def _trigger_failsafe_on_disconnect(self):
        if not self._mission_active:
            return
        with self._action_lock:
            busy = self._kill_in_progress or self._disarm_in_progress
        if busy:
            return
        log("Disconnect during mission. Triggering kill.", "WARN")
        t = threading.Thread(target=self._handle_kill)
        t.daemon = True
        t.start()

    # ── Helpers ───────────────────────────────────────────────────────

    def _send(self, message):
        sock = self._client_sock
        if sock is None:
            return
        try:
            payload = (message + u"\n").encode("utf-8", errors="replace")
            with self._send_lock:
                sock.sendall(payload)
        except Exception as e:
            log("Send failed: {}".format(e), "ERROR")

    def _signal_handler(self, signum, frame):
        log("Signal {}. Shutting down.".format(signum), "WARN")
        self._shutdown.set()

    def _cleanup(self):
        if self._client_sock:
            try:
                self._client_sock.close()
            except Exception:
                pass
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
        log("Cleanup complete.")


# ── MAIN ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    bridge = TCPBridge()
    bridge.start()
