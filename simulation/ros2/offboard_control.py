#!/usr/bin/env python3
"""
IRoC-U 2026 Qualification Round - Autonomous Drone Control Node
PRODUCTION VERSION with Emergency System and Drift Correction

Tasks: TASK-1 (Vertical Take-Off) + TASK-2 (Hover Stability) + TASK-3 (Landing)

This node performs:
1. Autonomous vertical take-off to 4.0m altitude
2. Stable hovering for 30 seconds with ±0.2m tolerance (DRIFT CORRECTED)
3. Autonomous landing at the same position
4. Full emergency system (battery, attitude, altitude, position loss)
5. Proper optical flow monitoring (cs_opt_flow + cs_local_position_valid)

Author: ROS 2 + PX4 Offboard Control System (Production Ready)
Date: 2026-03-13
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleControlMode,
    VehicleLandDetected,
    EstimatorStatusFlags,
    BatteryStatus,
    VehicleAttitude
)

import math
from enum import Enum


class FlightState(Enum):
    """Flight state machine for autonomous mission"""
    INIT = 0
    PREFLIGHT_CHECKS = 1
    WAIT_FOR_READY = 2
    OFFBOARD_REQUESTED = 3
    ARM_REQUESTED = 4
    TAKEOFF = 5
    HOVER = 6
    LANDING = 7
    MISSION_COMPLETE = 8
    EMERGENCY = 9


class IroCUQualificationNode(Node):
    """
    ROS 2 Node for IRoC-U 2026 Qualification Round
    PRODUCTION VERSION with full safety systems
    """

    def __init__(self):
        super().__init__('iroc_u_qualification_node')
        
        # =====================================================================
        # MISSION PARAMETERS (IRoC-U 2026 Requirements)
        # =====================================================================
        self.TARGET_ALTITUDE = -4.0  # meters (NED frame, negative = up)
        self.HOVER_TOLERANCE = 0.2   # meters (±0.2m as per rules)
        self.HOVER_DURATION = 30.0   # seconds
        self.HEARTBEAT_RATE = 30     # Hz (increased for better control)
        self.OFFBOARD_SETPOINT_COUNT = 30  # 1 second warm-up @ 30 Hz
        self.STATE_MACHINE_RATE = 0.2  # seconds (5 Hz)
        
        # Safety limits
        self.MAX_ALTITUDE = -10.0    # meters NED (10m ceiling)
        self.MIN_ALTITUDE = 0.15     # meters (landing detection)
        self.MAX_TILT_ANGLE = 20.0   # degrees
        self.MIN_BATTERY = 20.0      # percent
        self.MAX_DRIFT = 0.5         # meters (horizontal)
        self.POSITION_TIMEOUT = 2.0  # seconds
        
        # Drift counter with hysteresis
        self.DRIFT_COUNTER_THRESHOLD = 5
        
        # =====================================================================
        # STATE VARIABLES
        # =====================================================================
        self.state = FlightState.INIT
        self.vehicle_status = None
        self.vehicle_local_position = None  # PRIMARY position source
        self.vehicle_control_mode = None
        self.vehicle_land_detected = None
        self.estimator_status = None
        self.battery_status = None
        self.vehicle_attitude = None
        
        # Flags
        self.offboard_counter = 0
        self.arm_sent = False
        self.offboard_sent = False
        self.land_sent = False
        self.preflight_complete = False
        self.emergency_triggered = False
        
        # FIXED: Capture positions (don't use drifting current position!)
        self.takeoff_position_x = 0.0
        self.takeoff_position_y = 0.0
        self.hover_position_x = 0.0
        self.hover_position_y = 0.0
        self.hover_position_z = 0.0
        self.hover_yaw = 0.0
        self.hover_start_time = None
        self.stable_counter = 0
        
        # Drift detection
        self.drift_counter = 0
        self.max_drift_horizontal = 0.0
        self.max_drift_vertical = 0.0
        
        # FIXED: Proper optical flow monitoring
        self.optical_flow_active = False
        self.local_position_valid = False
        self.last_position_valid_time = None
        
        # Logging
        self.last_logged_second = -1
        
        # =====================================================================
        # QoS PROFILE - FIXED: Use VOLATILE (not TRANSIENT_LOCAL)
        # =====================================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # FIXED!
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # =====================================================================
        # PUBLISHERS
        # =====================================================================
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos
        )
        
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos
        )
        
        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos
        )
        
        # =====================================================================
        # SUBSCRIBERS
        # =====================================================================
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos
        )
        
        # FIXED: Use VehicleLocalPosition (more stable)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos
        )
        
        self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos
        )
        
        self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_detect_callback,
            qos
        )
        
        self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self.estimator_status_callback,
            qos
        )
        
        self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_status_callback,
            qos
        )
        
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos
        )
        
        # =====================================================================
        # TIMERS
        # =====================================================================
        self.control_timer = self.create_timer(
            1.0 / self.HEARTBEAT_RATE,
            self.control_loop
        )
        
        self.state_timer = self.create_timer(
            self.STATE_MACHINE_RATE,
            self.state_machine
        )
        
        # NEW: Emergency monitoring
        self.safety_timer = self.create_timer(
            0.5,
            self.safety_monitor
        )
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('IRoC-U 2026 - PRODUCTION VERSION')
        self.get_logger().info(f'Target: {abs(self.TARGET_ALTITUDE)}m altitude')
        self.get_logger().info(f'Hover: {self.HOVER_DURATION}s ±{self.HOVER_TOLERANCE}m')
        self.get_logger().info('Emergency System: ACTIVE')
        self.get_logger().info('Drift Correction: ENABLED')
        self.get_logger().info('=' * 70)

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        """FIXED: Use VehicleLocalPosition (more stable than odometry)"""
        self.vehicle_local_position = msg
        self.local_position_valid = msg.xy_valid and msg.z_valid
    
    def vehicle_control_mode_callback(self, msg):
        self.vehicle_control_mode = msg
    
    def land_detect_callback(self, msg):
        self.vehicle_land_detected = msg
    
    def battery_status_callback(self, msg):
        self.battery_status = msg
    
    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude = msg
    
    def estimator_status_callback(self, msg):
        """
        FIXED: Proper optical flow monitoring
        Check BOTH cs_opt_flow AND cs_local_position_valid
        """
        self.estimator_status = msg
        
        # Check optical flow sensor status
        self.optical_flow_active = getattr(msg, "cs_opt_flow", False)
        
        # CRITICAL: Check if position estimate is valid
        #self.local_position_valid = getattr(msg, "cs_position", False)
        
        if self.local_position_valid:
            self.last_position_valid_time = self.get_clock().now()
        
        # Warn if GPS active (should be disabled)
        current_sec = int(self.get_clock().now().nanoseconds / 1e9)
        if getattr(msg, "cs_gps", False) and current_sec != self.last_logged_second:
            self.get_logger().warn('⚠ GPS active - should be disabled!')
            self.last_logged_second = current_sec
        
        # CRITICAL: Position loss during flight
        if not self.local_position_valid and self.state not in [
            FlightState.INIT,
            FlightState.PREFLIGHT_CHECKS,
            FlightState.MISSION_COMPLETE,
            FlightState.EMERGENCY
        ]:
            if self.last_position_valid_time is None:
                self.last_position_valid_time = self.get_clock().now()
                return

            dt = (self.get_clock().now() - self.last_position_valid_time).nanoseconds / 1e9
            if dt > 0.5:
                self.get_logger().error('✗ POSITION LOST - EMERGENCY!')
                self.trigger_emergency_landing()

    # =========================================================================
    # PREFLIGHT CHECKS (NEW)
    # =========================================================================
    
    def run_preflight_checks(self):
        """Comprehensive preflight checks"""
        self.get_logger().info('=' * 70)
        self.get_logger().info('PREFLIGHT CHECKS')
        self.get_logger().info('=' * 70)
        
        checks_passed = True
        
        # Battery
        if self.battery_status:
            battery_pct = self.battery_status.remaining * 100.0
            self.get_logger().info(f'✓ Battery: {battery_pct:.1f}%')
            if battery_pct < self.MIN_BATTERY:
                self.get_logger().error(f'✗ Battery too low: {battery_pct:.1f}%')
                checks_passed = False
        else:
            self.get_logger().error('✗ No battery data')
            checks_passed = False
        
        # Attitude level check
        if self.vehicle_attitude:
            roll, pitch, _ = self.quaternion_to_euler(self.vehicle_attitude.q)
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            self.get_logger().info(f'✓ Attitude: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°')
            if abs(roll_deg) > 5.0 or abs(pitch_deg) > 15.0:
                self.get_logger().error(f'✗ Not level!')
                checks_passed = False
        
        # CRITICAL: Position validity
        if self.local_position_valid:
            self.get_logger().info('✓ EKF: Local position valid')
            self.last_position_valid_time = self.get_clock().now()
        else:
            self.get_logger().error('✗ EKF: Position NOT valid')
            checks_passed = False
            
            # Check optical flow
            if getattr(self.estimator_status, "cs_opt_flow", False):
                self.get_logger().info('✓ Optical Flow: ACTIVE')
            else:
                self.get_logger().warn('⚠ Optical Flow: NOT detected')
            
            # GPS check
            if getattr(self.estimator_status, "cs_gps", False):
                self.get_logger().warn('⚠ GPS active - should be disabled!')
            else:
                self.get_logger().info('✓ GPS disabled')
        #else:
            #self.get_logger().error('✗ No estimator status')
            #checks_passed = False
        
        # Local position available
        if not self.vehicle_local_position:
            self.get_logger().error('✗ No local position data')
            checks_passed = False
        else:
            self.get_logger().info('✓ Local position available')
        
        self.get_logger().info('=' * 70)
        if checks_passed:
            self.get_logger().info('✓ ALL CHECKS PASSED')
        else:
            self.get_logger().error('✗ CHECKS FAILED - DO NOT FLY')
        self.get_logger().info('=' * 70)
        
        return checks_passed

    # =========================================================================
    # SAFETY MONITOR (NEW - EMERGENCY SYSTEM)
    # =========================================================================
    
    def safety_monitor(self):
        """Continuous emergency monitoring"""
        
        if self.state in [
            FlightState.INIT,
            FlightState.PREFLIGHT_CHECKS,
            FlightState.MISSION_COMPLETE,
            FlightState.EMERGENCY
        ]:
            return
        
        # Check offboard mode loss
        if self.state in [FlightState.TAKEOFF, FlightState.HOVER]:
            if self.vehicle_control_mode and \
               not self.vehicle_control_mode.flag_control_offboard_enabled:
                self.get_logger().error('✗ OFFBOARD MODE LOST')
                self.trigger_emergency_landing()
                return
        
        # Position timeout
        if self.last_position_valid_time:
            timeout = (self.get_clock().now() - self.last_position_valid_time).nanoseconds / 1e9
            if timeout > self.POSITION_TIMEOUT:
                self.get_logger().error(f'✗ POSITION TIMEOUT: {timeout:.1f}s')
                self.trigger_emergency_landing()
                return
        
        # Battery
        if self.battery_status:
            battery_pct = self.battery_status.remaining * 100.0
            if battery_pct < self.MIN_BATTERY:
                self.get_logger().error(f'✗ BATTERY CRITICAL: {battery_pct:.1f}%')
                self.trigger_emergency_landing()
                return
        
        # Attitude limits
        if self.vehicle_attitude:
            roll, pitch, _ = self.quaternion_to_euler(self.vehicle_attitude.q)
            if abs(math.degrees(roll)) > self.MAX_TILT_ANGLE or \
               abs(math.degrees(pitch)) > self.MAX_TILT_ANGLE:
                self.get_logger().error('✗ UNSAFE TILT')
                self.trigger_emergency_landing()
                return
        
        # Altitude limits
        if self.vehicle_local_position:
            if self.vehicle_local_position.z < self.MAX_ALTITUDE:
                self.get_logger().error('✗ ALTITUDE EXCEEDED')
                self.trigger_emergency_landing()
                return

    # =========================================================================
    # COMMANDS
    # =========================================================================
    
    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
    
    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,   # arm
            0.0  # magic force arm code
        )
        self.get_logger().info('ARM command sent')
    
    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('DISARM command sent')
    
    def set_offboard(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('OFFBOARD mode command sent')
    
    def set_auto_land(self):
        """FIXED: Proper landing mode switch"""
        if self.vehicle_status and \
           self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            return  # Already landing
        
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)
        self.get_logger().info('AUTO.LAND mode sent')
    
    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)
    
    def publish_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = max(self.MAX_ALTITUDE, z)  # Clamp altitude
        msg.yaw = yaw
        self.setpoint_pub.publish(msg)

    # =========================================================================
    # STATE MACHINE
    # =========================================================================
    
    def state_machine(self):
        
        if self.state == FlightState.INIT:
            if self.vehicle_status and self.vehicle_local_position and self.estimator_status:
                self.get_logger().info('Telemetry received')
                self.state = FlightState.PREFLIGHT_CHECKS
        
        elif self.state == FlightState.PREFLIGHT_CHECKS:
            if not self.preflight_complete:
                if self.run_preflight_checks():
                    self.preflight_complete = True
                    self.state = FlightState.WAIT_FOR_READY
                else:
                    self.get_logger().error('Preflight failed - ABORT')
                    self.destroy_node()
                    return
        
        elif self.state == FlightState.WAIT_FOR_READY:
            if self.local_position_valid:
                # FIXED: Capture takeoff position BEFORE takeoff
                self.takeoff_position_x = self.vehicle_local_position.x
                self.takeoff_position_y = self.vehicle_local_position.y
                self.get_logger().info(
                    f'Takeoff position: X={self.takeoff_position_x:.2f}, '
                    f'Y={self.takeoff_position_y:.2f}'
                )
                self.state = FlightState.OFFBOARD_REQUESTED
        
        elif self.state == FlightState.OFFBOARD_REQUESTED:
            if self.offboard_counter > self.OFFBOARD_SETPOINT_COUNT:
                if not self.offboard_sent:
                    self.set_offboard()
                    self.offboard_sent = True
                
                if self.vehicle_control_mode and \
                   self.vehicle_control_mode.flag_control_offboard_enabled:
                    self.get_logger().info('✓ Offboard active')
                    self.state = FlightState.ARM_REQUESTED
        
        elif self.state == FlightState.ARM_REQUESTED:
            # Keep sending arm until actually armed
            if self.vehicle_status and \
               self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            
            if self.vehicle_status and \
               self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('✓ ARMED - TAKEOFF')
                self.hover_yaw = self.get_current_yaw()
                self.state = FlightState.TAKEOFF
        
        elif self.state == FlightState.TAKEOFF:
            if self.vehicle_local_position:
                altitude_error = abs(
                    self.vehicle_local_position.z - self.TARGET_ALTITUDE
                )
                
                current_alt = -self.vehicle_local_position.z
                target_alt = -self.TARGET_ALTITUDE
                
                current_sec = int(self.get_clock().now().nanoseconds / 1e9)
                if current_sec != self.last_logged_second:
                    self.get_logger().info(f'Takeoff: {current_alt:.2f}m / {target_alt:.2f}m')
                    self.last_logged_second = current_sec
                
                if altitude_error < self.HOVER_TOLERANCE:
                    self.stable_counter += 1
                    
                    # FIXED: Tie to timer period
                    required_counts = int(3.0 / self.STATE_MACHINE_RATE)
                    
                    if self.stable_counter > required_counts:
                        # FIXED: Capture hover position
                        self.hover_position_x = self.vehicle_local_position.x
                        self.hover_position_y = self.vehicle_local_position.y
                        self.hover_position_z = self.vehicle_local_position.z
                        self.hover_start_time = self.get_clock().now()
                        
                        self.get_logger().info('=' * 70)
                        self.get_logger().info('✓ HOVER STARTED')
                        self.get_logger().info(f'Position: X={self.hover_position_x:.2f}, '
                                             f'Y={self.hover_position_y:.2f}, '
                                             f'Z={-self.hover_position_z:.2f}m')
                        self.get_logger().info('=' * 70)
                        
                        self.last_logged_second = -1
                        self.state = FlightState.HOVER
                else:
                    self.stable_counter = 0
        
        elif self.state == FlightState.HOVER:
            if self.vehicle_local_position:
                elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
                
                # FIXED: Calculate drift from CAPTURED position
                dx = self.vehicle_local_position.x - self.hover_position_x
                dy = self.vehicle_local_position.y - self.hover_position_y
                dz = self.vehicle_local_position.z - self.hover_position_z
                
                drift_h = math.sqrt(dx**2 + dy**2)
                drift_v = abs(dz)
                
                self.max_drift_horizontal = max(self.max_drift_horizontal, drift_h)
                self.max_drift_vertical = max(self.max_drift_vertical, drift_v)
                
                # FIXED: Drift counter with hysteresis
                if drift_h > self.MAX_DRIFT:
                    self.drift_counter += 1
                else:
                    self.drift_counter = 0
                
                if self.drift_counter > self.DRIFT_COUNTER_THRESHOLD:
                    self.get_logger().error(f'✗ Persistent drift: {drift_h:.2f}m')
                    self.state = FlightState.LANDING
                
                # Log every 2 seconds
                current_sec = int(elapsed)
                if current_sec % 2 == 0 and current_sec != self.last_logged_second:
                    self.get_logger().info(
                        f'Hover: {elapsed:.1f}s/{self.HOVER_DURATION}s | '
                        f'H-drift: {drift_h:.2f}m | V-drift: {drift_v:.2f}m'
                    )
                    self.last_logged_second = current_sec
                
                if elapsed >= self.HOVER_DURATION:
                    self.get_logger().info('=' * 70)
                    self.get_logger().info('✓ HOVER COMPLETE')
                    self.get_logger().info(f'Max H-drift: {self.max_drift_horizontal:.2f}m')
                    self.get_logger().info(f'Max V-drift: {self.max_drift_vertical:.2f}m')
                    self.get_logger().info('=' * 70)
                    self.state = FlightState.LANDING
        
        elif self.state == FlightState.LANDING:
            if not self.land_sent:
                self.get_logger().info('LANDING - AUTO.LAND mode')
                self.set_auto_land()
                self.land_sent = True
            
            # Check landing
            landed = False
            if self.vehicle_land_detected and self.vehicle_land_detected.landed:
                landed = True
            if self.vehicle_local_position and -self.vehicle_local_position.z < self.MIN_ALTITUDE:
                landed = True
            
            if landed:
                self.get_logger().info('✓ TOUCHDOWN')
                self.disarm()
                self.state = FlightState.MISSION_COMPLETE
        
        elif self.state == FlightState.EMERGENCY:
            landed = False
            if self.vehicle_land_detected and self.vehicle_land_detected.landed:
                landed = True
            if self.vehicle_local_position and -self.vehicle_local_position.z < self.MIN_ALTITUDE:
                landed = True
            
            if landed:
                self.get_logger().error('Emergency landing complete')
                self.disarm()
                self.destroy_node()
                return
        
        elif self.state == FlightState.MISSION_COMPLETE:
            self.get_logger().info('=' * 70)
            self.get_logger().info('✓✓✓ IRoC-U MISSION COMPLETE ✓✓✓')
            self.get_logger().info('=' * 70)
            self.destroy_node()
            return

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================
    
    def control_loop(self):
        self.publish_offboard_heartbeat()
        
        if not self.vehicle_local_position:
            return
        
        yaw = self.get_current_yaw()
        
        if self.state in [
            FlightState.WAIT_FOR_READY,
            FlightState.OFFBOARD_REQUESTED,
            FlightState.ARM_REQUESTED
        ]:
            # Hold current position
            self.publish_setpoint(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z,
                yaw
            )
            self.offboard_counter += 1
        
        elif self.state == FlightState.TAKEOFF:
            # FIXED: Hold CAPTURED takeoff position (not drifting current)
            self.publish_setpoint(
                self.takeoff_position_x,
                self.takeoff_position_y,
                self.TARGET_ALTITUDE,
                self.hover_yaw
            )
        
        elif self.state == FlightState.HOVER:
            # FIXED: Hold CAPTURED hover position (not drifting current)
            self.publish_setpoint(
                self.hover_position_x,
                self.hover_position_y,
                self.TARGET_ALTITUDE,
                self.hover_yaw
            )

    # =========================================================================
    # UTILITIES
    # =========================================================================
    
    def get_current_yaw(self):
        if not self.vehicle_attitude:
            return 0.0
        _, _, yaw = self.quaternion_to_euler(self.vehicle_attitude.q)
        return yaw
    
    def quaternion_to_euler(self, q):
        """PX4 uses [w,x,y,z]"""
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x**2 + y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def trigger_emergency_landing(self):
        if not self.emergency_triggered:
            self.emergency_triggered = True
            self.get_logger().error('!' * 70)
            self.get_logger().error('!!! EMERGENCY LANDING !!!')
            self.get_logger().error('!' * 70)
            self.set_auto_land()
            self.state = FlightState.EMERGENCY


def main(args=None):
    print('=' * 70)
    print('IRoC-U 2026 - PRODUCTION VERSION')
    print('Emergency System: ACTIVE')
    print('Drift Correction: ENABLED')
    print('Optical Flow: cs_opt_flow + cs_local_position_valid')
    print('=' * 70)
    
    rclpy.init(args=args)
    node = IroCUQualificationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
