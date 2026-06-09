# ROS Node Architecture — IRoC-U 2026 ASCEND

## Stack Overview

All nodes run on Jetson Nano under a single roslaunch command.
Communication between nodes is via ROS topics and MAVROS services.

---

## Node Map

GCS Laptop
    |
    | TCP:5005
    |
tcp_bridge_node
    |-- publishes -> /gcs/flight_command   -> throttle_hover_node
    |-- publishes -> /gcs/kill_flag        -> kill.py (via throttle_hover)
    |-- subscribes <- /drone/flight_state
    |-- subscribes <- /lidar/altitude
    |-- subscribes <- /mavros/state
    |-- subscribes <- /mavros/imu/data
    |-- subscribes <- /mavros/battery

lidar_alt_node (TFMini-S on /dev/ttyUSB0)
    |-- publishes -> /lidar/altitude       -> throttle_hover, emergency_monitor, tcp_bridge, data_transfer, kill
    |-- publishes -> /lidar/strength
    |-- publishes -> /lidar/status
    |-- publishes -> /mavros/distance_sensor/rangefinder_sub

throttle_hover_node (main flight controller)
    |-- subscribes <- /lidar/altitude
    |-- subscribes <- /gcs/flight_command
    |-- subscribes <- /gcs/kill_flag
    |-- publishes -> /mavros/rc/override   -> Pixhawk (via MAVROS)
    |-- publishes -> /drone/flight_state
    |-- calls service -> /mavros/cmd/arming
    |-- calls service -> /mavros/set_mode

emergency_monitor_node (safety watchdog)
    |-- subscribes <- /lidar/altitude
    |-- subscribes <- /mavros/state
    |-- subscribes <- /mavros/imu/data
    |-- subscribes <- /mavros/battery
    |-- publishes -> /gcs/kill_flag        -> triggers kill descent
    |-- publishes -> /drone/flight_state

data_transfer_node (post-landing)
    |-- subscribes <- /drone/flight_state
    |-- subscribes <- /lidar/altitude
    |-- HTTP POST -> GCS laptop:3000/api/mission-data
    |-- HTTP POST -> GCS laptop:3000/api/upload-image

MAVROS (FCU bridge on /dev/ttyACM0)
    |-- bridges all /mavros/* topics <-> Pixhawk MAVLink

## Key Topics

| Topic                   | Type         | Publisher                  | Subscribers                                               |
|-------------------------|--------------|----------------------------|-----------------------------------------------------------|
| /lidar/altitude         | Float32      | lidar_alt_node             | throttle_hover, emergency, tcp_bridge, data_transfer, kill|
| /gcs/kill_flag          | Bool         | tcp_bridge, emergency      | throttle_hover                                            |
| /gcs/flight_command     | String       | tcp_bridge                 | throttle_hover                                            |
| /drone/flight_state     | String       | throttle_hover, kill, emergency | tcp_bridge, data_transfer                            |
| /mavros/rc/override     | OverrideRCIn | throttle_hover, kill       | MAVROS -> Pixhawk                                         |
| /mavros/state           | State        | MAVROS                     | tcp_bridge, emergency, throttle_hover                     |

---

## Safety Chain

emergency_monitor detects fault
        |
        v
publishes /gcs/kill_flag = True
        |
        v
throttle_hover receives kill flag
        |
        v
calls kill.begin_descent(current_throttle)
        |
        v
kill.py ramps throttle down, monitors /lidar/altitude
        |
        v
LiDAR confirms ground -> MAVROS disarm service
        |
        v
publishes /drone/flight_state = DISARMED
