# Jetson Nano — Flight Stack

IRoC-U 2026 ASCEND autonomous flight stack running on Jetson Nano Developer Kit.

## Hardware
- Jetson Nano Developer Kit (4GB)
- Radiolink Pixhawk (ArduCopter) via `/dev/ttyACM0`
- Benewake TFMini-S LiDAR via `/dev/ttyUSB0`
- WiFi hotspot for GCS communication

## Software
- Ubuntu 18.04
- ROS Melodic
- MAVROS 1.16.0
- Python 2.7

## Quick Start

```bash
roscore
sudo chmod 666 /dev/ttyACM0 /dev/ttyUSB0
roslaunch thl_drone unified_launch.launch
```

## Contents

| Folder          | Description                                  |
|-----------------|----------------------------------------------|
| ros_packages/   | Full catkin package (thl_drone)              |
| commands/       | Boot sequence and useful ROS commands        |
| configuration/  | Tuned Pixhawk parameters                     |
| documentation/  | ROS node architecture and topic map          |

## Key Files

| File                                        | Purpose                        |
|---------------------------------------------|--------------------------------|
| ros_packages/thl_drone/src/throttle_hover.py| Main flight controller         |
| ros_packages/thl_drone/src/lidar_alt.py     | TFMini-S altitude node         |
| ros_packages/thl_drone/src/kill.py          | Graceful descent controller    |
| ros_packages/thl_drone/src/tcp_bridge.py    | GCS communication bridge       |
| ros_packages/thl_drone/src/emergency_monitor.py | Safety watchdog            |
| ros_packages/thl_drone/launch/unified_launch.launch | Single launch file     |
