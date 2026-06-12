# Jetson Nano Requirements

## Hardware Platform

| Component          | Specification      |
| ------------------ | ------------------ |
| Companion Computer | NVIDIA Jetson Nano |
| RAM                | 4 GB               |
| Storage            | 32 GB SD Card      |


## Operating System

| Component | Version   |
| --------- | --------- |
| Ubuntu    | 18.04 LTS |


## Programming Environment

| Component | Version  |
| --------- | -------- |
| Python    | Python 2 |


## ROS Environment

| Component    | Version             |
| ------------ | ------------------- |
| ROS          | ROS 1               |
| Distribution | ROS Melodic Morenia |

### Notes

The Jetson Nano hardware stack was developed using ROS 1 (Melodic) rather than ROS 2.

During development, the simulation environment used ROS 2 Jazzy on Ubuntu 24.04, while the Jetson Nano remained on Ubuntu 18.04. This resulted in two separate software environments being maintained throughout the project.


## MAVLink Communication

| Component | Status                                                 |
| --------- | ------------------------------------------------------ |
| MAVROS    | Used                                                   |
| MAVLink   | Used for communication between Jetson Nano and Pixhawk |

### Purpose
* Flight controller communication
* Telemetry exchange
* Command transmission
* Vehicle status monitoring



## Flight Controller Interface

| Component              | Specification           |
| ---------------------- | ----------------------- |
| Flight Controller      | Radiolink Pixhawk 2.4.8 |
| Communication Protocol | MAVLink                 |
| ROS Interface          | MAVROS                  |


## Connected Sensors

* Logitech Camera
* TF Mini Plus LiDAR

## Network Configuration

The Jetson Nano acted as the onboard companion computer and communication hub.

Responsibilities included:

* Communication with Ground Control Station
* Communication with Pixhawk
* Sensor data processing
* ROS node execution
* Telemetry handling


## ROS Packages

### Repository Package

```text
thl_drone
```

### Nodes

```text
data_transfer.py
emergency_monitor.py
kill.py
lidar_alt.py
photo_capture.py
tcp_bridge.py
throttle_hover.py
vio_nav.py
```

## Launch Files

```text
unified_launch.launch
```

Location:

```text
jetson_nano/launch_files/
```

## System Services

### Linux Service

```text
ascend_drone.service
```

Purpose:

* Automatic startup of onboard software
* Service management
* System initialization


## Repository Structure

```text
jetson_nano/
├── commands/
├── configuration/
├── documentation/
├── launch_files/
├── ros_packages/
├── scripts/
└── service_files/
```

## Notes

This environment represents the onboard hardware deployment stack used on the physical ASCEND drone platform.

The Jetson Nano was used as the companion computer responsible for ROS execution, sensor interfacing, MAVLink communication, telemetry handling, and integration with the Ground Control Station and Pixhawk flight controller.