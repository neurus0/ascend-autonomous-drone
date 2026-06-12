# Hardware Requirements

## Overview

This project was developed and tested using a custom quadcopter platform consisting of flight-control, sensing, communication, power-distribution, and docking subsystems.

The hardware stack includes:

* S450 Frame
* Brushless DC Motors
* Electronic Speed Controllers (ESCs)
* Radiolink Pixhawk 2.4.8 Flight Controller
* NVIDIA Jetson Nano Companion Computer
* PX4Flow Optical Flow Sensor
* TF Mini Plus LiDAR
* Logitech webcam
* Holybro SiK Telemetry Radio
* TP-Link WiFi Module
* Power Distribution Board (PDB)
* UBEC Voltage Regulator
* LiPo Battery System
* Docking and Charging Hardware

## Detailed Hardware Documentation

For complete hardware specifications, quantities, descriptions, images, and subsystem information, refer to:

```text
hardware/HARDWARE_OVERVIEW.md
```

## Hardware Architecture

The complete hardware architecture diagram is available in:

```text
hardware/images/system_arch.png
```

## Notes

The hardware configuration documented in this repository corresponds to the ASCEND autonomous drone platform developed for the ISRO Robotics Challenge (IRoC-U 2026).

Hardware components were selected to support simulation, ROS integration, MAVLink communication, onboard processing, sensor integration, and autonomous flight experimentation.