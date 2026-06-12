# My Contributions

## Overview

ASCEND was developed as a team project for autonomous drone research and competition participation. My primary responsibility was the software, systems integration, simulation, communication, and deployment stack that connected the drone's hardware, onboard computing, flight controller, and ground station into a functioning autonomous system.

My work focused on understanding, configuring, integrating, testing, debugging, and maintaining the complete robotics pipeline rather than working on a single isolated subsystem.

---

## Areas of Ownership

### Linux & System Administration

* Ubuntu installation and configuration
* Package management and dependency handling
* Network configuration between devices
* System-level troubleshooting and debugging
* Development environment setup and maintenance

### ROS Development

* ROS workspace setup
* Package management
* Launch file creation and maintenance
* Service file management
* Node integration
* Multi-component communication architecture

### Simulation & Testing

* Gazebo simulation setup
* Environment preparation
* SITL (Software-In-The-Loop) testing
* HITL (Hardware-In-The-Loop) testing
* Validation of software behavior before hardware deployment

### Jetson Nano Development

* Companion computer configuration
* ROS deployment
* Software installation
* Runtime debugging
* Communication setup
* System recovery after configuration failures

### Pixhawk Integration

* Pixhawk communication setup
* MAVLink configuration
* Parameter management
* Mission Planner integration
* Controller troubleshooting and validation

### Communication Architecture

Worked on communication between:

* Laptop
* Ground Control Station
* Jetson Nano
* Pixhawk Flight Controller

Including:

* MAVLink communication
* Network configuration
* Command transmission
* Telemetry validation
* Data transfer workflows

### Sensor Integration

Worked with:

* IMU
* Optical Flow Sensor
* LiDAR

Responsibilities included:

* Sensor validation
* Data access verification
* Software integration
* Testing within the autonomous control framework

### Ground Control Station

Contributed to development and integration of:

* Command execution workflows
* Telemetry visualization
* Communication interfaces
* Data transfer mechanisms
* System monitoring utilities

---

## Engineering Challenges

### Learning an Entire Robotics Stack

Entered the project without prior experience in:

* ROS
* Gazebo
* SITL
* HITL
* MAVLink
* Companion computer development

Built working knowledge through documentation, experimentation, debugging, and repeated testing.

### Hardware and Configuration Failures

Worked through:

* Pixhawk failures
* Jetson reconfiguration
* Environment recreation
* Communication failures
* System reintegration after hardware setbacks

### System Integration Complexity

A recurring challenge was that individual components often worked independently but failed when combined.

This required system-level debugging across:

* ROS
* Jetson Nano
* Pixhawk
* Sensors
* Mission Planner
* Communication channels

---

## Key Takeaway

The most valuable lesson from ASCEND was learning to treat robotics systems as integrated engineering platforms rather than separate hardware and software components. A significant portion of my work involved understanding how sensing, computation, communication, flight control, simulation, and deployment interact as a complete autonomous system.