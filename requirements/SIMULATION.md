# IRoC-U 2026 Autonomous Drone Simulation Environment

## Vehicle

**Platform:** Quadcopter


## Operating System

| Component        | Version          |
| ---------------- | ---------------- |
| Operating System | Ubuntu 24.04 LTS |


## Flight Controller Software

| Component       | Version                         |
| --------------- | ------------------------------- |
| PX4 Autopilot   | v1.15.4                         |
| Simulation Mode | PX4 SITL (Software In The Loop) |


## Simulation Environment

| Component       | Version                        |
| --------------- | ------------------------------ |
| Gazebo Harmonic | Current Simulation Environment |
| gz-sim          | 8.10.0                         |


## Ground Control Station

| Component              | Software       |
| ---------------------- | -------------- |
| Ground Control Station | QGroundControl |


## ROS Framework

| Component        | Version / Package   |
| ---------------- | ------------------- |
| ROS Distribution | ROS 2 Jazzy Jalisco |
| Middleware       | Micro XRCE-DDS      |
| PX4 Integration  | px4_ros_com         |
| PX4 Messages     | px4_msgs            |


## Custom ROS 2 Components

### Packages

* `optical_flow_node`

### Scripts

* `offboard_control.py`

### Launch Files

* `sensor_combined_listener.launch.py`

---

## Drone Models

The following drone models were used within the simulation environment:

* `x500`
* `x500_base`
* `x500_depth`

---

## Custom Environment

### Mars Terrain

Custom terrain environment developed for simulation-based testing and validation.

* Environment Name: `mars_terrain`
* Format: Gazebo Simulation World

---

## 3D Modeling and Asset Tools

The following tools were used for asset creation, modification, and optimization:

* Blender
* MeshLab
* FreeCAD

---

## Simulation Assets

| Asset Type | Description                   |
| ---------- | ----------------------------- |
| SDF        | Simulation Description Format |
| DAE        | COLLADA 3D Models             |
| STL        | Mesh Models                   |
| PNG        | Textures and Visual Assets    |
| OBJ        | Terrain Models                |
| MTL        | Material Definitions          |

---

## Testing and Validation

The simulation environment was used for the following validation activities:

* Autonomous Takeoff
* Hover Stability Testing
* Autonomous Landing
* Vehicle Odometry Monitoring
* Estimator Status Monitoring
* Sensor Data Monitoring

---

## Useful ROS 2 Commands

### Vehicle Position

```bash
ros2 topic echo /fmu/out/vehicle_odometry | grep position -A 3
```

### Sensor Data

```bash
listener sensor_combined
```

### Estimator Status

```bash
listener estimator_status_flags
```

---

## Repository Purpose

This simulation environment was developed to support autonomous drone development for the ISRO Robotics Challenge (IRoC-U 2026).

Primary objectives include:

* Autonomous drone control development
* PX4 offboard control testing
* Sensor integration and validation
* SITL experimentation
* ROS 2 integration
* Navigation workflow testing
* Mission qualification and validation
* Safe software testing before hardware deployment