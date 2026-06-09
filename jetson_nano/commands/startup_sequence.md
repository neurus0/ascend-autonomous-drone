# Startup Sequence — IRoC-U 2026 ASCEND Flight Stack

## Prerequisites
- Jetson Nano powered on and connected to hotspot
- Pixhawk connected via USB (`/dev/ttyACM0`)
- LiDAR connected via USB (`/dev/ttyUSB0`)
- GCS laptop on same WiFi network

---

## Boot Sequence (every flight)

### Step 1 — Start ROS master
```bash
roscore
```

### Step 2 — Fix serial permissions (new terminal)
```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0
```

### Step 3 — Launch full flight stack (new terminal)
```bash
roslaunch thl_drone unified_launch.launch
```

Default FCU: `/dev/ttyACM0:115200`

Custom port:
```bash
roslaunch thl_drone unified_launch.launch fcu_url:=/dev/ttyACM0:57600
```

---

## Node Startup Order (automatic, handled by launch file)

| Delay | Node                  | Purpose                        |
|-------|-----------------------|--------------------------------|
| 0s    | MAVROS                | FCU bridge                     |
| 0s    | lidar_alt_node        | TFMini-S altitude              |
| 5s    | emergency_monitor     | Safety watchdog                |
| 3s    | tcp_bridge_node       | GCS communication              |
| 6s    | throttle_hover_node   | Main flight controller         |
| 5s    | data_transfer_node    | Post-landing data upload       |

---

## Verify Stack is Running

```bash
rostopic list
rostopic echo /lidar/altitude
rostopic echo /mavros/state
```

---

## Manual Kill (emergency)

```bash
rostopic pub /gcs/kill_flag std_msgs/Bool "data: true"
```

---

## Shutdown

```bash
# Ctrl+C in the roslaunch terminal
# Then:
rosnode kill --all
```
