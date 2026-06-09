# Drone GCS — Ground Control Station

Real-time browser-based drone dashboard for **Jetson Nano + Pixhawk** (ArduCopter) over WiFi.

---

## Project Structure

```
drone-gcs/
│
├── backend/                     ← Node.js server (ground laptop)
│   ├── server.js                ← Express + Socket.io + UDP listener
│   ├── mavlink-parser.js        ← MAVLink JSON → telemetry state
│   ├── simulator.js             ← Fake drone for UI testing
│   ├── package.json
│   └── .env                     ← Ports, camera URL, simulate flag
│
├── frontend/
│   └── public/
│       ├── index.html           ← 3-column dashboard
│       ├── css/style.css        ← Full GCS styling
│       └── js/
│           ├── app.js           ← Socket.io, telemetry, Camera Logs
│           ├── ahi.js           ← Artificial Horizon Indicator
│           ├── chart-vel.js     ← Velocity chart
│           └── simcam.js        ← Simulated camera canvas
│
├── drone-companion/             ← Scripts for Jetson / Raspberry Pi
│   ├── bridge.py                ← MAVLink → UDP/JSON forwarder (non-ROS)
│   ├── camera_stream.py         ← MJPEG HTTP camera server
│   └── data_collector.py        ← ROS1: captures image every 20 s + metadata
│
├── drone-bridge/
│   └── drone_bridge.py          ← Lightweight UDP bridge alternative
│
└── README.md
```

---

## Dashboard Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│  ⬡ DRONE GCS  |  MODE  ARMED  ◉ CONNECTED  |  START KILL RTL LAND  │
├─────────────┬──────────────────────────────────┬────────────────────┤
│  ATTITUDE   │  SYSTEM LOG & FAILURES (16:9)    │  BATTERY           │
│  (compact)  │  (vertical scroll, single lines) │                    │
├─────────────┤                                  │  CAMERA LOGS       │
│  POSITION   │                                  │  DATE TIME IMG     │
├─────────────│                                  │  LOC   VEL m/s     │
│  VELOCITY   │                                  │  (always visible,  │
├─────────────│──────────────────────────────────│   vertical scroll) │
│  CAMERA     │ FLIGHT MAP (fills rest)          │                    │
│  FEED (flex)│ (vertical scroll)                │                    │
└─────────────┴──────────────────────────────────┴────────────────────┘
```

---

## Prerequisites

### Ground Laptop
| Software | Version | Install |
|----------|---------|---------|
| Node.js  | ≥ 18.x  | https://nodejs.org |
| npm      | ≥ 9.x   | bundled |

### Jetson Nano (ROS1 system)
| Software | Install |
|----------|---------|
| ROS 1 Melodic | Ubuntu 18.04 |
| MAVROS | `sudo apt install ros-melodic-mavros ros-melodic-mavros-extras` |
| GeographicLib | `sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh` |
| Python 2 deps | `pip install requests numpy opencv-python` |

---

## Quick Start — Simulated Mode

```bash
cd drone-gcs/backend
npm install
npm run sim
# Open: http://localhost:3000
```

---

## Full Setup — Real Drone (Jetson + Pixhawk)

### 1 — Configure `backend/.env`

```env
PORT=3000
UDP_HOST=0.0.0.0
UDP_PORT=14550
SIMULATE=false
```

### 2 — Start GCS server (laptop)

```bash
cd backend && npm install && npm start
```

### 3 — ROS1 on Jetson

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch drone_mission mission.launch gcs_ip:=192.168.70.100
```

This starts all 6 ROS nodes:

| Node | Purpose |
|------|---------|
| `gcs_command_listener` | UDP listener for START / KILL / RTL from GCS |
| `mission_runner` | Arm → Takeoff 4 m → Hover 5 min → Land |
| `data_collector` | **Image every 20 s** during HOVER/TAKEOFF; logs DATE/TIME/IMG/LOC/VEL |
| `data_uploader` | **Uploads on battery CHARGING detection** after landing |
| `failure_reporter` | Forwards failures (LOW_BATTERY, TILT, etc.) to GCS |
| `telemetry_bridge` | Sends MAVROS telemetry at 20 Hz via UDP |

### 4 — Camera stream (optional live feed)

```bash
python3 drone-companion/camera_stream.py --port 8081
```

Enter `http://<jetson-ip>:8081/` in the Camera Feed URL box, click **⇄ SIM/LIVE**.

---

## Camera Logging

`data_collector.py` captures one JPEG **every 20 seconds** while in HOVER or TAKEOFF.

| Column | Value |
|--------|-------|
| DATE | DD/MM/YYYY (UTC) |
| TIME | HH:MM:SS (UTC) |
| IMG | Thumbnail / filename |
| LOCATION | ENU x,y in metres |
| VEL m/s | Horizontal ground speed |

Saved to: `~/mission_data/images/YYYYMMDD_HHMMSS_NNN.jpg`  
Log file: `~/mission_data/records.json`

**Upload trigger:** `data_uploader.py` watches `/mavros/battery`. Upload to GCS starts only when:
- Flight state is `MISSION_COMPLETE` or `LANDED`, **and**
- `battery.power_supply_status == CHARGING`

This ensures data transfers only when the drone has returned and the charger is connected. Live records appear in the **Camera Logs** panel as they arrive.

---

## Network

```
Jetson Nano (192.168.70.180)       Ground Laptop
  telemetry_bridge → UDP :14550  →  server.js :3000
  failure_reporter → UDP :14550  →  browser dashboard
  data_uploader    → HTTP :3000  →  /api/mission-data
  camera_stream.py → HTTP :8081  →  Camera Feed panel
  GCS commands     ← UDP :14555  ←  server.js → Jetson
```

---

## REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Server health |
| `/api/telemetry/latest` | GET | Current telemetry |
| `/api/mission-data` | GET | All camera log records |
| `/api/mission-data` | POST | Jetson uploads a record |
| `/api/upload-image` | POST | Jetson uploads an image |

---

## GCS Buttons

| Button | Command | Effect |
|--------|---------|--------|
| ▶ START | `START_MISSION` | Arm → 4 m takeoff → 5 min hover → land |
| ✕ KILL | `KILL` | Immediate AUTO.LAND + disarm |
| RTL | `RTL` | Return to launch |
| LAND | `LAND` | Descend and land |
| LOITER | `LOITER` | Hold position |

---

## Troubleshooting

**Dashboard shows DISCONNECTED** — check `server.js` running, WebSocket URL matches.  
**No telemetry** — check `telemetry_bridge` node running, `gcs_ip` param correct.  
**Camera Logs empty** — confirm `data_collector` node running, `/dev/video0` accessible.  
**No upload after landing** — confirm charger connected; `data_uploader` waits for charging state.  
**Camera feed blank** — verify `camera_stream.py` running on Jetson port 8081.  
**MAVROS timeout** — check Pixhawk at `/dev/ttyUSB0` baud 57600; `sudo chmod 666 /dev/ttyUSB0`.

---

## License

RAJDEEP MAITY
