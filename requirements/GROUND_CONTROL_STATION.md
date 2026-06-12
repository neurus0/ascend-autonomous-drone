# Ground Control Station Requirements

## Overview

The Ground Control Station (GCS) serves as the primary operator interface for the ASCEND autonomous drone platform.

The system consists of a web-based frontend, a Node.js backend, telemetry handling modules, communication bridges, and data logging utilities.


## Operating System

| Component            | Requirement     |
| -------------------- | --------------- |
| Operating System     | Windows / Linux |
| Network Connectivity | Required        |


## Backend Environment

| Component       | Requirement |
| --------------- | ----------- |
| Runtime         | Node.js     |
| Package Manager | npm         |

### Backend Entry Point

```text
server.js
```


## Node.js Dependencies

| Package                     | Purpose                         |
| --------------------------- | ------------------------------- |
| express                     | Backend web server              |
| socket.io                   | Real-time communication         |
| serialport                  | Serial communication            |
| @serialport/parser-readline | Serial data parsing             |
| cors                        | Cross-origin communication      |
| dotenv                      | Environment variable management |
| multer                      | File uploads                    |
| http-proxy-middleware       | Proxy services                  |
| winston                     | Logging                         |
| xlsx                        | Flight log generation           |

Source: package.json

---

## Development Dependencies

| Package | Purpose                         |
| ------- | ------------------------------- |
| nodemon | Development server auto-restart |

Source: package.json

---

## Backend Commands

### Production

```bash
npm start
```

### Development

```bash
npm run dev
```

### Simulation Mode

```bash
npm run sim
```

Source: package.json

---

## Frontend Components

Location:

```text
ground_control_station/frontend/public/
```

### Features

* Mission Monitoring
* Telemetry Visualization
* Mapping Interface
* Camera Feed Integration
* Vehicle Status Display
* Real-Time Charts

---

## Communication Components

### Drone Bridge

Location:

```text
ground_control_station/drone-bridge/
```

Files:

```text
drone_bridge.py
camera_stream.sh
```

Purpose:

* Drone communication
* Camera streaming
* Data relay

---

### Drone Companion

Location:

```text
ground_control_station/drone-companion/
```

Files:

```text
bridge.py
camera_stream.py
data_collector.py
```

Purpose:

* Companion computer communication
* Telemetry forwarding
* Data collection

---

## Logging

Location:

```text
ground_control_station/backend/logs/
```

Example:

```text
flight_log.xlsx
```

Purpose:

* Flight data storage
* Telemetry recording
* Post-flight analysis

---

## Networking

The Ground Control Station communicates with the onboard Jetson Nano using network-based communication channels and telemetry links.

The Ground Control Station acts as the operator-facing layer of the ASCEND system and provides monitoring, visualization, command execution, and data collection capabilities.

---

## Repository Structure

```text
ground_control_station/
├── backend/
├── frontend/
├── drone-bridge/
├── drone-companion/
└── images/
```