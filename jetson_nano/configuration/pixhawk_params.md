# Pixhawk Parameters — IRoC-U 2026 ASCEND

Tuned parameters for GPS-denied autonomous flight on ArduCopter.
Set via Mission Planner or MAVProxy.

---

## Failsafe (Disabled for autonomous ops)

| Parameter       | Value | Reason                              |
|-----------------|-------|-------------------------------------|
| FS_THR_ENABLE   | 0     | Disable RC throttle failsafe        |
| FS_GCS_ENABLE   | 0     | Disable GCS heartbeat failsafe      |
| ARMING_CHECK    | 0     | Disable all pre-arm checks          |

---

## Flight Controller

| Parameter       | Value | Reason                              |
|-----------------|-------|-------------------------------------|
| DISARM_DELAY    | 0     | Instant disarm after landing        |

---

## EKF / Navigation (GPS-Denied)

| Parameter       | Value | Reason                              |
|-----------------|-------|-------------------------------------|
| GPS_TYPE        | 0     | Disable GPS entirely                |
| EK3_ENABLE      | 1     | Use EKF3                            |
| EK3_SRC1_POSXY  | 0     | No XY position source               |
| EK3_SRC1_VELXY  | 5     | Optical flow for XY velocity        |
| EK3_SRC1_POSZ   | 1     | Barometer for Z (fallback)          |

---

## Throttle Controller (THL Mission)

Configured in `throttle_hover.py` — not Pixhawk params.

| Variable              | Value | Reason                          |
|-----------------------|-------|---------------------------------|
| TARGET_ALT_M          | 1.0   | Hover target altitude           |
| THROTTLE_INCREMENT    | 5     | PWM step per control loop       |
| BASE_THROTTLE_PCT     | 0     | Start from zero throttle        |

---

## How to Apply

### Via MAVProxy
```bash
param set FS_THR_ENABLE 0
param set FS_GCS_ENABLE 0
param set ARMING_CHECK 0
param set DISARM_DELAY 0
param set GPS_TYPE 0
```

### Via Mission Planner
Config/Tuning → Full Parameter List → search each parameter → set value → Write Params
