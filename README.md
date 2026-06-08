# C-UASC App — Mt. SAC UAV Team

Android drone control application built on the DJI Mobile SDK V4 sample project for the Mt. SAC UAV team competing in the **C-UASC (Collegiate Unmanned Aerial Systems Competition)**. The app adds four autonomous mission systems on top of the base SDK sample — geofencing, virtual stick waypoint navigation, a timed circuit, and a payload drop.

---

## Features

### Geofencing
Polygon-based airspace containment enforced in real time using the DJI flight controller state callback (~10 Hz).

- Define a boundary by entering lat/lng pairs manually or pasting a multi-line CSV block into the bulk-import field
- Waypoints persist across app restarts via `SharedPreferences`
- Fence state persists across view destruction (static fields), so enforcement continues even while navigating to other screens
- On boundary breach, Return-to-Home is triggered automatically and the flight log is closed
- Ray-casting point-in-polygon algorithm handles arbitrary convex/concave boundaries
- Ships with a default Mt. SAC area bounding polygon loaded on first launch

### Virtual Stick Waypoint Mission
Autonomous sequential waypoint navigation using DJI Virtual Stick mode. The drone takes off, flies to each waypoint in order, dwells at each one, then returns home.

**Architecture**

| File | Responsibility |
|---|---|
| `VirtualStickWaypointView` | UI widgets, button handlers, `MissionCallback` implementation |
| `WaypointMissionController` | Mission state machine, 20 Hz control loop, dwell logic, RTH |
| `WaypointStore` | Waypoint list, `SharedPreferences` persistence, QR parsing |
| `WaypointNavigationMath` | Haversine distance, bearing, coordinate conversion (no Android/DJI deps) |
| `TuningPanel` | PD gain UI, persisted to `SharedPreferences` |
| `MissionCallback` | Interface connecting controller → view |

**Control law**

- **Cruise phase** (distance > braking distance): fly at max horizontal speed
- **Braking phase** (distance ≤ braking distance): PD controller — `speed = Kp × distance − Kd × distanceRate`, clamped to `[MIN, MAX]`
- Vertical: proportional controller on altitude error, clamped to ±`MAX_VERTICAL_SPEED`
- Yaw: P controller with deadband; drone nose tracks the bearing toward the current waypoint

**Waypoint dwell**

Default dwell at each waypoint is **10 seconds**. Change `WAYPOINT_DWELL_MS` in `WaypointMissionController.java`. A per-second countdown updates the status label during the dwell.

**Waypoint import**

Waypoints can be entered manually, imported via QR code scan (`QrWaypointScanActivity`), or pre-loaded from `WaypointStore.loadDefaults()`. The QR parser accepts three payload formats:
- JSON object: `{ "type": "WAYPOINTS", "points": [ {"lat": ..., "lng": ..., "alt": ...} ] }`
- JSON array: `[ {"lat": ..., "lng": ..., "alt": ...} ]`
- Plain text CSV: one `lat,lng[,alt]` pair per line

Scanning gives an Append / Replace / Cancel dialog before modifying the stored list.

**Takeoff handling**

If the drone is already airborne, Virtual Stick is enabled immediately. If on the ground, `startTakeoff()` is called first and the mission begins once the drone exceeds `TAKEOFF_STABLE_ALT_M` (default 1.0 m).

**Offline debug mode**

Set `OfflineDebugConfig.OFFLINE_DEBUG_MODE = true` to simulate the full mission mathematically without a physical drone. The simulated position is updated by integrating velocity over each control tick.

### Time Trial Mission
Flies through waypoints (gates) as fast as possible with no dwell. Split times and total elapsed time are displayed live.

| File | Responsibility |
|---|---|
| `TimeTrialView` | UI entry point, timing display widgets |
| `TimeTrialMissionController` | High-speed control loop, gate detection, split timing |
| `TimeTrialWaypointStore` | Waypoint list + persistence (separate from the standard store) |
| `TimeTrialTuningPanel` | PD parameter UI |
| `TimeTrialCallback` | Interface connecting controller → view |

Shares `QrWaypointScanActivity`, `WaypointNavigationMath`, and `FlightLogger` with the waypoint system.

### Payload Drop
`PayloadDropMissionView` + `PayloadDropController` — navigate to a drop target and trigger a release. Drop state is tracked to prevent multiple actuations. The drop signal is sent via the DJI FlightAssistant downward fill light (`FillLightMode.ON`), matching behavior from `LEDControlView.java`.

---

## Flight Data Logging

`FlightLogger` writes a CSV to the device's external app files directory on every flight session.

**CSV format**
```
timestamp_ms, latitude, longitude, inside
1776131887625, 33.991556, -118.055788, true
```

**Pull the latest log**
```bash
./pull_flight_log.sh [destination_folder]
# default destination: ~/Desktop
```

Or manually via adb:
```bash
adb pull /sdcard/Android/data/com.dji.sdk.sample/files/flight_log_<timestamp>.csv
```

Sample flight logs from the April 2026 test sessions are included in the repo under `flight_log_2026-04-14_14-16-02.csv/`.

---

## QR Code Generator

`scripts/create_waypoint_qr.py` generates a QR image loadable by the in-app scanner.

**Setup (one time)**
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r scripts/requirements-qr.txt
```

**Generate from hardcoded waypoints**
```bash
python3 scripts/create_waypoint_qr.py --output waypoints_qr.png
```

**Generate from a CSV file**
```bash
python3 scripts/create_waypoint_qr.py --input scripts/my_waypoints.csv --output waypoints_qr.png
```

CSV rows can be `name,latitude,longitude,altitude_m`, `latitude,longitude,altitude_m`, or `latitude,longitude`.

---

## Build & Install

**Requirements**

- Android Studio (Arctic Fox or later)
- Android device running API 23+ (arm64-v8a or armeabi-v7a)
- DJI Mobile SDK V4 (included via project dependencies)

**Build**
```bash
./gradlew assembleDebug
```

**Install**
```bash
adb install app/build/outputs/apk/debug/app-debug.apk
```

The app ID is `com.dji.sdk.sample`. A valid DJI App Key must be set in `AndroidManifest.xml` before building.

---

## Project Structure

```
app/src/main/java/com/dji/sdk/sample/
├── demo/
│   ├── geofencing/          # Polygon geofencing + FlightLogger
│   ├── virtualstickwaypoint/# VS waypoint mission (main competition system)
│   ├── timetrial/           # Timed circuit mission
│   ├── drop/                # Payload drop controller
│   ├── map/                 # Map + mission data store
│   └── ...                  # Standard DJI SDK demo modules
├── internal/
│   ├── controller/          # Application + MainActivity
│   ├── utils/               # Helpers, ToastUtils, OfflineDebugConfig
│   └── view/                # Base view classes
scripts/
├── create_waypoint_qr.py    # QR code generator
└── my_waypoints.csv         # Sample waypoint CSV
pull_flight_log.sh           # adb log-pull helper
```

---

## Competition Context

Built for the [C-UASC](https://www.c-uasc.com/) competition. Mission systems target the autonomous navigation, geofencing, and payload delivery tasks defined in the C-UASC rules. Default waypoints in `WaypointStore.java` are set to the CSULA competition field.
