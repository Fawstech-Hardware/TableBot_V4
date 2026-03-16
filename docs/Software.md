# TableBot_V4 Software Documentation

This document explains the software interface, web server endpoints, and real-time telemetry system for TableBot_V4.

## Web Server

The ESP32 acts as a SoftAP (Access Point) allowing direct connection without the need for an external router.
- **SSID**: `TableBot`
- **Password**: `TableBot@2025`
- **IP Address**: `192.168.4.1`

When connected, simply open `http://192.168.4.1` in any browser to view the control panel.

## Control Interface (Web UI)

The web application consists of a single HTML page enriched with CSS for styling and JavaScript to execute asynchronous HTTP requests (AJAX/Fetch API).

- **D-Pad Component**
  - Manually controls the robot in four primary directions.
  - Sends `/cmd?c=fwd`, `/cmd?c=bwd`, `/cmd?c=left`, `/cmd?c=right` endpoints.
  - The big red **STOP** button invokes `/cmd?c=stop` and acts as an emergency stop.
- **Auto Cleaning Dashboard**
  - Input fields for Table Length and Width (currently unused in actual firmware logic but maintained for UI scaling or future expansion).
  - "Start Cleaning" triggers `/auto?l={length}&w={width}`, transitioning the firmware into `MODE_AUTO`.

## Telemetry System

The robot continuously streams back health and state information via a persistent JSON telemetry endpoint.

- **`/telemetry` JSON Endpoint**
  - Requested automatically by the browser every 500ms via `setInterval()`.
  - Used to dynamically update the UI without reloading the page.

### Example JSON Payload:
```json
{
  "o": 35.8,
  "el": false,
  "er": false,
  "rl": false,
  "rr": false,
  "yaw": 0.5,
  "dist": 10.2,
  "tl": 1450,
  "tr": 1450,
  "pass": 0,
  "total": 0,
  "m": "CLEANING"
}
```

### Field Definitions:
- `o`: Ultrasonic obstacle distance (cm).
- `el`, `er`, `rl`, `rr`: IR Edge sensor flags (Left/Right, Front/Rear). True = Edge Detected.
- `yaw`: Current gyro deviation in degrees.
- `dist`: Cumulative straight-line distance traveled based on average encoder counts.
- `tl`, `tr`: Absolute encoder ticks for the Left and Right motors.
- `pass`, `total`: Reserved variables for future smart-pathing completion tracking.
- `m`: Machine state / Mode (e.g., `IDLE`, `MANUAL`, `CLEANING`, `TURNING`, `REVERSE`).
