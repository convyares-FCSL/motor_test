# motor_test (ROS 2 Python)

Small ROS 2 workspace for Waveshare ESP32 serial-forwarded ST/SC servos.

## Structure

- `library/stservo-env/`: local vendor ST/SC Python SDK
- `src/servo_interfaces/`: ROS messages/services
- `src/servo_serial/`: lifecycle serial comms node (hardware access)
- `src/servo_motor/`: lifecycle motor action server (ID limits + one-axis execution)
- `src/servo_bridge/`: ROS web bridge + static slider UI
- `src/servo_bringup/`: lifecycle launch + config
- `scripts/`: helpers (`build_ros.sh`, `launch_ros.sh`, `run_ros.sh`, `move_st.sh`, single-servo utility)
- `docs/`: architecture and ops notes
- `archive/legacy_tools/`: earlier diagnostic scripts

## Nodes

- `servo_serial` (lifecycle)
  - Uses `STservo_sdk` from `library/stservo-env`
  - Opens COM/baud and exposes services:
    - `servo_serial/command` (`servo_interfaces/srv/ServoCommand`)
    - `servo_serial/control` (`servo_interfaces/srv/ServoControl`)
    - `servo_serial/read` (`servo_interfaces/srv/ServoRead`)

- `servo_motor` (lifecycle)
  - Enforces per-ID position limits
  - Hosts action server `servo_motor/move` (`servo_interfaces/action/MotorMove`)
  - Accepts one goal at a time (single-axis mode)
  - Publishes `servo/state` (`servo_interfaces/msg/MotorState`)
  - Calls `servo_serial/command`

- `ros_bridge` (regular node)
  - Hosts UI at `http://<host>:8080`
  - WebSocket `/ws` for action goals, 9 saved positions per motor, and live state

## Configure

Edit `.env`:

- `STS_PORT=COM13`
- `STS_USB_BAUD=115200`
- `STS_LIBRARY_PATH=/home/ecm/ai-workspace/projects/motor_test/library/stservo-env`
- `STS_SIMULATION=0` (`1` for offline simulation, no hardware)
- `STS_SIM_DEFAULT_POSITION=2048`
- `STS_WINDOWS_PROXY=1` (WSL/Linux + COM13: call Windows Python like `move_st.sh`)
- `STS_WINDOWS_PROXY_PYTHON="py -3"`
- `STS_WINDOWS_PROXY_TIMEOUT_S=10.0`
- `STS_WINDOWS_PROXY_SCRIPT=` (optional; defaults to `scripts/st_windows_proxy_daemon.py`)
- `STS_AUTO_FORWARD_ON=0` (`1` to auto-enable ESP32 serial forwarding before launch)
- `STS_FORWARD_BASE_URL=http://192.168.4.1` (ESP32 web UI host)
- `STS_FORWARD_HTTP_TIMEOUT_S=2`
- `STS_FORWARD_SETTLE_S=0.4`
- `ROS_BRIDGE_NAMES_FILE=/home/ecm/ai-workspace/projects/motor_test/data/motor_names.json`
- `ROS_BRIDGE_POSITIONS_FILE=/home/ecm/ai-workspace/projects/motor_test/data/motor_positions.json`
- `ROS_BRIDGE_LIVE_POLL_S=1.0` (live readback period for both axes)
- `SERVO_MOTOR_TYPE=waveshare_st3215`
- `STS_WAIT_FOR_IDS=1`
- `STS_REQUIRE_ALL_IDS=0` (recommended while stabilizing bus detection)
- `STS_WAIT_IDS_TIMEOUT_S=8.0`
- `STS_WAIT_IDS_RETRY_S=0.4`
- `STS_STARTUP_SETTLE_S=0.4`
- `STS_RECOVER_CYCLE_DELAY_S=0.25` (torque-off/on dwell used by `Recover`)

Edit motor limits in `src/servo_bringup/config/motors.yaml`:

- `motor_ids: [1, 2]`
- `motor_min: [...]`
- `motor_max: [...]`

Current configured limits:

- ID 1: 30%..65% of full scale (`1228..2662`)
- ID 2: 76%..100% of full scale (`3112..4095`)

## Build and Launch

Install web bridge dependency once:

```bash
sudo apt-get install python3-aiohttp
```

Build once:

```bash
./scripts/build_ros.sh
```

Launch without rebuilding:

```bash
./scripts/launch_ros.sh
```

Toggle ESP32 serial forwarding manually:

```bash
./scripts/serial_forwarding.sh on
./scripts/serial_forwarding.sh off
```

Convenience wrapper:

```bash
./scripts/run_ros.sh          # launch only
./scripts/run_ros.sh --build  # build then launch
```

With `--symlink-install`, Python code edits usually do not require rebuild. Rebuild after interface/package metadata changes.

Quick ROS action test:

```bash
ros2 action send_goal /servo_motor/move servo_interfaces/action/MotorMove "{id: 1, percent: 50.0, speed: 120, acc: 10, mode: 'manual'}"
```

## Offline Simulation Mode

To validate the full stack (launch + ROS nodes + web sliders) without hardware:

1. Set `.env`:
   - `STS_SIMULATION=1`
2. Launch:
   - `./scripts/launch_ros.sh`
3. Open:
   - `http://<host>:8080`

In simulation mode, `servo_serial` does not open COM and returns fake successful read/command feedback for IDs listed in `known_ids`.

## Motor Name Persistence

The web UI supports per-motor name editing with one `Save Names` button.

- Names are persisted to JSON at `ROS_BRIDGE_NAMES_FILE`.
- Names are reloaded on startup and pushed to connected browser clients.

## Saved Positions

- Each motor has 9 saved percentage slots (`S1..S9`).
- Slot values persist to `ROS_BRIDGE_POSITIONS_FILE`.
- UI mode `Manual` sends on slider release.
- UI mode `Saved Slot` sends immediately when a slot button is pressed.
- Axis selection is via radio buttons (`ID 1` / `ID 2`).
- Mode selection is a single toggle button (`Manual` / `Saved Slot` / `Dual Preset`).
- Saved-position buttons are hidden while in `Manual` mode.
- Manual mode includes setup buttons: `Release`, `Set Middle`, `Middle`, `Stop`, `Torque On`, `Recover`.
- Manual mode includes jog controls (`Jog -` / `Jog +`) with configurable step percent.
- Manual mode supports per-axis manual range (`Min %` / `Max %`) and these limits are persisted.
- Speed and acceleration in the web UI are sliders in percent (`0..100`), mapped to the active motor profile raw limits.
- A global `STOP ALL` button is always visible and does two things:
  - Cancels any in-flight action goals.
  - Sends `stop` control commands to all known motor IDs.
- State cards are color-coded from live telemetry/error text:
  - `OK` (green): valid live readback.
  - `WARN` (amber): communication/result issue not classified as hard fault.
  - `FAULT` (red): overload/overheat/voltage/current/angle errors detected.
  - `OFFLINE` (blue-gray): timeout/no-status/transport unavailable.

## Dual Presets

- `Dual Preset` mode stores 9 named presets (`D1..D9`).
- Each dual preset includes:
  - Preset name
  - ID 1 target percent
  - ID 2 target percent
- Clicking a dual preset button triggers both axis moves from one click.
- Because motor execution is currently one-axis-at-a-time, dual preset execution is sequenced internally (ID 1 then ID 2).
- Bridge polls live position every `ROS_BRIDGE_LIVE_POLL_S` seconds using `servo_serial/read` and updates both axis state boxes.

## Motor Profile

- Current profile: `waveshare_st3215`
- UI tuning slider mapping:
  - Speed raw max: `3073`
  - Acc raw max: `150`

## Quick Direct Test (non-ROS utility)

```bash
./scripts/move_st.sh --id 1 --percent 50 --speed 120
```

or positional shortcut:

```bash
./scripts/move_st.sh 1 80 120
```

## Notes

- Serial forwarding on the ESP32 must be enabled for raw ST/SC packet flow.
- Firmware control endpoint used by the script is:
  - `/cmd?inputT=1&inputI=14&inputA=0&inputB=0` -> forwarding ON
  - `/cmd?inputT=1&inputI=15&inputA=0&inputB=0` -> forwarding OFF
- In WSL, `COM13` is not a Linux serial device. `STS_WINDOWS_PROXY=1` makes `servo_serial` use Windows Python/COM transport (same path as `./scripts/move_st.sh`).
- In proxy mode, COM is opened once by `st_windows_proxy_daemon.py`; ID scan happens during lifecycle configure, not per move.
- `servo_bringup` launch auto-configures and auto-activates lifecycle nodes.
