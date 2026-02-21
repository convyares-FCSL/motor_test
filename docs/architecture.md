# Architecture

## Data Flow

1. Web slider (`servo_bridge`) sends `{id, percent, speed, acc}`.
2. `servo_bridge` publishes `servo/command_percent`.
3. `servo_motor` validates ID and clamps percent by configured limits.
4. `servo_motor` converts percent to absolute position and calls `servo_serial/command`.
5. `servo_serial` sends ST library commands over COM and returns present position.
6. `servo_motor` publishes `servo/state`; `servo_bridge` pushes updates to browser.
7. `servo_bridge` stores user-assigned motor names in `motor_names.json`.

## Separation of Concerns

- `servo_serial`: transport + vendor SDK boundary
- `servo_motor`: control policy and limits
- `servo_bridge`: UX and browser transport
  - Stores/loads editable motor names for IDs.
- `servo_interfaces`: type contract between nodes

## Lifecycle

- `servo_serial`: configure -> open port, set baud, ping known IDs
- `servo_serial`: activate -> expose services
- `servo_motor`: configure -> load limits, connect service client
- `servo_motor`: activate -> subscribe/publish + command pump timer

## Offline Simulation

`servo_serial` supports a simulation backend selected by launch parameter `simulation_enabled`.

- `simulation_enabled=true`:
  - No COM port open.
  - Per-ID in-memory position state initialized from `simulation_default_position`.
  - `servo_serial/command` updates simulated position and returns success.
  - `servo_serial/read` returns simulated position.

This allows validating UI -> bridge -> ROS topics -> services -> feedback flow with no hardware attached.
