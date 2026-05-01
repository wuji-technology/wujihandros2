# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **Tactile split into sibling ROS packages.** Tactile messages and
  the tactile driver node are no longer co-located with the joint
  package. See `docs/refactor-plan.md` for the full move map.
  - `wujihand_tactile_msgs/` (new package): `TactileFrame.msg`,
    `TactileDiagnostics.msg`, `SetTactileStreaming.srv`,
    `SetTactileSampleRate.srv`, `ResetTactileCounters.srv`.
  - `wujihand_tactile_driver/` (new package): `tactile_driver_node`
    binary, in C++ namespace `wujihand_tactile_driver`.
  - `wujihand_msgs/` shrinks back to joint-only (HandDiagnostics +
    SetEnabled + ResetError).
  - `wujihand_driver/` shrinks back to joint-only; entry renamed
    `main.cpp` → `wujihand_driver_main.cpp` for symmetry with
    `wujihand_driver_node.cpp`.
  - **Breaking** for downstream consumers:
    `find_package(wujihand_msgs)` no longer exposes tactile types →
    use `find_package(wujihand_tactile_msgs)`. Launch files referring
    to `Node(package="wujihand_driver", executable="tactile_driver_node")`
    must change `package` to `"wujihand_tactile_driver"`. No alias
    layer (refactor-plan.md §6).

- **Tactile stack rewired for the new firmware wire protocol**
  (`wh110-firmware/docs/tactile-wire-protocol.md`):
  - `wujihand_tactile_msgs/msg/TactileFrame.msg` carries
    `float32[768] pressure` in `[0.0, 1.0]` (was `int16[768]` with
    inverted polarity); `NaN` marks invalid cells. Field renamed
    `device_timestamp` → `device_timestamp_ms` for unit clarity.
  - `tactile_driver_node` parameters changed: dropped `pressure_max`,
    added `sample_rate_hz` (1..120, default 120) and
    `streaming_at_startup` (default true). Heatmap colormap and NaN
    handling rewritten accordingly.
  - Driver logs serial / hw_revision / fw_version / git short sha at
    startup via the new `GET_DEVICE_INFO` + `GET_FW_BUILD` commands.
  - New topic `~/tactile/diagnostics`
    (`wujihand_tactile_msgs/msg/TactileDiagnostics`, 10 Hz).
  - New services: `~/tactile/set_streaming`
    (`wujihand_tactile_msgs/srv/SetTactileStreaming`),
    `~/tactile/set_sample_rate`
    (`wujihand_tactile_msgs/srv/SetTactileSampleRate`),
    `~/tactile/reset_counters`
    (`wujihand_tactile_msgs/srv/ResetTactileCounters`).
  - Disconnect now reported via the SDK's `set_disconnect_callback`
    (was a fragile zero-init-frame heuristic that broke once `0.0`
    became a valid pressure value).
  - SDK upgraded to its tactile-namespaced API (`wujihandcpp::tactile::Board`
    etc.); ROS driver consumes via `find_package(wujihandcpp CONFIG)`.

- **Launch composition refactored**.
  `wujihand_full.launch.py` now uses `IncludeLaunchDescription` to
  delegate to `wujihand.launch.py` and `tactile.launch.py` instead of
  duplicating the per-driver `Node()` blocks. `tactile.launch.py`
  gained `parent_frame` and `namespace` args so it composes cleanly
  under any hand-namespaced TF tree.

- **RViz config simplified.** The two ~616-line `*_tactile.rviz`
  files are gone; `wujihand_full.launch.py` now composes the joint-only
  base config from `wuji_hand_description/rviz/{left,right}.rviz` with
  a 14-line `wujihand_bringup/rviz/tactile_overlay.txt` snippet at
  launch time, writing to a temp file (cleaned up on process exit).
  No new runtime dependency added (stdlib text splice; no PyYAML).

### Added

- `wujihand_tactile_msgs` package (5 interfaces, see split notes
  above).
- `wujihand_tactile_driver` package (1 binary, see split notes above).
- `docs/refactor-plan.md` — round-8 structural refactor design doc;
  authoritative sibling lives in `wujihandpy/docs/refactor-plan.md`.
- `wujihand_bringup/rviz/tactile_overlay.txt` — RViz Image-display
  snippet injected by `wujihand_full.launch.py` when tactile is
  active.
- `wujihand_bringup/package.xml` runtime deps that were missing:
  `tf2_ros` (for `static_transform_publisher`),
  `ament_index_python` (for `get_package_share_directory`).

### Removed

- `wujihand_bringup/rviz/left_tactile.rviz`,
  `wujihand_bringup/rviz/right_tactile.rviz` — replaced by the
  launch-time composer + `tactile_overlay.txt`.

## [1.0.1] - 2026-01-21

### Changed

- Driver publishes joint names with handedness prefix (e.g., `right_finger1_joint1`) to match URDF
- RViz startup delayed to 2.5s and dynamically selects config based on detected handedness

### Fixed

- TF tree now properly published by robot_state_publisher (joint names match URDF)
- Joint commands support both prefixed and non-prefixed names for backward compatibility

## [1.0.0] - 2026-01-16

### Added

- `rviz` and `foxglove` parameters for `wujihand.launch.py` for visualization control
- `hand_type` parameter for manual hand type override
- `wuji-hand-description` git submodule for unified URDF source
- Populate `effort` field in `joint_states` topic with real-time effort data
- Add `effort_limits` field to `HandDiagnostics` message for monitoring effort limit settings

### Changed

- Consolidate URDF to external `wuji-hand-description` package
- Remove xacro dependency, use pre-generated URDF files
- Merge `display.left.py` and `display.right.py` into `display.launch.py`
- Update submodule branch from master to main

### Removed

- `joint_prefix` parameter - joint names no longer have prefix
- Internal `wujihand_description` package
- `wujihand_rviz.launch.py` and `wujihand_foxglove.launch.py`

### Fixed

- robot_state_publisher TF - joint names now match URDF without prefix
- RViz config uses relative topic path for namespace compatibility
- Use SensorDataQoS for joint_commands subscription to support high-frequency control
- Improve wave_demo.py with dedicated thread for consistent 100Hz publishing rate
- Upgrade wujihandcpp SDK to v1.5.0
- Fix RViz display and parameter issues
- Force serial_number to string type for ROS2 Kilted compatibility
- Submodule URL update for branch compatibility

## [0.1.0] - 2025-12-19

### Added

- ROS2 driver for Wuji Hand with wujihandcpp SDK 1.4.0
- Multi-hand namespace support with XACRO for running multiple hands simultaneously
- Auto-detect handedness from hardware (0=right, 1=left)
- Separate xacro files for left and right hand models
- ROS logging API integration and error handling in launch files
- Common utility module (spawn_robot_state_publisher)
- CI build status badge in README

[Unreleased]: https://github.com/wuji-technology/wujihandros2/compare/v1.0.1...HEAD
[1.0.1]: https://github.com/wuji-technology/wujihandros2/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/wuji-technology/wujihandros2/releases/tag/v1.0.0
[0.1.0]: https://github.com/wuji-technology/wujihandros2/releases/tag/v0.1.0
