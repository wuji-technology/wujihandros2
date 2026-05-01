# Round-8 结构重构方案 — wujihandros2 视角

**Authoritative doc**: `wujihandpy-tactile-ros2/docs/refactor-plan.md`
(rename / import maps and 15-phase table live there).
**This file**: ROS-specific package split, launch contract, and
CI/build_deb/package.xml deltas.

**Status**: design frozen — execution in progress on `feat/tactile-ros2`
**Heads when frozen**: wujihandpy `05ad20a`, wujihandros2 `e2cd22c`

---

## 1. ROS package map

| Old | New |
|---|---|
| `wujihand_msgs/msg/TactileFrame.msg` | `wujihand_tactile_msgs/msg/TactileFrame.msg` |
| `wujihand_msgs/msg/TactileDiagnostics.msg` | `wujihand_tactile_msgs/msg/TactileDiagnostics.msg` |
| `wujihand_msgs/srv/SetTactileStreaming.srv` | `wujihand_tactile_msgs/srv/SetTactileStreaming.srv` |
| `wujihand_msgs/srv/SetTactileSampleRate.srv` | `wujihand_tactile_msgs/srv/SetTactileSampleRate.srv` |
| `wujihand_msgs/srv/ResetTactileCounters.srv` | `wujihand_tactile_msgs/srv/ResetTactileCounters.srv` |
| `wujihand_driver/src/tactile_driver_node.cpp` | `wujihand_tactile_driver/src/tactile_driver_node.cpp` |
| `wujihand_driver/src/tactile_driver_main.cpp` | `wujihand_tactile_driver/src/main.cpp` |
| `wujihand_driver/include/wujihand_driver/tactile_driver_node.hpp` | `wujihand_tactile_driver/include/wujihand_tactile_driver/tactile_driver_node.hpp` |
| `wujihand_driver/include/wujihand_driver/colormap.hpp` | `wujihand_tactile_driver/include/wujihand_tactile_driver/colormap.hpp` |

After the split, `wujihand_msgs` and `wujihand_driver` only own
joint-control surface; the unaffiliated `wujihand_bringup` continues to
host both launches.

### Joint-side cleanup (Phase 6c)

| Old | New |
|---|---|
| `wujihand_driver/src/main.cpp` | `wujihand_driver/src/wujihand_driver_main.cpp` |

Naming becomes symmetric inside `wujihand_driver`:
`wujihand_driver_main.cpp` ↔ `wujihand_driver_node.cpp`. The same
pattern lives inside the new `wujihand_tactile_driver`:
`main.cpp` ↔ `tactile_driver_node.cpp` (no prefix needed because the
package is single-purpose).

### Namespace inside the new tactile package

C++ namespace inside `wujihand_tactile_driver` is `wujihand_tactile_driver`
(matches the package name, mirrors how `wujihand_driver::WujiHandDriverNode`
uses its own package name).

---

## 2. Downstream consumer impact (breaking)

| Consumer | Change required |
|---|---|
| `find_package(wujihand_msgs)` users that referenced tactile types | switch to `find_package(wujihand_tactile_msgs)` |
| Launch files referencing `Node(package="wujihand_driver", executable="tactile_driver_node")` | change to `package="wujihand_tactile_driver"`, executable still `tactile_driver_node` |
| Anyone running `ros2 service call /set_tactile_streaming ...` via legacy global path | services were already moved under `tactile/` prefix in round 5; no change here |
| Debian users installing `ros-humble-wujihand-driver` who got the tactile binary by side-effect | now also need `apt install ros-humble-wujihand-tactile-driver` |

No alias / shim / compatibility layer. PR description must call out the
break explicitly.

---

## 3. Launch contract normalization (Phase 7.0)

Before Phase 7 can compose launches via `IncludeLaunchDescription`, the
existing `tactile.launch.py` arg surface must be fully explicit.

### Args that must be declared (or arg-ized from hardcoded values)

| Arg | Today | After Phase 7.0 |
|---|---|---|
| `parent_frame` | hardcoded `palm_link` | declared, default `palm_link`; full launch overrides per-hand to `{left,right}_palm_link` |
| `serial_number` | declared | declared (no change) |
| `namespace` | not declared (uses node default) | declared, default `""` |
| `rviz` | not declared (RViz spawned unconditionally where present) | declared, default `false` |
| `tactile_active` | `SetLaunchConfiguration` written but not declared | declared, default mirrors whether tactile board was discovered |
| `image_rate` / `sample_rate_hz` / `streaming_at_startup` | declared | declared (no change) |

### Static-TF wiring

Currently the static-TF publisher in `tactile.launch.py` parents the
`tactile_sensor_link` to `palm_link`. In `wujihand_full.launch.py` the
joint URDF's palm link is namespaced — `left_palm_link` or
`right_palm_link`. After Phase 7.0 the parent is whatever
`parent_frame` arg is set to, removing the implicit assumption.

---

## 4. RViz overlay (Phase 8)

Today:
```
wujihand_bringup/rviz/
  left_tactile.rviz       ← copy of left.rviz + Image display + tactile-specific tweaks
  right_tactile.rviz      ← same shape, mirror
external/wuji-hand-description/rviz/
  left.rviz               ← base joint-only config (submodule)
  right.rviz
```

After:
```
wujihand_bringup/rviz/
  tactile_overlay.json    ← only the Image display + supporting Group/View entries
  (left/right_tactile.rviz deleted)
```

Launch-time composition (`wujihand_bringup/launch/common.py`):

1. Read `wuji_hand_description/rviz/{left|right}.rviz` based on
   `hand_type`.
2. Read `wujihand_bringup/rviz/tactile_overlay.json`.
3. Merge under `Visualization Manager.Displays` (RViz config is YAML
   — using stdlib `json` plus a tiny YAML serializer is OK; do NOT
   pull `PyYAML` as a runtime dep).
4. Write to `tempfile.NamedTemporaryFile(suffix='.rviz', delete=False)`.
5. Pass tempfile path to `Node(package='rviz2', executable='rviz2',
   arguments=['-d', path])`.
6. Register `atexit` (or launch `OnShutdown`) cleanup to delete the
   tempfile.

### Constraint — no new runtime deps

`wujihand_bringup/package.xml` may not gain a `python3-yaml` dep.
RViz config is load-as-JSON-style nested dicts; minimal hand-written
serializer suffices.

---

## 5. CI / build_deb / package.xml deltas (Phase 6.0)

Tracked because they are easy to forget and break the rest of the chain.

### CI workflow change

```yaml
# Old (sketch):
- run: sudo apt install ros-humble-wujihandcpp

# New (sketch):
- uses: actions/checkout@v4
  with:
    repository: wuji-technology/wujihandpy
    ref: ${{ github.event.pull_request.head.ref }}  # or pin
    path: wujihandpy
- run: |
    cd wujihandpy
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_STATIC_WUJIHANDCPP=ON
    cmake --build build -j$(nproc)
    cmake --install build --prefix $PWD/install
- run: |
    export CMAKE_PREFIX_PATH=$GITHUB_WORKSPACE/wujihandpy/install
    colcon build --symlink-install
```

Pinning vs auto-tracking the SDK ref is a separate decision (raise as a
Phase 6.0 commit message question if unclear at that point).

### `build_deb.sh` deps

Currently lists deb deps for the monolithic `wujihand-driver`. Split:

- `ros-humble-wujihand-msgs` Depends: unchanged
- `ros-humble-wujihand-driver` Depends: drops tactile transitive
- `ros-humble-wujihand-tactile-msgs` (new): Depends on `wujihandcpp`,
  `ros-humble-std-msgs`
- `ros-humble-wujihand-tactile-driver` (new): Depends on
  `wujihand-tactile-msgs`, `wujihandcpp`, `ros-humble-rclcpp`,
  `ros-humble-sensor-msgs`

### `package.xml` runtime deps audit

`wujihand_bringup/package.xml` is missing:

- `<exec_depend>tf2_ros</exec_depend>` (used by `wujihand_full.launch.py`
  for static_transform_publisher)
- `<exec_depend>ament_index_python</exec_depend>` (used by
  `common.py` / `wujihand_full.launch.py` for
  `get_package_share_directory`)
- After Phase 6: `<exec_depend>wujihand_tactile_driver</exec_depend>`

---

## 6. ROS HIL test commit (Phase 9)

Move `/tmp/run_driver_test.sh` (currently lives only on dev workstation)
to `tests/hil_driver.sh` in the repo. Pattern matches SDK side: env
gate + README documenting hardware prerequisite.

```bash
#!/bin/bash
# Requires: tactile board flashed with firmware PR #30, ROS workspace built.
# Run: WUJIHAND_HIL=1 bash tests/hil_driver.sh
[[ "$WUJIHAND_HIL" == "1" ]] || { echo "skip: set WUJIHAND_HIL=1"; exit 0; }
# ... existing script body ...
```

---

## 7. Phase ordering (this repo's view)

Phases 0–5 happen entirely in `wujihandpy`. This repo's first action is
**Phase 6.0** (CI adapt) which must precede 6a/b/c. After Phase 6c the
repo is in a buildable state with both packages renamed; Phase 7.0 then
normalizes launch contracts; 7 composes; 8 cleans RViz; 9a syncs docs;
9 commits the HIL script and refreshes the PR body.

For the cross-repo dependency on namespace rename: this repo cannot
build between Phase 1 (SDK rename committed) and Phase 6b (this repo
adopts new names) without a broken intermediate state. Two options:

a. Land Phase 1–5 (SDK) first, push, then start this repo's Phase 6.0.
b. Land all SDK + ROS phases on the respective branches in lockstep,
   coordinate merge order at the end.

Going with (a). Each SDK phase is self-contained on its branch; this
repo's `feat/tactile-ros2` only catches up at Phase 6.0+.

---

## 8. What this doc does NOT change

- `wujihand_bringup` itself stays a single package (it owns launches and
  RViz config for the whole hand assembly — joint + tactile is a
  natural composition). It just gets new `<exec_depend>` entries.
- The Apache 2.0 license header / clang-format / `.editorconfig`
  conventions established in round 7 stay in force; new files in the
  new packages adopt them from day one.
- `wujihand_msgs::HandDiagnostics` and other joint surfaces stay where
  they are.
