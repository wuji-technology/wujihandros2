# wujihandros2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wujihandros2)](https://github.com/wuji-technology/wujihandros2/releases)

ROS2 driver package for Wuji Hand dexterous hand. Provides 1000Hz joint state publishing, real-time control interface, multi-hand setup, and RViz visualization.

**Get started with [Quick Start](#quick-start). For detailed documentation, please refer to [ROS2 Tutorial](https://docs.wuji.tech/docs/en/wuji-hand/latest/ros2-user-guide/index) on Wuji Docs Center.**

| ROS2 Version | Ubuntu | Build Status | Deb Package |
|:------------:|:------:|:------------:|:-----------:|
| Humble | 22.04 | [![CI](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml) | [Download](https://github.com/wuji-technology/wujihandros2/releases) |
| Kilted | 24.04 | [![CI](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml) | [Download](https://github.com/wuji-technology/wujihandros2/releases) |

## Repository Structure

```text
├── wujihand_bringup/
│   ├── launch/
│   ├── rviz/
│   └── scripts/
├── external/
│   └── wuji-hand-description/    # git submodule
├── wujihand_driver/                # joint controller driver
│   ├── include/
│   └── src/
├── wujihand_msgs/                  # joint controller messages
│   ├── msg/
│   └── srv/
├── wujihand_tactile_driver/        # tactile sensor driver
│   ├── include/
│   └── src/
├── wujihand_tactile_msgs/          # tactile sensor messages
│   ├── msg/
│   └── srv/
├── docs/
└── README.md
```

### Directory Description

| Directory | Description |
|-----------|-------------|
| `wujihand_bringup/` | Launch files, RViz config, and demo scripts |
| `external/wuji-hand-description/` | URDF models, mesh files, and base RViz configuration (submodule) |
| `wujihand_driver/` | Joint controller ROS 2 driver node |
| `wujihand_msgs/` | Joint controller messages and services |
| `wujihand_tactile_driver/` | Tactile sensor ROS 2 driver node |
| `wujihand_tactile_msgs/` | Tactile sensor messages and services |
| `docs/` | API reference and documentation |

## Quick Start

### Installation

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wujihandros2.git
cd wujihandros2
# If already cloned without --recurse-submodules, run:
# git submodule update --init --recursive
source /opt/ros/humble/setup.bash  # or kilted

# Both wujihand_driver and wujihand_tactile_driver link against the
# wujihandcpp SDK via find_package(wujihandcpp CONFIG REQUIRED). Build
# and install it once into a prefix, then point colcon at that prefix:
git clone https://github.com/wuji-technology/wujihandpy.git ../wujihandpy
cmake -S ../wujihandpy/wujihandcpp -B ../wujihandpy/wujihandcpp/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_STATIC_WUJIHANDCPP=ON \
    -DWUJIHANDCPP_INSTALL=ON \
    -DCMAKE_INSTALL_PREFIX=$PWD/../wujihandpy/install
cmake --build ../wujihandpy/wujihandcpp/build -j"$(nproc)"
cmake --install ../wujihandpy/wujihandcpp/build

colcon build --cmake-args -DCMAKE_PREFIX_PATH=$PWD/../wujihandpy/install
source install/setup.bash
```

### Running

```bash
# Joint driver only
ros2 launch wujihand_bringup wujihand.launch.py
ros2 launch wujihand_bringup wujihand.launch.py rviz:=true

# Joint + tactile + visualization (auto-discover both boards by USB PID)
ros2 launch wujihand_bringup wujihand_full.launch.py

# Tactile driver only
ros2 launch wujihand_bringup tactile.launch.py
ros2 launch wujihand_bringup tactile.launch.py serial_number:=<SN>

# Disable tactile in the full launch
ros2 launch wujihand_bringup wujihand_full.launch.py tactile:=false

# Verify operation
ros2 topic echo /hand_0/joint_states --once
ros2 topic echo /hand_0/tactile/diagnostics --once
ros2 topic hz /hand_0/tactile/raw                      # default 120 Hz
ros2 topic hz /hand_0/tactile/image                    # default 30 Hz
```

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
