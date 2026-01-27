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
│   └── scripts/
├── external/
│   └── wuji-hand-description/    # git submodule
├── wujihand_driver/
│   ├── include/
│   └── src/
├── wujihand_msgs/
│   ├── msg/
│   └── srv/
├── docs/
└── README.md
```

### Directory Description

| Directory | Description |
|-----------|-------------|
| `wujihand_bringup/` | Launch files and demo scripts for starting the driver |
| `external/wuji-hand-description/` | URDF models, mesh files, and RViz configuration (submodule) |
| `wujihand_driver/` | Core ROS2 driver node for hardware communication |
| `wujihand_msgs/` | Custom ROS2 message and service definitions |
| `docs/` | API reference and documentation |

## Quick Start

### Installation

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wujihandros2.git
cd wujihandros2
# If already cloned without --recurse-submodules, run:
# git submodule update --init --recursive
source /opt/ros/humble/setup.bash  # or kilted
colcon build
source install/setup.bash
```

### Running

```bash
# Launch driver
ros2 launch wujihand_bringup wujihand.launch.py

# Launch with RViz visualization
ros2 launch wujihand_bringup wujihand.launch.py rviz:=true

# Verify operation
ros2 topic echo /hand_0/joint_states --once
```

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
