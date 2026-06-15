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
├── wujihand_bringup/            // Launch files and demo scripts for starting the driver
│   ├── launch/
│   └── scripts/
├── external/
│   └── wuji-description/        // URDF models, mesh files, and RViz configuration (submodule)
├── wujihand_driver/             // Core ROS2 driver node for hardware communication
│   ├── include/
│   └── src/
├── wujihand_msgs/               // Custom ROS2 message and service definitions
│   ├── msg/
│   └── srv/
├── docs/                        // API reference and documentation
└── README.md
```

## Quick Start

### Installation

The driver links against the **wujihandcpp** C++ SDK, which is distributed
separately (it is **not** a git submodule, so `git submodule update` does not
provide it). Install the SDK `.deb` **before** running `colcon build`.

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wujihandros2.git
cd wujihandros2
# If already cloned without --recurse-submodules, run:
# git submodule update --init --recursive

# Install the wujihandcpp C++ SDK from the wujihandpy releases.
# Fetch the latest amd64 .deb (use *-arm64.deb on ARM):
DEB_URL=$(curl -sH "Accept: application/vnd.github+json" \
  https://api.github.com/repos/wuji-technology/wujihandpy/releases | \
  grep -oP '"browser_download_url":\s*"\K[^"]*wujihandcpp[^"]*-amd64\.deb' | head -1)
wget -q "$DEB_URL"
sudo dpkg -i --force-overwrite wujihandcpp-*.deb
sudo apt-get install -f -y

source /opt/ros/humble/setup.bash  # or kilted
colcon build
source install/setup.bash
```

> The SDK can also be downloaded manually from the
> [wujihandpy releases page](https://github.com/wuji-technology/wujihandpy/releases).

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
