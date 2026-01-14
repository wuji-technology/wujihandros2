# wujihandros2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wujihandros2)](https://github.com/wuji-technology/wujihandros2/releases)

ROS2 driver package for Wuji Hand dexterous hand. Provides 1000Hz joint state publishing, real-time control interface, multi-hand setup, and RViz visualization.

| ROS2 Version | Ubuntu | Build Status | Deb Package |
|:------------:|:------:|:------------:|:-----------:|
| Humble | 22.04 | [![CI](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml) | [Download](https://github.com/wuji-technology/wujihandros2/releases) |
| Kilted | 24.04 | [![CI](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/wuji-technology/wujihandros2/actions/workflows/ci.yml) | [Download](https://github.com/wuji-technology/wujihandros2/releases) |

## Table of Contents

- [Repository Structure](#repository-structure)
- [Usage](#usage)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Running](#running)
    - [1. Quick Start](#1-quick-start)
    - [2. Launch Options](#2-launch-options)
    - [3. Basic Usage](#3-basic-usage)
- [Appendix](#appendix)
- [Contact](#contact)

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

## Usage

### Prerequisites

| Component | Minimum Version |
|:----------|:----------------|
| wujihandcpp | 1.5.0 |
| Firmware | 1.2.0 |

```bash
# Install wujihandcpp SDK
wget https://github.com/wuji-technology/wujihandpy/releases/download/v1.5.0/wujihandcpp-1.5.0-amd64.deb
sudo apt install ./wujihandcpp-1.5.0-amd64.deb
```

### Installation

#### Option 1: Build from Source

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# Install ROS2 Humble and dependencies
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-sensor-msgs ros-humble-std-msgs \
    python3-colcon-common-extensions python3-rosdep
```

</details>

<details>
<summary><b>Ubuntu 24.04 (Kilted)</b></summary>

```bash
# Install ROS2 Kilted and dependencies
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-robot-state-publisher \
    ros-kilted-rviz2 ros-kilted-sensor-msgs ros-kilted-std-msgs \
    python3-colcon-common-extensions python3-rosdep
```

</details>

**Build and Run:**

```bash
git clone https://github.com/wuji-technology/wujihandros2.git
cd wujihandros2
source /opt/ros/humble/setup.bash  # or kilted
colcon build
source install/setup.bash
```

#### Option 2: Deb Package

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# Install dependencies
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-sensor-msgs ros-humble-std-msgs

# Install driver
wget https://github.com/wuji-technology/wujihandros2/releases/download/v0.1.0/ros-humble-wujihand_0.1.0_amd64.deb
sudo apt install ./ros-humble-wujihand_0.1.0_amd64.deb
```

</details>

<details>
<summary><b>Ubuntu 24.04 (Kilted)</b></summary>

```bash
# Install dependencies
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-robot-state-publisher \
    ros-kilted-sensor-msgs ros-kilted-std-msgs

# Install driver
wget https://github.com/wuji-technology/wujihandros2/releases/download/v0.1.0/ros-kilted-wujihand_0.1.0_amd64.deb
sudo apt install ./ros-kilted-wujihand_0.1.0_amd64.deb
```

</details>

### Running

#### 1. Quick Start

**Launch Driver:**

```bash
cd wujihandros2
source /opt/ros/humble/setup.bash  # or kilted
source install/setup.bash
ros2 launch wujihand_bringup wujihand.launch.py
```

On successful launch:

```text
[wujihand_driver]: Connected to WujiHand (right)
[wujihand_driver]: WujiHand driver started (state: 1000.0 Hz, diagnostics: 10.0 Hz)
```

**Verify Operation:**

```bash
ros2 topic echo /hand_0/joint_states --once
```

**Run Demo:**

```bash
# Wave demo: fingers bend and extend sequentially
ros2 run wujihand_bringup wave_demo.py
```

#### 2. Launch Options

| Command | Description |
|:--------|:------------|
| `ros2 launch wujihand_bringup wujihand.launch.py` | Driver only |
| `ros2 launch wujihand_bringup wujihand.launch.py rviz:=true` | Driver + RViz visualization |

**Launch Parameters:**

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `hand_name` | `hand_0` | Namespace for topics and services |
| `serial_number` | `""` | Device serial number (empty for auto-detect) |
| `publish_rate` | `1000.0` | Joint state publish rate (Hz) |
| `filter_cutoff_freq` | `10.0` | Low-pass filter cutoff frequency (Hz) |
| `diagnostics_rate` | `10.0` | Diagnostics publish rate (Hz) |
| `rviz` | `false` | Launch RViz for visualization |

> **Note**: Left/right hand type is auto-detected from hardware, no manual specification needed.

**Single-Hand Examples:**

```bash
# Default namespace "hand_0"
ros2 launch wujihand_bringup wujihand.launch.py

# Custom namespace
ros2 launch wujihand_bringup wujihand.launch.py hand_name:=my_hand
```

**Multi-Hand Setup:**

```bash
# Launch left hand (distinguished by serial number)
ros2 launch wujihand_bringup wujihand.launch.py \
    hand_name:=left_hand serial_number:=ABC123 &

# Launch right hand
ros2 launch wujihand_bringup wujihand.launch.py \
    hand_name:=right_hand serial_number:=DEF456 &
```

Topic structure:

```
/left_hand/joint_states
/left_hand/joint_commands
/left_hand/hand_diagnostics
/left_hand/robot_description
/left_hand/tf
/left_hand/tf_static

/right_hand/joint_states
/right_hand/joint_commands
/right_hand/hand_diagnostics
/right_hand/robot_description
/right_hand/tf
/right_hand/tf_static
```

TF frames (no prefix, use RViz fixed frame "palm_link"):

```
palm_link
finger1_link1
...
```

#### 3. Basic Usage

**Control Commands:**

```bash
# Move all joints to zero position
ros2 topic pub /hand_0/joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}" --once
```

**View Status:**

```bash
ros2 topic echo /hand_0/joint_states       # Joint states (1000Hz)
ros2 topic echo /hand_0/hand_diagnostics   # Diagnostics (10Hz)
```

**Service Calls:**

```bash
# Disable all joints
ros2 service call /hand_0/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: false}"

# Enable all joints
ros2 service call /hand_0/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: true}"

# Enable single finger (e.g., index finger_id=1)
ros2 service call /hand_0/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 1, joint_id: 255, enabled: true}"

# Reset all errors
ros2 service call /hand_0/reset_error wujihand_msgs/srv/ResetError \
  "{finger_id: 255, joint_id: 255}"
```

**Parameter Reference:**

| Parameter | Value | Meaning |
|:----------|:------|:--------|
| `finger_id` | 0-4 | Thumb, Index, Middle, Ring, Little |
| `finger_id` | 255 | All fingers |
| `joint_id` | 0-3 | Individual joints |
| `joint_id` | 255 | All joints |

## Appendix

- **API Reference**: [docs/API.md](docs/API.md) - Topics, Services, Parameters, Error Handling

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
