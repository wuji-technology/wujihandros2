# WujiHand ROS2

**English** | [中文](README_CN.md)

ROS2 driver package for WujiHand dexterous hand, providing high-frequency joint state publishing (1000Hz) and real-time control interface.

## Requirements

| Component | Minimum Version | Description |
|:----------|:----------------|:------------|
| wujihandcpp SDK | 1.4.0 | C++ SDK |
| Firmware | 1.1.0 | TPDO proactive reporting support |

## Build Status

| ROS2 Version | Ubuntu | Build Status | Deb Package |
|:------------:|:------:|:------------:|:-----------:|
| Humble | 22.04 | [![CI](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) | [Download](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/releases) |
| Kilted | 24.04 | [![CI](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) | [Download](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/releases) |

## Installation

### Option 1: Build from Source

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# Install ROS2 Humble and dependencies
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-sensor-msgs ros-humble-std-msgs \
    ros-humble-xacro ros-humble-foxglove-bridge \
    python3-colcon-common-extensions python3-rosdep

# Install wujihandcpp SDK
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo apt install ./wujihandcpp-1.4.0-amd64.deb
```

</details>

<details>
<summary><b>Ubuntu 24.04 (Kilted)</b></summary>

```bash
# Install ROS2 Kilted and dependencies
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-robot-state-publisher \
    ros-kilted-rviz2 ros-kilted-sensor-msgs ros-kilted-std-msgs \
    ros-kilted-xacro ros-kilted-foxglove-bridge \
    python3-colcon-common-extensions python3-rosdep

# Install wujihandcpp SDK
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo apt install ./wujihandcpp-1.4.0-amd64.deb
```

</details>

**Build and Run:**

```bash
cd wujihandros2

# Humble
source /opt/ros/humble/setup.bash

# Or Kilted
source /opt/ros/kilted/setup.bash

# Build
colcon build

# Source workspace
source install/setup.bash
```

### Option 2: Deb Package Installation

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# Install dependencies
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-sensor-msgs ros-humble-std-msgs

# Install wujihandcpp SDK
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo apt install ./wujihandcpp-1.4.0-amd64.deb

# Install driver (download from releases page)
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/releases/download/v0.1.0/ros-humble-wujihand_0.1.0_amd64.deb
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

# Install wujihandcpp SDK
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo apt install ./wujihandcpp-1.4.0-amd64.deb

# Install driver (download from releases page)
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/releases/download/v0.1.0/ros-kilted-wujihand_0.1.0_amd64.deb
sudo apt install ./ros-kilted-wujihand_0.1.0_amd64.deb
```

</details>

## Quick Start

### Launch Driver

```bash
# Terminal 1: Launch driver
cd wujihandros2
source /opt/ros/kilted/setup.bash  # or humble
source install/setup.bash
ros2 launch wujihand_bringup wujihand.launch.py
```

On successful launch, you will see:
```
[wujihand_driver]: Connected to WujiHand (right)
[wujihand_driver]: WujiHand driver started (state: 1000.0 Hz, diagnostics: 10.0 Hz)
```

### Verify Operation

```bash
# Terminal 2: Check joint states
source /opt/ros/kilted/setup.bash  # or humble
source install/setup.bash
ros2 topic echo /hand_0/joint_states --once
```

### Run Demo

```bash
# Wave demo: fingers bend and extend sequentially
ros2 run wujihand_bringup wave_demo.py
```

## Launch Options

| Command | Description |
|:--------|:------------|
| `ros2 launch wujihand_bringup wujihand.launch.py` | Driver only |
| `ros2 launch wujihand_bringup wujihand_rviz.launch.py` | Driver + RViz visualization |
| `ros2 launch wujihand_bringup wujihand_foxglove.launch.py` | Driver + Foxglove/Lichtblick |

### Launch Parameters

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `hand_name` | `hand_0` | Hand name, used as namespace and TF prefix |
| `serial_number` | `""` | Device serial number (empty for auto-detect) |
| `publish_rate` | `1000.0` | Joint state publish rate in Hz |
| `filter_cutoff_freq` | `10.0` | Low-pass filter cutoff frequency in Hz |
| `diagnostics_rate` | `10.0` | Diagnostics publish rate in Hz |

> **Note**: Left/right hand type is auto-detected from hardware, no manual specification needed.

### Single Hand Examples

```bash
# Default namespace "hand_0"
ros2 launch wujihand_bringup wujihand_foxglove.launch.py

# Custom namespace
ros2 launch wujihand_bringup wujihand_foxglove.launch.py hand_name:=my_hand
```

### Multi-Hand Setup

```bash
# Launch left hand (distinguished by serial number)
ros2 launch wujihand_bringup wujihand_foxglove.launch.py \
    hand_name:=left_hand serial_number:=ABC123 &

# Launch right hand
ros2 launch wujihand_bringup wujihand_foxglove.launch.py \
    hand_name:=right_hand serial_number:=DEF456 &
```

Topic structure:
```
/left_hand/joint_states
/left_hand/joint_commands
/left_hand/hand_diagnostics
/left_hand/robot_description

/right_hand/joint_states
/right_hand/joint_commands
/right_hand/hand_diagnostics
/right_hand/robot_description
```

TF frames (with prefix):
```
left_hand/palm_link
left_hand/finger1_link1
...
right_hand/palm_link
right_hand/finger1_link1
...
```

> **Note**: Foxglove launch supports both [Foxglove Studio](https://foxglove.dev/) and [Lichtblick](https://github.com/Lichtblick-Suite/lichtblick), connect to `ws://localhost:8765`

## Basic Usage

### Control Commands

```bash
# Move all joints to zero position
ros2 topic pub /hand_0/joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}" --once
```

### View Status

```bash
ros2 topic echo /hand_0/joint_states       # Joint states (1000Hz)
ros2 topic echo /hand_0/hand_diagnostics   # Diagnostics (10Hz)
```

### Service Calls

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

## Documentation

- [API Reference](docs/API.md) - Topics, Services, Parameters, Error Handling
