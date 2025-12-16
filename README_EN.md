# WujiHand ROS2

**English** | [中文](README.md)

ROS2 driver package for WujiHand dexterous hand, providing high-frequency joint state publishing (1000Hz) and real-time control interface.

## Build Status

| ROS2 Version | Ubuntu | Build Status | Deb Package |
|:------------:|:------:|:------------:|:-----------:|
| Humble | 22.04 | [![CI](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) | [Download](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) |
| Kilted | 24.04 | [![CI](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) | [Download](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) |

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
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.3.0/wujihandcpp-1.3.0-amd64.deb
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb
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
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.3.0/wujihandcpp-1.3.0-amd64.deb
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb
```

</details>

**Build and Run:**

```bash
cd wujihandros2
source env.sh          # Load environment (auto-detect ROS2 version)
colcon build           # Build
source env.sh          # Reload environment
```

### Option 2: Deb Package Installation

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# Install dependencies
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-sensor-msgs ros-humble-std-msgs

# Install SDK and driver
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.3.0/wujihandcpp-1.3.0-amd64.deb
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb
sudo dpkg -i ros-humble-wujihand-ros2_0.1.0_amd64.deb
```

</details>

<details>
<summary><b>Ubuntu 24.04 (Kilted)</b></summary>

```bash
# Install dependencies
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-robot-state-publisher \
    ros-kilted-sensor-msgs ros-kilted-std-msgs

# Install SDK and driver
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.3.0/wujihandcpp-1.3.0-amd64.deb
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb
sudo dpkg -i ros-kilted-wujihand-ros2_0.1.0_amd64.deb
```

</details>

## Quick Start

### Launch Driver

```bash
# Terminal 1: Launch driver
cd wujihandros2
source env.sh
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
cd wujihandros2
source env.sh
ros2 topic echo /joint_states --once
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

> **Note**: Foxglove launch supports both [Foxglove Studio](https://foxglove.dev/) and [Lichtblick](https://github.com/Lichtblick-Suite/lichtblick), connect to `ws://localhost:8765`

## Basic Usage

### Control Commands

```bash
# Move all joints to zero position
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}" --once
```

### View Status

```bash
ros2 topic echo /joint_states       # Joint states (1000Hz)
ros2 topic echo /hand_diagnostics   # Diagnostics (10Hz)
```

### Service Calls

```bash
# Enable/disable finger
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled "{finger_id: 0, enabled: true}"

# Reset error
ros2 service call /reset_error wujihand_msgs/srv/ResetError "{finger_id: 0}"
```

## Documentation

- [API Reference](docs/API.md) - Topics, Services, Parameters, Error Handling
