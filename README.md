# WujiHand ROS2

[English](README_EN.md) | **中文**

WujiHand 灵巧手 ROS2 驱动包，提供高频关节状态发布（1000Hz）和实时控制接口。

## 系统要求

| 组件 | 最低版本 | 说明 |
|:-----|:--------|:-----|
| wujihandcpp SDK | 1.4.0 | C++ SDK |
| 固件版本 | 1.1.0 | 支持 TPDO 主动上报 |

## 构建状态

| ROS2 版本 | Ubuntu | 构建状态 | Deb 包 |
|:---------:|:------:|:--------:|:------:|
| Humble | 22.04 | [![CI](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) | [下载](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) |
| Kilted | 24.04 | [![CI](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) | [下载](https://github.com/Wuji-Technology-Co-Ltd/wujihandros2/actions/workflows/ci.yml?query=branch%3Amaster) |

## 安装

### 方式一：源码编译

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# 安装 ROS2 Humble 及依赖
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-sensor-msgs ros-humble-std-msgs \
    ros-humble-xacro ros-humble-foxglove-bridge \
    python3-colcon-common-extensions python3-rosdep

# 安装 wujihandcpp SDK
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo dpkg -i wujihandcpp-1.4.0-amd64.deb
```

</details>

<details>
<summary><b>Ubuntu 24.04 (Kilted)</b></summary>

```bash
# 安装 ROS2 Kilted 及依赖
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-robot-state-publisher \
    ros-kilted-rviz2 ros-kilted-sensor-msgs ros-kilted-std-msgs \
    ros-kilted-xacro ros-kilted-foxglove-bridge \
    python3-colcon-common-extensions python3-rosdep

# 安装 wujihandcpp SDK
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo dpkg -i wujihandcpp-1.4.0-amd64.deb
```

</details>

**编译与运行：**

```bash
cd wujihandros2
source env.sh          # 加载环境（自动检测 ROS2 版本）
colcon build           # 编译
source env.sh          # 重新加载环境
```

### 方式二：Deb 包安装

<details>
<summary><b>Ubuntu 22.04 (Humble)</b></summary>

```bash
# 安装依赖
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher \
    ros-humble-sensor-msgs ros-humble-std-msgs

# 安装 SDK 和驱动
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo dpkg -i wujihandcpp-1.4.0-amd64.deb
sudo dpkg -i ros-humble-wujihand_0.1.0_amd64.deb
```

</details>

<details>
<summary><b>Ubuntu 24.04 (Kilted)</b></summary>

```bash
# 安装依赖
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-robot-state-publisher \
    ros-kilted-sensor-msgs ros-kilted-std-msgs

# 安装 SDK 和驱动
wget https://github.com/Wuji-Technology-Co-Ltd/wujihandpy/releases/download/v1.4.0/wujihandcpp-1.4.0-amd64.deb
sudo dpkg -i wujihandcpp-1.4.0-amd64.deb
sudo dpkg -i ros-kilted-wujihand_0.1.0_amd64.deb
```

</details>

## 快速开始

### 启动驱动

```bash
# 终端 1: 启动驱动
cd wujihandros2
source env.sh
ros2 launch wujihand_bringup wujihand.launch.py
```

启动成功后会看到：
```
[wujihand_driver]: Connected to WujiHand (right)
[wujihand_driver]: WujiHand driver started (state: 1000.0 Hz, diagnostics: 10.0 Hz)
```

### 验证运行

```bash
# 终端 2: 查看关节状态
cd wujihandros2
source env.sh
ros2 topic echo /joint_states --once
```

### 运行演示

```bash
# 波浪演示：五指依次弯曲伸展
ros2 run wujihand_bringup wave_demo.py
```

## 启动方式

| 命令 | 说明 |
|:-----|:-----|
| `ros2 launch wujihand_bringup wujihand.launch.py` | 仅驱动 |
| `ros2 launch wujihand_bringup wujihand_rviz.launch.py` | 驱动 + RViz 可视化 |
| `ros2 launch wujihand_bringup wujihand_foxglove.launch.py` | 驱动 + Foxglove/Lichtblick |

> **提示**: Foxglove launch 同时支持 [Foxglove Studio](https://foxglove.dev/) 和 [Lichtblick](https://github.com/Lichtblick-Suite/lichtblick)，连接地址为 `ws://localhost:8765`

## 基础使用

### 控制命令

```bash
# 所有关节归零
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}" --once
```

### 状态查看

```bash
ros2 topic echo /joint_states       # 关节状态 (1000Hz)
ros2 topic echo /hand_diagnostics   # 诊断信息 (10Hz)
```

### 服务调用

```bash
# 使能/失能手指
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled "{finger_id: 0, enabled: true}"

# 重置错误
ros2 service call /reset_error wujihand_msgs/srv/ResetError "{finger_id: 0}"
```

## 文档

- [API 参考](docs/API.md) - Topics、Services、Parameters、错误处理
