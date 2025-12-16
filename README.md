# WujiHand ROS2

WujiHand 灵巧手 ROS2 驱动包。

## 30秒快速开始

```bash
# 1. 配置环境
source /opt/ros/kilted/setup.bash

# 2. 启动驱动
ros2 launch wujihand_bringup wujihand.launch.py

# 3. 验证 (新终端)
ros2 topic echo /joint_states --once
```

启动成功后会看到：
```
[wujihand_driver]: Connected to WujiHand (right)
[wujihand_driver]: WujiHand driver started (state: 1000.0 Hz, diagnostics: 10.0 Hz)
```

## 安装

### Deb 包安装 (推荐)

```bash
# 1. 安装 wujihandcpp SDK
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb

# 2. 安装 wujihand-ros2
sudo dpkg -i ros-kilted-wujihand_0.1.0-1_amd64.deb
```

### 源码编译

```bash
# 克隆仓库
git clone https://github.com/user/wujihand-ros2.git
cd wujihand-ros2

# 编译
source /opt/ros/kilted/setup.bash
colcon build

# 使用
source install/setup.bash
```

## 基础使用

### 启动方式

| 命令 | 说明 |
|-----|------|
| `ros2 launch wujihand_bringup wujihand.launch.py` | 仅驱动 |
| `ros2 launch wujihand_bringup wujihand_rviz.launch.py` | 驱动 + RViz |
| `ros2 launch wujihand_bringup wujihand_foxglove.launch.py` | 驱动 + Foxglove |

### 发送控制命令

```bash
# 所有关节归零
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}" --once

# 握拳
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{position: [0,1.5,1.5,1.5, 0,1.5,1.5,1.5, 0,1.5,1.5,1.5, 0,1.5,1.5,1.5, 0,1.5,1.5,1.5]}" --once
```

### 查看状态

```bash
# 关节状态 (1000Hz)
ros2 topic echo /joint_states

# 诊断信息 (10Hz)
ros2 topic echo /hand_diagnostics
```

### 演示程序

```bash
ros2 run wujihand_bringup wave_demo.py
```

### 服务调用

```bash
# 使能/失能手指
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled "{finger_id: 0, enabled: true}"

# 重置错误
ros2 service call /reset_error wujihand_msgs/srv/ResetError "{finger_id: 0}"
```

## 支持的平台

| Ubuntu | ROS2 版本 |
|--------|----------|
| 22.04 | Humble |
| 24.04 | Jazzy |
| 24.04 | Kilted |

## 更多文档

- [API 参考](docs/API.md) - Topics、Services、Parameters、错误处理
