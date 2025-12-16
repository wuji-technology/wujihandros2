# WujiHand ROS2

WujiHand 灵巧手 ROS2 驱动包。

## 方式一：源码编译

### 1. 环境准备

```bash
# 安装 ROS2 (以 Kilted 为例)
sudo apt install ros-kilted-ros-base ros-kilted-robot-state-publisher ros-kilted-rviz2

# 安装 wujihandcpp SDK
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb
```

### 2. 编译

```bash
cd wujihand_ros2

# 配置环境
source /opt/ros/kilted/setup.bash

# 编译
colcon build

# 加载编译结果
source install/setup.bash
```

### 3. 测试驱动

```bash
# 启动驱动
ros2 launch wujihand_bringup wujihand.launch.py

# 新终端验证
source /opt/ros/kilted/setup.bash
source install/setup.bash
ros2 topic echo /joint_states --once
```

启动成功后会看到：
```
[wujihand_driver]: Connected to WujiHand (right)
[wujihand_driver]: WujiHand driver started (state: 1000.0 Hz, diagnostics: 10.0 Hz)
```

### 4. 测试脚本

```bash
# 运行波浪演示
ros2 run wujihand_bringup wave_demo.py
```

---

## 方式二：Deb 包安装

### 1. 安装依赖

```bash
# 安装 ROS2
sudo apt install ros-kilted-ros-base ros-kilted-robot-state-publisher

# 安装 wujihandcpp SDK
sudo dpkg -i wujihandcpp-1.3.0-amd64.deb
```

### 2. 安装驱动包

```bash
sudo dpkg -i ros-kilted-wujihand-ros2_0.1.0_amd64.deb
```

### 3. 测试驱动

```bash
# 配置环境
source /opt/ros/kilted/setup.bash

# 启动驱动
ros2 launch wujihand_bringup wujihand.launch.py

# 新终端验证
source /opt/ros/kilted/setup.bash
ros2 topic echo /joint_states --once
```

### 4. 测试脚本

```bash
ros2 run wujihand_bringup wave_demo.py
```

---

## 启动方式

| 命令 | 说明 |
|-----|------|
| `ros2 launch wujihand_bringup wujihand.launch.py` | 仅驱动 |
| `ros2 launch wujihand_bringup wujihand_rviz.launch.py` | 驱动 + RViz |
| `ros2 launch wujihand_bringup wujihand_foxglove.launch.py` | 驱动 + Foxglove |

## 基础使用

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
