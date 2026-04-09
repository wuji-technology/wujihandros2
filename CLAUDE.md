# WujiHand ROS2 Workspace

> 优先参考全局配置：`~/.claude/CLAUDE.md`（Git 规范、Commit 格式等通用约定）

## 项目概述

WujiHand 灵巧手 ROS2 驱动和工具包。包含关节控制驱动、触觉传感器驱动、URDF 模型、可视化启动脚本。

## 常用命令

```bash
# 构建全部
source /opt/ros/humble/setup.bash
export WUJIHANDPY_SOURCE_DIR=/path/to/wujihandpy   # 必须设置
colcon build --symlink-install

# 构建单个包
colcon build --packages-select wujihand_bringup --symlink-install

# 一键启动（灵巧手 + 触觉 + 可视化）
source install/setup.bash
ros2 launch wujihand_bringup wujihand_full.launch.py

# 单独启动
ros2 launch wujihand_bringup wujihand.launch.py serial_number:=<SN> rviz:=true
ros2 launch wujihand_bringup tactile.launch.py serial_number:=<SN>
```

## 包结构

| 包名 | 用途 |
|------|------|
| wujihand_msgs | 自定义消息（TactileFrame 等） |
| wujihand_driver | C++ 驱动节点（关节 + 触觉） |
| wujihand_bringup | Launch 文件、RViz 配置、示例脚本 |
| wuji_hand_description | URDF 模型、mesh（git submodule） |

## 构建依赖

**wujihand_driver 依赖 wujihandcpp 库。** 必须设置环境变量指向 wujihandpy 源码目录：

```bash
export WUJIHANDPY_SOURCE_DIR=/home/misty/work/worktrees/tactile-ros2/wujihandpy-tactile-ros2
```

不设置会报错 `wujihandcpp not found`。wujihandpy 需要先 `cmake --build build` 编译出 `libwujihandcpp.a`。

## git submodule

`external/wuji-hand-description` 是 submodule。clone 后必须初始化：

```bash
git submodule update --init --recursive
```

不初始化会导致 `wuji_hand_description` 包不存在，launch 报错 `package not found`。

## USB 设备识别

| VID:PID | 设备 | 用途 |
|---------|------|------|
| 0483:2000 | WUJIHAND (sboard) | 灵巧手关节控制 |
| 0483:5700 | WujiHand G-Board | 触觉传感器 |
| 0483:5701 | WujiHand TBoard Bootloader | 触觉板 OTA 模式 |

`wujihand_full.launch.py` 通过扫描 `/sys/bus/usb/devices` 自动按 PID 区分设备，不需要手动填 serial number。

## Topic 命名

```
/hand_0/joint_states              # sensor_msgs/JointState (1000Hz)
/hand_0/joint_commands            # 关节控制
/hand_0/diagnostics               # 诊断信息 (10Hz)
/hand_0/tactile/raw               # TactileFrame (120Hz, Best Effort)
/hand_0/tactile/image             # sensor_msgs/Image (30Hz, Reliable)
/hand_0/robot_description         # URDF
/tf, /tf_static                   # TF 树
```

## 踩坑记录

### 1. RViz2 Image display 不支持 Best Effort QoS (rviz2 Humble bug)

**现象：** RViz config 文件里写了 `Reliability Policy: Best Effort`，但 Image display 始终用 Reliable 订阅，导致 `incompatible QoS` 警告且不显示图像。

**教训：** RViz2 Humble 版的 Image display plugin 忽略 config 文件中的 QoS 设置。

**解决：** 把 image publisher 端改为 Reliable QoS（`create_publisher("topic", 10)` 而不是 `SensorDataQoS()`）。24x32 @ 30Hz 的小图用 Reliable 无性能问题。Raw 高频数据（120Hz）继续用 Best Effort。

### 2. 多 USB 设备同 VID 导致连接失败

**现象：** `wujihand_driver_node` 报错 `2 devices found with specified vendor id (0x0483)` 然后退出。

**教训：** 灵巧手和触觉板共用 VID 0x0483，driver 默认按 VID 搜索，找到多个设备时会失败。

**解决：** 启动时必须指定 `serial_number` 参数。`wujihand_full.launch.py` 通过 PID 自动区分并传入 serial number。

### 3. wujihand_bringup 不在 install 里

**现象：** `ros2 launch wujihand_bringup ... : package not found`。

**教训：** 首次 `colcon build` 可能只编译了 wujihand_driver 和 wujihand_msgs，bringup 被跳过。

**解决：** `colcon build --packages-select wujihand_bringup --symlink-install` 单独编译。

### 4. detect_handedness 重复调用导致启动很慢

**现象：** launch 启动后 30-60 秒才出现 RViz，终端持续输出 `failed to send response to get_parameters` 警告。

**教训：** `detect_handedness()` 通过 ROS2 参数服务轮询 driver 节点。每次调用最多等 15 秒。如果在 launch 里调用多次（URDF 一次、RViz 一次），总耗时翻倍。

**解决：** 在 `setup_viz_and_urdf()` 中只调用一次 `detect_handedness()`，缓存结果同时用于 URDF 和 RViz config 选择。

### 5. ROS2 daemon 在 WSL2 上经常卡死

**现象：** `ros2 topic list` 挂起不返回，报 `TimeoutError: Connection timed out`。

**教训：** WSL2 的网络栈对 localhost xmlrpc 连接有时不稳定。

**解决：** `ros2 daemon stop && ros2 daemon start`，或者加 `--no-daemon` 参数：`ros2 topic list --no-daemon`。

### 6. FastRTPS 在 WSL2 上的 DDS 发现问题

**现象：** 同一机器上的 ROS2 节点互相看不到 topic（`0 publishers`），但 launch 内部的节点通信正常。

**教训：** FastRTPS 依赖 multicast 做节点发现，WSL2 对 multicast 支持不完善。launch 内部节点走进程内通信（intra-process）所以不受影响，但跨进程（如独立启动的 subscriber）可能发现不了。

**解决：** 可以切换到 CycloneDDS：`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（所有进程必须用同一个 RMW）。或者在 launch 内部做所有事情，避免跨进程订阅。
