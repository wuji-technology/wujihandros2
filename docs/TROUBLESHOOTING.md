# 故障排查指南

## 硬件连接问题

### ERROR_BUSY: 设备被占用

**错误信息**: `Failed to claim interface: -6 (ERROR_BUSY)`

**原因**: 其他进程正在占用 USB 设备。

**解决方法**:
```bash
# 查找占用进程
ps aux | grep wujihand

# 杀掉相关进程
pkill -f wujihand_driver_node
pkill -f robot_state_publisher
pkill -f "ros2 launch"

# 重新启动
ros2 launch wujihand_bringup wujihand.launch.py
```

### USB 连接检查

```bash
# 查看 USB 设备
lsusb

# 查看详细信息
dmesg | tail -20
```

### WSL2 USB 转发

在 WSL2 中使用需要转发 USB 设备：

```powershell
# Windows PowerShell (管理员)
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

---

## ROS2 通信问题

### ros2 topic list 卡住

**解决步骤**:
```bash
# 1. 检查 daemon 状态
ros2 daemon status

# 2. 重启 daemon
ros2 daemon stop
ros2 daemon start

# 3. 清理缓存 (如果上述无效)
rm -rf ~/.ros/ros2_daemon/
rm -rf /dev/shm/fastrtps*
ros2 daemon start
```

### 切换到 CycloneDDS

如果 FastDDS 不稳定，可以切换到 CycloneDDS：

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## RViz 显示问题

### No TF data / 模型不显示

**检查步骤**:

1. 确认 joint_states 正常发布：
   ```bash
   ros2 topic echo /joint_states --once
   ```

2. 确认 robot_state_publisher 运行：
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

3. 确认 Fixed Frame 设置为 `palm_link`

---

## 日志和调试

### 查看详细日志

```bash
# 设置日志级别
ros2 launch wujihand_bringup wujihand.launch.py --ros-args --log-level debug
```

### 查看话题数据

```bash
# 查看所有话题
ros2 topic list

# 查看话题详情
ros2 topic info /joint_states

# 查看话题数据
ros2 topic echo /joint_states

# 只看一条
ros2 topic echo /joint_states --once

# 查看发布频率
ros2 topic hz /joint_states
```

### 查看服务

```bash
# 列出所有服务
ros2 service list

# 查看服务类型
ros2 service type /set_enabled

# 调用服务
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: true}"
```

### 查看参数

```bash
# 列出节点参数
ros2 param list /wujihand_driver

# 获取参数值
ros2 param get /wujihand_driver handedness
```

---

## 错误码参考

| 错误码 | 含义 | 处理方法 |
|-------|------|---------|
| 0 | 正常 | - |
| 1 | 过温保护 | 等待冷却后重试 |
| 2 | 过流保护 | 检查负载，重置错误 |
| 3 | 通信超时 | 检查连接，重启驱动 |

**重置错误**:
```bash
ros2 service call /reset_error wujihand_msgs/srv/ResetError \
  "{finger_id: 255, joint_id: 255}"
```

---

## 常见问题 FAQ

### Q: 如何知道连接的是左手还是右手？
```bash
ros2 param get /wujihand_driver handedness
```

### Q: 如何降低 CPU 占用？
降低发布频率：
```bash
ros2 launch wujihand_bringup wujihand.launch.py publish_rate:=100.0
```

### Q: 如何在没有硬件的情况下测试？
使用仿真模式：
```bash
ros2 launch wujihand_bringup wujihand_mock.launch.py
```

### Q: 如何同时控制左右手？
启动两个节点，指定不同的序列号和命名空间：
```bash
# 终端1
ros2 launch wujihand_bringup wujihand.launch.py serial_number:=LEFT_SN

# 终端2  
ros2 launch wujihand_bringup wujihand.launch.py serial_number:=RIGHT_SN
```
