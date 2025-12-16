# API 参考

## Topics

### /joint_commands (订阅)

控制关节位置的命令话题。

**消息类型**: `sensor_msgs/msg/JointState`

**示例**:
```bash
# 位置数组格式 (20个关节)
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}"

# 命名关节格式
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['finger2_joint2', 'finger2_joint3'], position: [1.0, 1.0]}"
```

### /joint_states (发布)

当前关节位置状态。

**消息类型**: `sensor_msgs/msg/JointState`  
**发布频率**: 1000 Hz

### /hand_diagnostics (发布)

硬件诊断信息。

**消息类型**: `wujihand_msgs/msg/HandDiagnostics`  
**发布频率**: 10 Hz

**消息定义**:
```
std_msgs/Header header
string handedness               # "left" 或 "right"
float32 system_temperature      # 系统温度 (°C)
float32 input_voltage           # 输入电压 (V)
float32[20] joint_temperatures  # 关节温度 (°C)
uint32[20] error_codes          # 错误码
bool[20] enabled                # 启用状态
```

---

## Services

### /set_enabled

启用或禁用关节。

```bash
# 禁用所有关节
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: false}"

# 启用食指
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 1, joint_id: 255, enabled: true}"
```

### /reset_error

重置关节错误状态。

```bash
ros2 service call /reset_error wujihand_msgs/srv/ResetError \
  "{finger_id: 255, joint_id: 255}"
```

---

## Parameters

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `serial_number` | "" | 设备序列号，空则自动连接 |
| `publish_rate` | 1000.0 | 状态发布频率 (Hz) |
| `filter_cutoff_freq` | 10.0 | 低通滤波截止频率 (Hz) |
| `diagnostics_rate` | 10.0 | 诊断发布频率 (Hz) |

---

## 关节命名规则

20个关节 = 5根手指 × 4个关节/手指

| 索引 | 关节名 | 手指 |
|-----|-------|------|
| 0-3 | finger1_joint1~4 | 拇指 |
| 4-7 | finger2_joint1~4 | 食指 |
| 8-11 | finger3_joint1~4 | 中指 |
| 12-15 | finger4_joint1~4 | 无名指 |
| 16-19 | finger5_joint1~4 | 小指 |

**索引计算**: `index = finger_id * 4 + joint_id`

---

## 代码示例

### Python

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
    
    def send_command(self, positions):
        msg = JointState()
        msg.position = positions
        self.pub.publish(msg)

rclpy.init()
node = Commander()
node.send_command([0.0] * 20)  # 所有关节归零
```

### C++

```cpp
#include <sensor_msgs/msg/joint_state.hpp>

auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

sensor_msgs::msg::JointState msg;
msg.position.resize(20, 0.0);
pub->publish(msg);
```

---

## 错误码参考

| 错误码 | 含义 | 处理方法 |
|-------|------|---------|
| 0 | 正常 | - |
| 1 | 过温保护 | 等待冷却后重试 |
| 2 | 过流保护 | 检查负载，重置错误 |
| 3 | 通信超时 | 检查连接，重启驱动 |

---

## 故障排查

### ERROR_BUSY: 设备被占用

```bash
pkill -f wujihand_driver_node
pkill -f robot_state_publisher
ros2 launch wujihand_bringup wujihand.launch.py
```

### ros2 topic list 卡住

```bash
ros2 daemon stop
ros2 daemon start
```

### RViz 模型不显示

1. 确认 Fixed Frame 设置为 `palm_link`
2. 检查 joint_states: `ros2 topic echo /joint_states --once`

### WSL2 USB 转发

```powershell
# Windows PowerShell (管理员)
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```
