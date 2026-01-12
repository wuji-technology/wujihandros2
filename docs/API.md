# API 参考

## 命名空间和前缀

所有话题、服务和 TF 坐标系都使用 `hand_name` 参数作为前缀，支持多手同时运行。左右手类型从硬件自动检测。

| 参数 | 说明 | 影响范围 |
|-----|------|---------|
| `hand_name` | 命名空间/前缀 | Topics, Services, TF frames, Joint names |

**默认配置** (`hand_name:=hand_0`):
```
Topics:     /hand_0/joint_states, /hand_0/joint_commands, ...
Services:   /hand_0/set_enabled, /hand_0/reset_error
TF frames:  hand_0/palm_link, hand_0/finger1_link1, ...
Joints:     hand_0/finger1_joint1, hand_0/finger1_joint2, ...
```

**多手配置示例**:
```bash
# 左手（通过序列号区分）
ros2 launch wujihand_bringup wujihand_foxglove.launch.py \
    hand_name:=left_hand serial_number:=ABC123

# 右手
ros2 launch wujihand_bringup wujihand_foxglove.launch.py \
    hand_name:=right_hand serial_number:=DEF456
```

---

## Topics

> 以下示例使用默认命名空间 `hand_0`，实际使用时替换为你的 `hand_name`。

### /{hand_name}/joint_commands (订阅)

控制关节位置的命令话题。

**消息类型**: `sensor_msgs/msg/JointState`

**示例**:
```bash
# 位置数组格式 (20个关节)
ros2 topic pub /hand_0/joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}"

# 命名关节格式（支持带前缀或不带前缀的关节名）
ros2 topic pub /hand_0/joint_commands sensor_msgs/msg/JointState \
  "{name: ['hand_0/finger2_joint2', 'hand_0/finger2_joint3'], position: [1.0, 1.0]}"

# 不带前缀的关节名也支持（向后兼容）
ros2 topic pub /hand_0/joint_commands sensor_msgs/msg/JointState \
  "{name: ['finger2_joint2', 'finger2_joint3'], position: [1.0, 1.0]}"
```

### /{hand_name}/joint_states (发布)

当前关节状态反馈。

**消息类型**: `sensor_msgs/msg/JointState`
**发布频率**: 1000 Hz（可通过 `publish_rate` 参数配置）

**关节名格式**: `{hand_name}/finger{1-5}_joint{1-4}`

**字段说明**:

| 字段 | 类型 | 说明 |
|------|------|------|
| `position` | `float64[20]` | 关节位置（弧度） |
| `velocity` | `float64[20]` | 未使用 |
| `effort` | `float64[20]` | 关节 effort（安培） |

> **Effort 字段说明**
>
> Effort 是电流空间的执行器作用量，经过滤波处理后输出。它不是实际测量的电流值，应将其理解为相对驱动强度。
>
> 典型应用场景：
> - 碰撞检测：effort 突增表示关节受阻
> - 负载监控：可通过 `effort / effort_limit` 计算当前输出百分比
>
> 默认 effort_limit 为 1.5A，最大 3.5A。

### /{hand_name}/hand_diagnostics (发布)

硬件诊断信息。

**消息类型**: `wujihand_msgs/msg/HandDiagnostics`  
**发布频率**: 10 Hz（可通过 `diagnostics_rate` 参数配置）

**消息定义**:
```
std_msgs/Header header
string handedness               # "left" 或 "right"
float32 system_temperature      # 系统温度 (°C)
float32 input_voltage           # 输入电压 (V)
float32[20] joint_temperatures  # 关节温度 (°C)
uint32[20] error_codes          # 错误码
bool[20] enabled                # 启用状态
float32[20] effort_limits       # Effort 限制设置 (A)
```

### /{hand_name}/robot_description (发布)

URDF 机器人描述，用于可视化。

**消息类型**: `std_msgs/msg/String`  
**发布方式**: Latched (QoS transient_local)

---

## Services

### /{hand_name}/set_enabled

启用或禁用关节。

```bash
# 禁用所有关节
ros2 service call /hand_0/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: false}"

# 启用食指
ros2 service call /hand_0/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 1, joint_id: 255, enabled: true}"
```

### /{hand_name}/reset_error

重置关节错误状态。

```bash
ros2 service call /hand_0/reset_error wujihand_msgs/srv/ResetError \
  "{finger_id: 255, joint_id: 255}"
```

---

## Launch Parameters

### 通用参数

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `hand_name` | `hand_0` | 命名空间和 TF 前缀 |
| `serial_number` | `""` | 设备序列号，空则自动连接 |

> **说明**：左右手类型从硬件自动检测，无需手动指定。

### 驱动参数

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `publish_rate` | `1000.0` | 状态发布频率 (Hz) |
| `filter_cutoff_freq` | `10.0` | 低通滤波截止频率 (Hz) |
| `diagnostics_rate` | `10.0` | 诊断发布频率 (Hz) |

---

## 关节命名规则

20个关节 = 5根手指 × 4个关节/手指

**完整关节名格式**: `{hand_name}/finger{1-5}_joint{1-4}`

| 索引 | 关节名 (hand_name=hand_0) | 手指 |
|-----|---------------------------|------|
| 0-3 | hand_0/finger1_joint1~4 | 拇指 |
| 4-7 | hand_0/finger2_joint1~4 | 食指 |
| 8-11 | hand_0/finger3_joint1~4 | 中指 |
| 12-15 | hand_0/finger4_joint1~4 | 无名指 |
| 16-19 | hand_0/finger5_joint1~4 | 小指 |

**索引计算**: `index = finger_id * 4 + joint_id`

---

## 代码示例

### Python

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Commander(Node):
    def __init__(self, hand_name='hand_0'):
        super().__init__('commander')
        self.hand_name = hand_name
        self.pub = self.create_publisher(
            JointState, 
            f'/{hand_name}/joint_commands', 
            10
        )
    
    def send_command(self, positions):
        msg = JointState()
        msg.position = list(positions)
        self.pub.publish(msg)

rclpy.init()
node = Commander(hand_name='hand_0')
node.send_command([0.0] * 20)  # 所有关节归零
```

### C++

```cpp
#include <sensor_msgs/msg/joint_state.hpp>

std::string hand_name = "hand_0";
auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/" + hand_name + "/joint_commands", 10);

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

### ros2 topic/service list 显示不完整

ROS2 daemon 可能缓存了旧的节点信息，导致话题或服务列表不完整：

```bash
# 重启 daemon
ros2 daemon stop
ros2 daemon start
sleep 2

# 重新查看
ros2 topic list
ros2 service list
```

### 找不到 set_enabled/reset_error 服务

1. 确认驱动正在运行：
```bash
ps aux | grep wujihand_driver
```

2. 重启 ROS2 daemon（见上）

3. 使用 `--no-daemon` 选项直接查询：
```bash
ros2 service list --no-daemon | grep -E "set_enabled|reset_error"
```

### RViz 模型不显示

1. 确认 Fixed Frame 设置为 `{hand_name}/palm_link`（如 `hand_0/palm_link`）
2. 确认 RobotModel 的 Description Topic 设置为 `/{hand_name}/robot_description`
3. 检查 joint_states: `ros2 topic echo /hand_0/joint_states --once`

### Foxglove/Lichtblick URDF 不显示

1. 在 3D Panel 中添加 Custom Layer -> URDF
2. 设置 Topic 为 `/{hand_name}/robot_description`（如 `/hand_0/robot_description`）
3. Frame Prefix 留空（已在 URDF 中处理）

### WSL2 USB 转发

```powershell
# Windows PowerShell (管理员)
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```
