# API 参考

## Topics

### /joint_commands (订阅)

控制关节位置的命令话题。

**消息类型**: `sensor_msgs/msg/JointState`

**发布频率**: 用户定义 (建议 100Hz 以上)

**格式说明**:
- 支持两种格式：命名关节或位置数组
- 位置单位：弧度 (radians)

**示例**:
```bash
# 位置数组格式 (20个关节)
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{position: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}"

# 命名关节格式
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['finger2_joint2', 'finger2_joint3'], position: [1.0, 1.0]}"
```

---

### /joint_states (发布)

当前关节位置状态。

**消息类型**: `sensor_msgs/msg/JointState`

**发布频率**: 1000 Hz (可配置)

**字段说明**:
- `header.stamp`: 时间戳
- `name`: 20个关节名称
- `position`: 20个关节位置 (弧度)

**示例**:
```bash
ros2 topic echo /joint_states --once
```

---

### /hand_diagnostics (发布)

硬件诊断信息。

**消息类型**: `wujihand_msgs/msg/HandDiagnostics`

**发布频率**: 10 Hz (可配置)

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

**示例**:
```bash
ros2 topic echo /hand_diagnostics
```

---

## Services

### /set_enabled

启用或禁用关节。

**服务类型**: `wujihand_msgs/srv/SetEnabled`

**请求**:
```
uint8 finger_id   # 0-4 (拇指到小指), 255 表示所有手指
uint8 joint_id    # 0-3 (4个关节), 255 表示所有关节
bool enabled      # true 启用, false 禁用
```

**响应**:
```
bool success
string message
```

**示例**:
```bash
# 禁用所有关节
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: false}"

# 启用食指
ros2 service call /set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 1, joint_id: 255, enabled: true}"
```

---

### /reset_error

重置关节错误状态。

**服务类型**: `wujihand_msgs/srv/ResetError`

**请求**:
```
uint8 finger_id   # 0-4 或 255
uint8 joint_id    # 0-3 或 255
```

**响应**:
```
bool success
string message
```

**示例**:
```bash
ros2 service call /reset_error wujihand_msgs/srv/ResetError \
  "{finger_id: 255, joint_id: 255}"
```

---

## Parameters

### 可配置参数

| 参数 | 类型 | 默认值 | 说明 |
|-----|------|-------|------|
| `serial_number` | string | "" | 设备序列号，空则自动连接 |
| `publish_rate` | double | 1000.0 | 状态发布频率 (Hz) |
| `filter_cutoff_freq` | double | 10.0 | 低通滤波截止频率 (Hz) |
| `diagnostics_rate` | double | 10.0 | 诊断发布频率 (Hz) |

**启动时配置**:
```bash
ros2 launch wujihand_bringup wujihand.launch.py \
  serial_number:=XXXXX \
  publish_rate:=500.0
```

### 只读参数

连接硬件后自动设置：

| 参数 | 类型 | 说明 |
|-----|------|------|
| `handedness` | string | "left" 或 "right" |
| `firmware_version` | string | 固件版本，如 "1.0.1" |
| `joint_upper_limits` | double[20] | 关节上限 (弧度) |
| `joint_lower_limits` | double[20] | 关节下限 (弧度) |

**查询参数**:
```bash
ros2 param get /wujihand_driver handedness
ros2 param get /wujihand_driver firmware_version
```

---

## 关节命名规则

20个关节 = 5根手指 × 4个关节/手指

| 索引 | 关节名 | 手指 | 关节类型 |
|-----|-------|------|---------|
| 0-3 | finger1_joint1~4 | 拇指 | 侧摆、屈曲、近端、远端 |
| 4-7 | finger2_joint1~4 | 食指 | 侧摆、屈曲、近端、远端 |
| 8-11 | finger3_joint1~4 | 中指 | 侧摆、屈曲、近端、远端 |
| 12-15 | finger4_joint1~4 | 无名指 | 侧摆、屈曲、近端、远端 |
| 16-19 | finger5_joint1~4 | 小指 | 侧摆、屈曲、近端、远端 |

**索引计算**: `index = finger_id * 4 + joint_id`

---

## Python 使用示例

### 发布命令

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

### 订阅状态

```python
from sensor_msgs.msg import JointState

def callback(msg):
    print(f"Positions: {msg.position}")

node.create_subscription(JointState, '/joint_states', callback, 10)
```

### 调用服务

```python
from wujihand_msgs.srv import SetEnabled

client = node.create_client(SetEnabled, '/set_enabled')
request = SetEnabled.Request()
request.finger_id = 255
request.joint_id = 255
request.enabled = True
future = client.call_async(request)
```

---

## C++ 使用示例

### 发布命令

```cpp
#include <sensor_msgs/msg/joint_state.hpp>

auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

sensor_msgs::msg::JointState msg;
msg.position.resize(20, 0.0);
pub->publish(msg);
```

### 订阅状态

```cpp
auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [](const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("example"), "Position[0]: %f", msg->position[0]);
    });
```

### 调用服务

```cpp
#include <wujihand_msgs/srv/set_enabled.hpp>

auto client = node->create_client<wujihand_msgs::srv::SetEnabled>("/set_enabled");
auto request = std::make_shared<wujihand_msgs::srv::SetEnabled::Request>();
request->finger_id = 255;
request->joint_id = 255;
request->enabled = true;
auto future = client->async_send_request(request);
```
