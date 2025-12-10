# WujiHand ROS2 技术方案设计文档

## 1. 概述

### 1.1 项目背景

WujiHand 是一款五指灵巧手，具有 20 个自由度（每根手指 4 个关节）。本项目为 WujiHand 提供 ROS2 驱动支持，使其能够无缝集成到 ROS2 机器人系统中。

### 1.2 设计目标

- 提供简洁、高效的 ROS2 驱动接口
- 支持 1000Hz 高频实时控制
- 兼容 ROS2 Humble (Ubuntu 22.04) 和 Kilted (Ubuntu 24.04)
- 提供 URDF 模型支持 RViz 可视化
- 最小化外部依赖，便于用户部署

### 1.3 设计原则

1. **简洁优先**：采用简单的发布/订阅架构，避免过度封装
2. **性能优先**：高频控制循环中避免阻塞操作
3. **易于集成**：提供标准 ROS2 接口，兼容现有生态

## 2. 系统架构

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         用户应用层                               │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  用户节点       │  │  MoveIt2        │  │  其他 ROS2 节点  │  │
│  └────────┬────���───┘  └────────┬────────┘  └────────┬────────┘  │
└───────────┼────────────────────┼────────────────────┼───────────┘
            │                    │                    │
            ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                         ROS2 通信层                              │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │  Topics: /hand_command, /hand_state, /joint_states          ││
│  │  Services: /get_hand_info, /get_diagnostics, /set_enabled   ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────┐
│                         驱动层                                   │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                  wujihand_driver_node                       ││
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐          ││
│  │  │ 状态发布    │  │ 命令订阅    │  │ 服务处理    │          ││
│  │  │ (1000Hz)    │  │             │  │             │          ││
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘          ││
│  │         │                │                │                  ││
│  │         ▼                ▼                ▼                  ││
│  │  ┌─────────────────────────────────────────────────────┐    ││
│  │  │              wujihandcpp SDK (静态链接)              │    ││
│  │  │  ┌─────────────────┐  ┌─────────────────────────┐   │    ││
│  │  │  │ 实时控制器      │  │ SDO 读写接口            │   │    ││
│  │  │  │ (位置读写)      │  │ (温度/电压/错误码)      │   │    ││
│  │  │  └────────┬────────┘  └────────────┬────────────┘   │    ││
│  │  └───────────┼────────────────────────┼────────────────┘    ││
│  └──────────────┼────────────────────────┼─────────────────────┘│
└─────────────────┼────────────────────────┼──────────────────────┘
                  │                        │
                  ▼                        ▼
┌─────────────────────────────────────────────────────────────────┐
│                         硬件层                                   │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    USB 通信 (libusb)                        ││
│  └─────────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    WujiHand 硬件                            ││
│  │  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐               ││
│  │  │ F1  │  │ F2  │  │ F3  │  │ F4  │  │ F5  │               ││
│  │  │拇指 │  │食指 │  │中指 │  │无名指│  │小指 │               ││
│  │  │4 DOF│  │4 DOF│  │4 DOF│  │4 DOF│  │4 DOF│               ││
│  │  └─────┘  └─────┘  └─────┘  └─────┘  └─────┘               ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 架构特点

采用简洁的发布/订阅架构：

| 特性 | 说明 |
|-----|------|
| 代码量 | ~230 行核心代码 |
| 复杂度 | 低（标准 ROS2 模式） |
| 灵活性 | 完全自由，易于定制 |
| 调试难度 | 低，标准 ROS2 工具即可 |

架构优势：
1. WujiHand 作为独立设备，不需要复杂的控制器管理
2. 用户学习成本低，熟悉 ROS2 即可上手
3. 调试和维护简单

## 3. 模块设计

### 3.1 包结构

```
wujihand-ros2/
├── third_party/
│   └── wujihandpy/                 # wujihandcpp SDK (git submodule)
│       └── wujihandcpp/
├── wujihand_ws/
│   └── src/wujihand/
│       ├── wujihand_msgs/          # 消息和服务定义
│       ├── wujihand_driver/        # 核心驱动节点
│       └── wujihand_description/   # URDF 模型和可视化配置
└── example/
    └── src/wujihand_example/       # 示例程序
```

### 3.2 wujihand_driver

核心驱动节点，负责与硬件通信。

#### 3.2.1 类设计

```cpp
class WujiHandDriverNode : public rclcpp::Node {
public:
    static constexpr size_t NUM_JOINTS = 20;
    static constexpr size_t NUM_FINGERS = 5;
    static constexpr size_t JOINTS_PER_FINGER = 4;

private:
    // 硬件连接
    std::unique_ptr<wujihandcpp::device::Hand> hand_;
    std::unique_ptr<wujihandcpp::device::IController> controller_;

    // 发布者
    rclcpp::Publisher<HandState>::SharedPtr state_pub_;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;

    // 订阅者
    rclcpp::Subscription<HandCommand>::SharedPtr cmd_sub_;

    // 服务
    rclcpp::Service<GetHandInfo>::SharedPtr get_info_srv_;
    rclcpp::Service<GetDiagnostics>::SharedPtr get_diagnostics_srv_;
    rclcpp::Service<SetEnabled>::SharedPtr set_enabled_srv_;
    rclcpp::Service<ResetError>::SharedPtr reset_error_srv_;

    // 定时器
    rclcpp::TimerBase::SharedPtr state_timer_;

    // 参数
    std::string serial_number_;
    double publish_rate_;           // 默认 1000Hz
    double filter_cutoff_freq_;     // 默认 10Hz

    // 缓存
    std::array<double, NUM_JOINTS> last_positions_;
    HandState hand_state_msg_;      // 预分配消息
    JointState joint_state_msg_;    // 预分配消息
};
```

#### 3.2.2 数据流设计

**位置控制流（高频路径）**：

```
用户节点 → /hand_command → command_callback()
                               │
                               ▼
                    controller_->set_joint_target_position()
                               │
                               ▼
                          USB PDO 写入
                               │
                               ▼
                           硬件执行
```

**状态发布流（高频路径）**：

```
state_timer_ (1000Hz) → publish_state()
                               │
                               ▼
              controller_->get_joint_actual_position()
                    （从缓存读取，无 USB 通信）
                               │
                               ▼
                    发布 /hand_state, /joint_states
```

**诊断数据流（低频路径，按需）**：

```
用户调用 /get_diagnostics → get_diagnostics_callback()
                                      │
                                      ▼
                          hand_->read<Temperature>()
                          hand_->read<InputVoltage>()
                          joint.read<Temperature>()
                          joint.read<ErrorCode>()
                                （USB SDO 读取，较慢）
                                      │
                                      ▼
                                 返回响应
```

#### 3.2.3 性能优化

1. **分离快慢数据路径**
   - 位置数据：从实时控制器缓存读取，无 USB 通信
   - 诊断数据：通过服务按需读取，避免阻塞高频循环

2. **预分配消息内存**
   - `hand_state_msg_` 和 `joint_state_msg_` 在构造时预分配
   - 避免每次发布时动态分配

3. **静态链接 SDK**
   - wujihandcpp 编译为静态库
   - 避免运行时库路径问题

### 3.3 wujihand_msgs

定义 ROS2 消息和服务接口。

#### 3.3.1 消息定义

**HandCommand.msg**
```
std_msgs/Header header
float64[20] target_positions    # 目标位置 (弧度)
```

**HandState.msg**
```
std_msgs/Header header
string handedness               # "left" 或 "right"
float32 system_temperature      # 系统温度 (°C)
float32 input_voltage           # 输入电压 (V)
float64[20] actual_positions    # 实际位置 (弧度)
float64[20] target_positions    # 目标位置 (弧度)
float32[20] joint_temperatures  # 关节温度 (°C)
uint32[20] error_codes          # 错误码
bool[20] enabled                # 启用状态
```

#### 3.3.2 服务定义

**GetHandInfo.srv**
```
---
string handedness
string firmware_version
float64[20] upper_limits
float64[20] lower_limits
```

**GetDiagnostics.srv**
```
---
float64 system_temperature
float64 input_voltage
float64[20] joint_temperatures
uint32[20] error_codes
```

**SetEnabled.srv**
```
uint8 finger_id      # 255 = 所有手指
uint8 joint_id       # 255 = 所有关节
bool enabled
---
bool success
string message
```

**ResetError.srv**
```
uint8 finger_id      # 255 = 所有手指
uint8 joint_id       # 255 = 所有关节
---
bool success
string message
```

### 3.4 wujihand_description

URDF 模型和可视化配置。

#### 3.4.1 关节命名规范

```
finger{1-5}_joint{1-4}

其中：
- finger1 = 拇指
- finger2 = 食指
- finger3 = 中指
- finger4 = 无名指
- finger5 = 小指

- joint1 = 侧摆关节 (abduction)
- joint2 = 掌指关节 (MCP flexion)
- joint3 = 近端指间关节 (PIP)
- joint4 = 远端指间关节 (DIP)
```

#### 3.4.2 坐标系约定

- 使用 ROS 标准坐标系（X 前，Y 左，Z 上）
- URDF 文件后缀 `-ros.urdf` 表示 ROS 坐标系版本

## 4. 接口规范

### 4.1 话题

| 话题名 | 类型 | 方向 | 频率 | 说明 |
|-------|-----|------|-----|------|
| `/hand_command` | HandCommand | 订阅 | - | 位置命令输入 |
| `/hand_state` | HandState | 发布 | 1000Hz | 完整状态输出 |
| `/joint_states` | JointState | 发布 | 1000Hz | RViz 兼容接口 |

### 4.2 服务

| 服务名 | 类型 | 说明 |
|-------|-----|------|
| `/get_hand_info` | GetHandInfo | 获取设备静态信息 |
| `/get_diagnostics` | GetDiagnostics | 获取诊断数据 |
| `/set_enabled` | SetEnabled | 启用/禁用关节 |
| `/reset_error` | ResetError | 重置错误状态 |

### 4.3 参数

| 参数名 | 类型 | 默认值 | 说明 |
|-------|-----|-------|------|
| `serial_number` | string | "" | 设备序列号，空则自动连接 |
| `publish_rate` | double | 1000.0 | 状态发布频率 (Hz) |
| `filter_cutoff_freq` | double | 10.0 | 低通滤波截止频率 (Hz) |

## 5. 依赖管理

### 5.1 SDK 集成方式

采用 Git Submodule 方式集成 wujihandcpp SDK：

```bash
third_party/
└── wujihandpy/           # git submodule
    └── wujihandcpp/      # C++ SDK 源码
```

优点：
- 版本锁定，保证兼容性
- 用户无需单独安装 SDK
- 支持离线编译

### 5.2 编译配置

wujihandcpp 编译为静态库：

```cmake
set(BUILD_STATIC_WUJIHANDCPP ON CACHE BOOL "" FORCE)
add_subdirectory(${WUJIHANDCPP_DIR} ... EXCLUDE_FROM_ALL)
```

好处：
- 可执行文件自包含，无运行时库依赖
- 避免 LD_LIBRARY_PATH 配置问题
- 部署更简单

### 5.3 系统依赖

```bash
# ROS2
ros-${ROS_DISTRO}-desktop

# USB 库
libusb-1.0-0-dev
```

## 6. 构建系统

### 6.1 编译流程

```bash
# 1. 克隆仓库（含 submodule）
git clone --recursive <repo_url>

# 2. 安装依赖
sudo apt install libusb-1.0-0-dev

# 3. 编译
cd wujihand_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

### 6.2 包依赖关系

```
wujihand_msgs          (无依赖)
       │
       ▼
wujihand_driver        (依赖 wujihand_msgs, wujihandcpp)

wujihand_description   (无依赖，URDF 模型)

wujihand_example       (依赖 wujihand_msgs, wujihand_description)
```

## 7. 版本历史

| 版本 | 日期 | 说明 |
|-----|------|------|
| 1.0.0 | 2024-12 | 初始版本，基础位置控制 |
