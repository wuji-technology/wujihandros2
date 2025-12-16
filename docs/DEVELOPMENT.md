# 开发者指南

## 项目结构

```
wujihand_ros2/
├── wujihand_driver/        # C++ 驱动节点
│   ├── include/            # 头文件
│   ├── src/                # 源代码
│   ├── CMakeLists.txt
│   └── package.xml
├── wujihand_msgs/          # 消息和服务定义
│   ├── msg/                # 消息文件
│   ├── srv/                # 服务文件
│   ├── CMakeLists.txt
│   └── package.xml
├── wujihand_bringup/       # 启动和演示
│   ├── launch/             # 启动文件
│   ├── scripts/            # Python 脚本
│   ├── CMakeLists.txt
│   └── package.xml
├── wujihand_description/   # URDF 模型
│   ├── urdf/               # URDF 文件
│   ├── meshes/             # 3D 网格
│   ├── rviz/               # RViz 配置
│   ├── CMakeLists.txt
│   └── package.xml
├── docs/                   # 文档
├── env.sh                  # 环境配置脚本
└── README.md
```

## 依赖关系

```
wujihand_bringup
    ├── wujihand_driver
    │   ├── wujihand_msgs
    │   └── wujihandcpp (外部 SDK)
    └── wujihand_description
```

---

## 如何添加新的演示脚本

### 1. 创建 Python 脚本

在 `wujihand_bringup/scripts/` 下创建脚本：

```python
#!/usr/bin/env python3
"""
My Demo Script
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MyDemo(Node):
    def __init__(self):
        super().__init__('my_demo')
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.get_logger().info('My Demo started')

    def timer_callback(self):
        msg = JointState()
        msg.position = [0.0] * 20
        # 你的控制逻辑
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. 注册到 CMakeLists.txt

编辑 `wujihand_bringup/CMakeLists.txt`：

```cmake
install(PROGRAMS
  scripts/wave_demo.py
  scripts/mock_driver.py
  scripts/my_demo.py      # 添加这行
  DESTINATION lib/${PROJECT_NAME}
)
```

### 3. 编译并运行

```bash
colcon build --packages-select wujihand_bringup
source install/setup.bash
ros2 run wujihand_bringup my_demo.py
```

---

## 如何添加新的 Launch 文件

在 `wujihand_bringup/launch/` 下创建：

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    my_node = Node(
        package='wujihand_bringup',
        executable='my_demo.py',
        name='my_demo',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([my_node])
```

无需修改 CMakeLists.txt，launch 目录会自动安装。

---

## 如何修改驱动行为

### 驱动代码结构

主要文件：
- `wujihand_driver/include/wujihand_driver/wujihand_driver_node.hpp` - 类定义
- `wujihand_driver/src/wujihand_driver_node.cpp` - 实现

### 关键函数

| 函数 | 作用 |
|-----|------|
| `connect_hardware()` | 连接硬件，初始化 |
| `command_callback()` | 处理关节命令 |
| `publish_state()` | 发布关节状态 (1000Hz) |
| `publish_diagnostics()` | 发布诊断信息 (10Hz) |
| `set_enabled_callback()` | 处理启用/禁用服务 |
| `reset_error_callback()` | 处理错误重置服务 |

### 添加新参数

1. 在头文件中声明成员变量
2. 在构造函数中 `declare_parameter()` 和 `get_parameter()`
3. 在 launch 文件中添加 `DeclareLaunchArgument`

---

## 编译和测试

### 完整编译

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
```

### 编译单个包

```bash
colcon build --packages-select wujihand_driver
```

### 清理编译

```bash
rm -rf build install log
colcon build
```

### 测试仿真模式

```bash
# 启动仿真
ros2 launch wujihand_bringup wujihand_mock.launch.py

# 另一个终端，运行演示
ros2 run wujihand_bringup wave_demo.py

# 查看状态
ros2 topic echo /joint_states
```

---

## 代码风格

### C++
- 遵循 ROS2 代码风格
- 使用 `clang-format` 格式化

### Python
- 遵循 PEP8
- 使用 `black` 或 `ruff` 格式化

### 格式化命令

```bash
# C++
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# Python
black wujihand_bringup/scripts/
```

---

## 提交规范

### Commit 消息格式

```
<type>: <description>

[optional body]
```

### Type 类型

| Type | 说明 |
|------|------|
| feat | 新功能 |
| fix | Bug 修复 |
| docs | 文档更新 |
| refactor | 重构 |
| test | 测试相关 |
| chore | 构建/工具相关 |

### 示例

```
feat: add mock driver for simulation
fix: resolve joint state publishing race condition
docs: update API reference
```
