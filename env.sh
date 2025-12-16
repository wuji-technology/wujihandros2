#!/bin/bash
# WujiHand ROS2 环境配置脚本
# 使用方法: source env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 自动检测并加载 ROS2 环境
if [ -f /opt/ros/kilted/setup.bash ]; then
    source /opt/ros/kilted/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: No ROS2 installation found in /opt/ros/"
fi

# 加载工作空间
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "WujiHand ROS2 环境已配置 (ROS_DISTRO=$ROS_DISTRO)"
else
    echo "Warning: Workspace not built yet. Run 'colcon build' first."
    echo "ROS2 环境已配置 (ROS_DISTRO=$ROS_DISTRO)"
fi
