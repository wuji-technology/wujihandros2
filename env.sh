#!/usr/bin/env bash
# WujiHand ROS2 Environment Setup Script
# Usage: source env.sh

# Detect shell type and get script directory
if [ -n "$ZSH_VERSION" ]; then
    SCRIPT_DIR="${0:A:h}"
elif [ -n "$BASH_VERSION" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    echo "Unsupported shell. Please use bash or zsh."
    return 1
fi

# Auto-detect ROS2 installation
ROS2_DISTRO=""
ROS2_DISTROS=("rolling" "kilted" "jazzy" "iron" "humble")

for distro in "${ROS2_DISTROS[@]}"; do
    if [ -f "/opt/ros/${distro}/setup.zsh" ] || [ -f "/opt/ros/${distro}/setup.bash" ]; then
        ROS2_DISTRO="$distro"
        echo "Detected ROS2 ${distro}"
        break
    fi
done

if [ -z "$ROS2_DISTRO" ]; then
    echo "Error: No ROS2 installation found in /opt/ros/"
    echo "Please install ROS2 first."
    return 1
fi

# Source ROS2 base environment (use appropriate setup file for shell)
if [ -n "$ZSH_VERSION" ]; then
    if [ -f "/opt/ros/${ROS2_DISTRO}/setup.zsh" ]; then
        source "/opt/ros/${ROS2_DISTRO}/setup.zsh"
    else
        # Fallback: emulate bash for setup.bash
        emulate -L bash
        source "/opt/ros/${ROS2_DISTRO}/setup.bash"
        emulate -L zsh
    fi
else
    source "/opt/ros/${ROS2_DISTRO}/setup.bash"
fi

# Source workspace overlay if built
if [ -n "$ZSH_VERSION" ] && [ -f "${SCRIPT_DIR}/install/setup.zsh" ]; then
    source "${SCRIPT_DIR}/install/setup.zsh"
    echo "Sourced workspace: ${SCRIPT_DIR}/install"
elif [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    if [ -n "$ZSH_VERSION" ]; then
        emulate -L bash
        source "${SCRIPT_DIR}/install/setup.bash"
        emulate -L zsh
    else
        source "${SCRIPT_DIR}/install/setup.bash"
    fi
    echo "Sourced workspace: ${SCRIPT_DIR}/install"
else
    echo "Workspace not built yet. Run 'colcon build' first."
fi

echo "WujiHand ROS2 environment ready."
