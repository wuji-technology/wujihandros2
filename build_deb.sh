#!/bin/bash
set -e

# WujiHand ROS2 Debian Package Builder
# Usage: ./build_deb.sh [ROS_DISTRO] [VERSION]
# Example: ./build_deb.sh kilted 0.2.0-rc0
#
# Prerequisite: wujihandcpp must already be built and install-tree
# accessible via CMAKE_PREFIX_PATH. The CI workflow handles this by
# checking out the sibling SDK repo, running cmake --install into
# /opt/wujihandcpp, and exporting CMAKE_PREFIX_PATH=/opt/wujihandcpp
# before invoking this script.
#
# For manual runs:
#   cd /path/to/wujihandpy/wujihandcpp
#   cmake -B build && cmake --build build && cmake --install build --prefix /tmp/wcpp
#   cd /path/to/wujihandros2
#   CMAKE_PREFIX_PATH=/tmp/wcpp ./build_deb.sh humble

if [ -z "${CMAKE_PREFIX_PATH:-}" ]; then
    echo "Error: CMAKE_PREFIX_PATH is not set." >&2
    echo "       Build wujihandcpp first and point CMAKE_PREFIX_PATH at its install prefix." >&2
    echo "       See the comment block at the top of this script for details." >&2
    exit 1
fi

ROS_DISTRO=${1:-kilted}
VERSION=${2:-0.1.0}

# Convert version to Debian format (replace - with ~ for prerelease)
# e.g., 0.2.0-rc0 -> 0.2.0~rc0 (in Debian, ~ sorts before anything)
DEB_VERSION=$(echo "${VERSION}" | sed 's/-/~/g')

ARCH=$(dpkg --print-architecture)
PACKAGE_NAME="ros-${ROS_DISTRO}-wujihand"

echo "Building ${PACKAGE_NAME} ${DEB_VERSION} for ${ARCH}..."

# Check ROS installation
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Error: ROS ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}"
    exit 1
fi

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash

# Use separate build directory to avoid affecting local development
BUILD_DIR=".deb_build"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# Clean previous debian builds
rm -rf debian
rm -f "${PACKAGE_NAME}"*.deb

# Ensure submodules are initialized and updated
echo "Initializing and updating submodules..."
git submodule update --init --recursive

# Create package directory structure
INSTALL_DIR="${BUILD_DIR}/install"
mkdir -p "${INSTALL_DIR}"

# Build with colcon using separate directories. Pass CMAKE_PREFIX_PATH
# through so wujihand_driver's find_package(wujihandcpp) resolves.
echo "Building packages with colcon..."
colcon build \
    --build-base "${BUILD_DIR}/build" \
    --install-base "${INSTALL_DIR}" \
    --merge-install \
    --cmake-args -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"

# Create DEBIAN control directory
PACKAGE_DIR="debian/${PACKAGE_NAME}_${DEB_VERSION}-1_${ARCH}"
mkdir -p "${PACKAGE_DIR}/DEBIAN"
mkdir -p "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}"

# Copy installed files
cp -r "${INSTALL_DIR}"/* "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/"

# Remove conflicting workspace files
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/local_setup.bash"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/local_setup.sh"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/local_setup.zsh"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/local_setup.ps1"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/setup.bash"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/setup.sh"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/setup.zsh"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/setup.ps1"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/.colcon_install_layout"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/COLCON_IGNORE"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/_local_setup_util_sh.py"
rm -f "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/_local_setup_util_ps1.py"

# Generate control file
cat > "${PACKAGE_DIR}/DEBIAN/control" << EOF
Package: ${PACKAGE_NAME}
Version: ${DEB_VERSION}-1
Section: misc
Priority: optional
Architecture: ${ARCH}
Depends: ros-${ROS_DISTRO}-ros-base, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-robot-state-publisher, ros-${ROS_DISTRO}-tf2-ros, ros-${ROS_DISTRO}-rviz2, ros-${ROS_DISTRO}-foxglove-bridge, ros-${ROS_DISTRO}-ros2bag, ros-${ROS_DISTRO}-rosbag2-storage-mcap, libusb-1.0-0
Maintainer: Wuji Technology <support@wuji.com>
Description: WujiHand ROS2 Driver
 ROS2 driver package for WujiHand dexterous hand.
 Includes driver node, messages, services, URDF models, and launch files.
EOF

# Build the package
echo "Building deb package..."
dpkg-deb --build "${PACKAGE_DIR}"

# Move to project root directory (filename matches control Version field)
DEB_FILENAME="${PACKAGE_NAME}_${DEB_VERSION}-1_${ARCH}.deb"
mv "${PACKAGE_DIR}.deb" "${DEB_FILENAME}"

# Clean up build directory
rm -rf "${BUILD_DIR}" debian

echo ""
echo "Package built successfully:"
echo "  ${DEB_FILENAME}"
echo ""
echo "Install with:"
echo "  sudo dpkg -i ${DEB_FILENAME}"
