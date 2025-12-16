#!/bin/bash
set -e

# WujiHand ROS2 Debian Package Builder
# Usage: ./build_deb.sh [ROS_DISTRO]
# Example: ./build_deb.sh kilted

ROS_DISTRO=${1:-kilted}
VERSION="0.1.0"
ARCH=$(dpkg --print-architecture)
PACKAGE_NAME="ros-${ROS_DISTRO}-wujihand-ros2"

echo "Building ${PACKAGE_NAME} ${VERSION} for ${ARCH}..."

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
rm -f ${PACKAGE_NAME}*.deb

# Create package directory structure
INSTALL_DIR="${BUILD_DIR}/install"
mkdir -p "${INSTALL_DIR}"

# Build with colcon using separate directories
echo "Building packages with colcon..."
colcon build \
    --build-base "${BUILD_DIR}/build" \
    --install-base "${INSTALL_DIR}" \
    --merge-install

# Create DEBIAN control directory
PACKAGE_DIR="debian/${PACKAGE_NAME}_${VERSION}-1_${ARCH}"
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
Version: ${VERSION}-1
Section: misc
Priority: optional
Architecture: ${ARCH}
Depends: ros-${ROS_DISTRO}-ros-base, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-robot-state-publisher, wujihandcpp (>= 1.3.0)
Maintainer: Wuji Technology <support@wuji.com>
Description: WujiHand ROS2 Driver
 ROS2 driver package for WujiHand dexterous hand.
 Includes driver node, messages, services, URDF models, and launch files.
EOF

# Build the package
echo "Building deb package..."
dpkg-deb --build "${PACKAGE_DIR}"

# Move to project root directory
DEB_FILENAME="${PACKAGE_NAME}_${VERSION}_${ARCH}.deb"
mv "${PACKAGE_DIR}.deb" "${DEB_FILENAME}"

# Clean up build directory
rm -rf "${BUILD_DIR}" debian

echo ""
echo "Package built successfully:"
echo "  ${DEB_FILENAME}"
echo ""
echo "Install with:"
echo "  sudo dpkg -i ${DEB_FILENAME}"
