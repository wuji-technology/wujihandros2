#!/bin/bash
set -e

# WujiHand ROS2 Debian Package Builder
# Usage: ./build_deb.sh [ROS_DISTRO]
# Example: ./build_deb.sh kilted

ROS_DISTRO=${1:-kilted}
VERSION="0.1.0"
ARCH=$(dpkg --print-architecture)
PACKAGE_NAME="ros-${ROS_DISTRO}-wujihand"

echo "Building ${PACKAGE_NAME} ${VERSION} for ${ARCH}..."

# Check ROS installation
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Error: ROS ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}"
    exit 1
fi

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash

# Clean previous builds
rm -rf build install log
rm -rf debian/tmp debian/.debhelper debian/${PACKAGE_NAME}*
rm -f ../${PACKAGE_NAME}*.deb ../${PACKAGE_NAME}*.changes ../${PACKAGE_NAME}*.buildinfo

# Create package directory structure
INSTALL_DIR="debian/tmp/opt/ros/${ROS_DISTRO}"
mkdir -p "${INSTALL_DIR}"

# Build with colcon
echo "Building packages with colcon..."
colcon build --merge-install --install-base "${INSTALL_DIR}"

# Create DEBIAN control directory
PACKAGE_DIR="debian/${PACKAGE_NAME}_${VERSION}-1_${ARCH}"
mkdir -p "${PACKAGE_DIR}/DEBIAN"
mkdir -p "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}"

# Copy installed files (excluding workspace setup files that conflict with ros-workspace)
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
rm -rf "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/_local_setup_util_sh.py"
rm -rf "${PACKAGE_DIR}/opt/ros/${ROS_DISTRO}/_local_setup_util_ps1.py"

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
mv "${PACKAGE_DIR}.deb" "${PACKAGE_NAME}_${VERSION}-1_${ARCH}.deb"

echo ""
echo "Package built successfully:"
echo "  ${PACKAGE_NAME}_${VERSION}-1_${ARCH}.deb"
echo ""
echo "Install with:"
echo "  sudo dpkg -i ${PACKAGE_NAME}_${VERSION}-1_${ARCH}.deb"
