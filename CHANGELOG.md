# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- Upgrade wujihandcpp SDK to v1.4.0
- Fix RViz display and parameter issues
- Force serial_number to string type for ROS2 Kilted compatibility

### Changed

- Rename organization from Wuji-Technology-Co-Ltd to wuji-technology

## [0.1.0] - 2025-12-19

### Added

- ROS2 driver for Wuji Hand with wujihandcpp SDK 1.4.0
- Multi-hand namespace support with XACRO for running multiple hands simultaneously
- Auto-detect handedness from hardware (0=right, 1=left)
- Separate xacro files for left and right hand models
- ROS logging API integration and error handling in launch files
- Common utility module (spawn_robot_state_publisher)
- CI build status badge in README

[Unreleased]: https://github.com/wuji-technology/wujihandros2/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/wuji-technology/wujihandros2/releases/tag/v0.1.0
