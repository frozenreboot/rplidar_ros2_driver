# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.2.0] - 2026-01-09
### Changed
- Updated RPLIDAR SDK to v2.1.0 via git submodule.
- Updated copyright years to 2026.

### Fixed
- **Breaking:** Fixed scan data mirroring issue (inverted left/right) for A1/S1 series. 
  - *Note: Please check your robot's rotation direction after this update.*
- Fixed an issue where `launch arguments` were overriding parameter file settings.

## [1.1.0] - 2026-01-06
### Added
- **ROS 2 Component Support:** Refactored `RPlidarNode` into a `rclcpp_component` to enable zero-copy composition and improve IPC performance.
- **Diagnostics:** Implemented standard system health monitoring (frequency, connection status) via `diagnostic_updater` (REP-107).

### Changed
- **REP-117 Compliance:** Updated scan data output to use `Inf` for out-of-range measurements, strictly adhering to standard ROS REPs.