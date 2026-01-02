/*
 * RPLIDAR ROS2 DRIVER
 *
 * Copyright (c) 2025, frozenreboot
 * Copyright (c) 2009 - 2014 RoboPeak Team
 * Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *
 * This project is a refactored version of the original rplidar_ros package.
 * The architecture has been redesigned to support ROS2 Lifecycle and
 * Multithreading.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file lidar_driver_wrapper.cpp
 * @brief Implementation of the LidarDriverInterface for Slamtec RPLIDAR.
 *
 * This file implements the concrete driver classes declared in
 * @ref lidar_driver_wrapper.hpp:
 *  - @ref RealLidarDriver : wraps the Slamtec SDK and performs hardware I/O
 *  - @ref DummyLidarDriver : provides a synthetic LIDAR for testing/simulation
 *
 * The implementation:
 *  - Bridges the Slamtec SDK with a clean C++/ROS 2-friendly interface
 *  - Supports automatic protocol detection (legacy vs. S-series / DTOF)
 *  - Retrieves and caches device capabilities such as max range and scan modes
 *  - Provides a simple synthetic scan generator for dummy mode
 *
 * @author  frozenreboot (frozenreboot@gmail.com)
 * @date    2025-12-24
 */

#include "lidar_driver_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

// Optional: suppress warnings originating from the Slamtec SDK headers,
// which may use C-style constructs that trigger pedantic warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop

using namespace sl;

// ============================================================================
// [Real Lidar Driver Implementation]
// ============================================================================

RealLidarDriver::RealLidarDriver() {
  drv_ = *sl::createLidarDriver();
  if (drv_) {
    std::memset(&devinfo_, 0, sizeof(devinfo_));
  }
}

RealLidarDriver::~RealLidarDriver() {
  if (drv_) {
    if (drv_->isConnected()) {
      drv_->stop();
      drv_->setMotorSpeed(0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    drv_->disconnect();
    delete drv_;
    drv_ = nullptr;
  }
}

bool RealLidarDriver::connect(const std::string &port, sl_u32 baudrate,
                              bool use_geometric_compensation) {
  if (!drv_) {
    return false;
  }

  // Reset cached device info before connecting.
  std::memset(&devinfo_, 0, sizeof(devinfo_));

  // Store geometric compensation preference in the profile.
  profile_.apply_geometric_correction = use_geometric_compensation;

  // Create serial channel.
  auto channel_res = createSerialPortChannel(port, baudrate);

  // Explicit cast to sl_result to avoid ambiguity.
  if (SL_IS_FAIL(static_cast<sl_result>(channel_res))) {
    std::cerr << "[Driver] CRITICAL: Failed to open serial port '" << port
              << "'!" << std::endl;
    std::cerr << "[Driver] Hint: Check permissions (e.g., sudo chmod 666 "
              << port << ") or physical connection." << std::endl;
    return false;
  }

  IChannel *channel = *channel_res;
  if (!channel) {
    return false;
  }

  // Bind driver to the channel.
  auto ans = drv_->connect(channel);
  if (SL_IS_FAIL(ans)) {
    std::cerr << "[Driver] Failed to bind connection with device." << std::endl;
    return false;
  }

  // Retrieve device info.
  auto res = drv_->getDeviceInfo(devinfo_);
  if (SL_IS_FAIL(res)) {
    std::cerr << "[Driver] Failed to get device info (data may be corrupted)."
              << std::endl;
    std::memset(&devinfo_, 0, sizeof(devinfo_));
    return false;
  }

  return true;
}

void RealLidarDriver::detect_and_init_strategy() {
  if (!drv_) {
    return;
  }

  sl::LIDARTechnologyType tech_type = drv_->getLIDARTechnologyType(&devinfo_);
  sl::LIDARMajorType major_type = drv_->getLIDARMajorType(&devinfo_);

  std::cout << "[Driver] Device Model ID: " << (int)devinfo_.model << std::endl;

  // S-Series / DTOF logic.
  if (tech_type == sl::LIDAR_TECHNOLOGY_DTOF ||
      major_type == sl::LIDAR_MAJOR_TYPE_S_SERIES) {
    profile_.protocol = ProtocolType::NEW_TYPE;
    switch (devinfo_.model) {
    case 65: // 0x41
      profile_.model_name = "RPLIDAR C1";
      break;
    default:
      profile_.model_name = "S-Series (ToF)";
      break;
    }
    profile_.hw_max_distance = 40.0f; // Initial default; may be updated later.
    is_s_series_detected_ = true;
    std::cout << "[Driver] Detected new-type device: " << profile_.model_name
              << " (Model ID: " << (int)devinfo_.model << ")" << std::endl;
  } else {
    profile_.protocol = ProtocolType::OLD_TYPE;
    profile_.model_name = "A-Series (Triangulation)";
    profile_.hw_max_distance = 12.0f;
    is_s_series_detected_ = false;
    std::cout << "[Driver] Detected legacy (A-series) device." << std::endl;
  }
}

bool RealLidarDriver::start_motor(std::string user_mode_pref,
                                  uint16_t user_rpm_pref) {
  if (!isConnected()) {
    return false;
  }

  // Use user RPM if provided, otherwise fall back to a typical default.
  uint16_t target_rpm = (user_rpm_pref > 0) ? user_rpm_pref : 600;
  profile_.active_rpm = target_rpm;

  // ------------------------------------------------------------------------
  // Strategy A: New-Type (S/C-Series)
  // ------------------------------------------------------------------------
  if (profile_.protocol == ProtocolType::NEW_TYPE) {
    drv_->setMotorSpeed(target_rpm);
    std::cout << "[Driver] Starting S-series motor (warm-up 1s)..."
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<LidarScanMode> modes;
    drv_->getAllSupportedScanModes(modes);

    sl_u16 selected_mode_id = static_cast<sl_u16>(-1);
    std::string selected_mode_name = "Default";
    float detected_max_dist = 0.0f;

    // If user requested a specific scan mode, try to match it by name.
    if (!user_mode_pref.empty()) {
      for (const auto &m : modes) {
        if (std::string(m.scan_mode) == user_mode_pref) {
          selected_mode_id = m.id;
          selected_mode_name = m.scan_mode;
          detected_max_dist = m.max_distance;
          break;
        }
      }
      if (selected_mode_id == static_cast<sl_u16>(-1)) {
        std::cerr << "[Driver] Scan mode '" << user_mode_pref
                  << "' not found. Falling back to automatic selection."
                  << std::endl;
      }
    }

    // Automatic fallback: prefer "DenseBoost" if available.
    if (selected_mode_id == static_cast<sl_u16>(-1)) {
      for (const auto &m : modes) {
        if (std::string(m.scan_mode).find("DenseBoost") != std::string::npos) {
          selected_mode_id = m.id;
          selected_mode_name = m.scan_mode;
          detected_max_dist = m.max_distance;
          break;
        }
      }
    }

    // Second fallback: try "Sensitivity" modes.
    if (selected_mode_id == static_cast<sl_u16>(-1)) {
      for (const auto &m : modes) {
        if (std::string(m.scan_mode).find("Sensitivity") != std::string::npos) {
          selected_mode_id = m.id;
          selected_mode_name = m.scan_mode;
          detected_max_dist = m.max_distance;
          break;
        }
      }
    }

    if (selected_mode_id != static_cast<sl_u16>(-1)) {
      LidarScanMode scanParams;
      drv_->startScanExpress(false, selected_mode_id, 0, &scanParams);
      profile_.active_mode = selected_mode_name;

      if (detected_max_dist > 0.0f) {
        profile_.hw_max_distance = detected_max_dist;
      }
      return true;
    }
  }

  // ------------------------------------------------------------------------
  // Strategy B: Old-Type (A-Series Legacy)
  // ------------------------------------------------------------------------
  drv_->setMotorSpeed(600);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  profile_.active_mode = "Standard";
  profile_.hw_max_distance = 12.0f; // Legacy default.

  return SL_IS_OK(drv_->startScan(0, 1));
}

void RealLidarDriver::stop_motor() {
  if (isConnected()) {
    drv_->stop();
    drv_->setMotorSpeed(0);
  }
}

void RealLidarDriver::print_summary() {
  std::cout << "\n========================================" << std::endl;
  std::cout << "      RPLIDAR DRIVER CONFIG REPORT      " << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << " Model       : " << profile_.model_name << std::endl;
  std::cout << " Protocol    : "
            << (profile_.protocol == ProtocolType::NEW_TYPE
                    ? "HQ (New-Type)"
                    : "Legacy (Old-Type)")
            << std::endl;
  std::cout << " Active Mode : " << profile_.active_mode << std::endl;
  std::cout << " Target RPM  : " << profile_.active_rpm << std::endl;
  std::cout << " Max Range   : " << profile_.hw_max_distance << " m"
            << std::endl;
  std::cout << " Geo. Comp.  : "
            << (profile_.apply_geometric_correction ? "ON (SDK managed)"
                                                    : "OFF (raw data)")
            << std::endl;
  std::cout << "========================================\n" << std::endl;
}

float RealLidarDriver::get_hw_max_distance() const {
  return profile_.hw_max_distance;
}

bool RealLidarDriver::is_new_type() const {
  return profile_.protocol == ProtocolType::NEW_TYPE;
}

bool RealLidarDriver::grab_scan_data(
    std::vector<sl_lidar_response_measurement_node_hq_t> &nodes) {
  if (!isConnected()) {
    return false;
  }

  static std::vector<sl_lidar_response_measurement_node_hq_t> raw_nodes_buffer;

  // Ensure buffer is large enough for worst-case scan size.
  if (raw_nodes_buffer.size() < 8192) {
    raw_nodes_buffer.resize(8192);
  }

  size_t count = raw_nodes_buffer.size();

  // 1. Retrieve raw scan data.
  auto res = drv_->grabScanDataHq(raw_nodes_buffer.data(), count);

  if (SL_IS_OK(res)) {
    // 2. Conditionally apply geometric correction and sorting
    //    only when explicitly enabled in the profile.
    if (profile_.apply_geometric_correction) {
      drv_->ascendScanData(raw_nodes_buffer.data(), count);
    } else {
      // When disabled, raw ordering and count are used as returned
      // by grabScanDataHq. The SDK updates `count` to the number
      // of valid nodes.
    }

    // 3. Copy the valid portion of the buffer into the output vector.
    nodes.assign(raw_nodes_buffer.begin(), raw_nodes_buffer.begin() + count);
    return true;
  }

  return false;
}

bool RealLidarDriver::set_motor_speed(uint16_t rpm) {
  if (!isConnected()) {
    return false;
  }

  // If rpm is 0, restore a reasonable default (e.g., 600 RPM).
  uint16_t target_rpm = (rpm > 0) ? rpm : 600;

  drv_->setMotorSpeed(target_rpm);
  profile_.active_rpm = target_rpm;

  return true;
}

std::string RealLidarDriver::get_device_info_str() const {
  if (!drv_) {
    return "Disconnected";
  }

  // If serial number is zeroed out, treat as not connected or inaccessible.
  if (devinfo_.serialnum[0] == 0) {
    return "N/A (Not connected or permission denied)";
  }

  std::stringstream ss;
  ss << "S/N: ";
  for (int pos = 0; pos < 16; ++pos) {
    char hex[3];
    std::snprintf(hex, sizeof(hex), "%02X", devinfo_.serialnum[pos]);
    ss << hex;
  }
  ss << " | FW: " << (devinfo_.firmware_version >> 8) << "."
     << (devinfo_.firmware_version & 0xFF);
  ss << " | HW: " << static_cast<int>(devinfo_.hardware_version);
  ss << " | Type: " << profile_.model_name;
  return ss.str();
}

void RealLidarDriver::disconnect() {
  if (drv_) {
    drv_->disconnect();
  }
}

bool RealLidarDriver::isConnected() { return drv_ && drv_->isConnected(); }

int RealLidarDriver::getHealth() {
  if (!isConnected()) {
    // 2 = error / not healthy
    return 2;
  }
  sl_lidar_response_device_health_t h;
  if (SL_IS_FAIL(drv_->getHealth(h))) {
    return 2;
  }
  if (h.status == SL_LIDAR_STATUS_ERROR) {
    return 2;
  } else if (h.status == SL_LIDAR_STATUS_WARNING) {
    return 1;
  }
  return 0;
}

void RealLidarDriver::reset() {
  if (isConnected()) {
    drv_->reset();
  }
}

// ============================================================================
// [Dummy Lidar Driver Implementation]
// ============================================================================

bool DummyLidarDriver::connect(const std::string &, sl_u32, bool) {
  return true;
}

void DummyLidarDriver::disconnect() {}

bool DummyLidarDriver::isConnected() { return true; }

int DummyLidarDriver::getHealth() { return 0; }

void DummyLidarDriver::reset() {}

bool DummyLidarDriver::start_motor(std::string, uint16_t) { return true; }

void DummyLidarDriver::stop_motor() {}

void DummyLidarDriver::detect_and_init_strategy() {}

void DummyLidarDriver::print_summary() {
  std::cout << "[Dummy] Virtual RPLIDAR device ready." << std::endl;
}

float DummyLidarDriver::get_hw_max_distance() const { return 40.0f; }

bool DummyLidarDriver::grab_scan_data(
    std::vector<sl_lidar_response_measurement_node_hq_t> &nodes) {
  nodes.clear();

  const int count = 360;
  nodes.reserve(count);

  // Simple synthetic scan: a ring with slight sinusoidal variation.
  static float phase = 0.0f;
  phase += 0.1f;

  for (int i = 0; i < count; ++i) {
    sl_lidar_response_measurement_node_hq_t node{};

    // Angle in Q14 format (0~360 degrees mapped appropriately).
    node.angle_z_q14 =
        static_cast<sl_u16>((static_cast<float>(i) * 16384.0f / 90.0f));

    float dist_meters =
        2.0f +
        0.5f * std::sin(static_cast<float>(i) * 3.141592f / 180.0f + phase);

    node.dist_mm_q2 = static_cast<sl_u32>(dist_meters * 1000.0f * 4.0f);
    node.quality = 200;

    nodes.push_back(node);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}
