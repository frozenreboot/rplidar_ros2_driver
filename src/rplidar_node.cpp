/*
 * RPLIDAR ROS2 DRIVER
 *
 * Copyright (c) 2025 - 2026 frozenreboot
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
 * @file rplidar_node.cpp
 * @brief Implementation of the RPlidarNode ROS 2 Lifecycle node.
 *
 * This file provides the concrete implementation of @ref RPlidarNode declared
 * in @ref rplidar_node.hpp.
 *
 * Responsibilities:
 *  - Declare and load ROS 2 parameters
 *  - Instantiate the appropriate driver (real vs dummy) via a simple factory
 *  - Manage the driver and LIDAR through a fault-tolerant FSM
 *  - Run a dedicated scan thread to acquire data
 *  - Convert raw driver data into @c sensor_msgs::msg::LaserScan
 *  - Publish diagnostics using @c diagnostic_updater
 *
 * The node is intended to be managed via the ROS 2 Lifecycle mechanism and
 * can be driven automatically using the provided launch file.
 *
 * @author  frozenreboot
 * @date    2025-12-24
 */

#include "rplidar_node.hpp"

#include <algorithm> // std::min
#include <chrono>
#include <cmath>
#include <limits>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

namespace rplidar_driver {

// ============================================================================
// Constructor / Destructor
// ============================================================================

RPlidarNode::RPlidarNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("rplidar_node", options),
      diagnostic_updater_(this) {

  RCLCPP_INFO(this->get_logger(),
              "[Lifecycle] Node created. Waiting for configuration.");
}

RPlidarNode::~RPlidarNode() {
  RCLCPP_INFO(this->get_logger(), "[Lifecycle] Destroying node resources...");

  is_scanning_ = false;
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }

  if (driver_) {
    RCLCPP_INFO(this->get_logger(),
                "[Driver] Stopping motor during destruction...");
    driver_->stop_motor();
    driver_->disconnect();
  }
}

// ============================================================================
// Lifecycle Callbacks
// ============================================================================

RPlidarNode::CallbackReturn
RPlidarNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "[Lifecycle] Configuring node...");

  // Load parameters into local struct.
  init_parameters();

  // ------------------------------------------------------------------------
  // 1. Driver instantiation (simple factory: real vs dummy)
  // ------------------------------------------------------------------------
  if (params_.dummy_mode) {
    RCLCPP_WARN(this->get_logger(),
                "[Driver] DUMMY MODE ENABLED - using synthetic scan data.");
    driver_ = std::make_unique<DummyLidarDriver>();
  } else {
    driver_ = std::make_unique<RealLidarDriver>();
  }

  // ------------------------------------------------------------------------
  // 2. Initial connection attempt
  // ------------------------------------------------------------------------
  if (!driver_->connect(params_.serial_port, params_.serial_baudrate,
                        params_.angle_compensate)) {
    RCLCPP_WARN(this->get_logger(),
                "[Driver] Initial connection failed at %s. FSM will retry in "
                "scan loop.",
                params_.serial_port.c_str());
  }

  // ------------------------------------------------------------------------
  // 3. Dynamic parameter callback registration
  // ------------------------------------------------------------------------
  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RPlidarNode::parameters_callback, this, std::placeholders::_1));

  // ------------------------------------------------------------------------
  // 4. QoS setup for LaserScan publisher
  // ------------------------------------------------------------------------
  std::string qos_policy;

  rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

  if (params_.qos_policy == "sensor_data") {
    RCLCPP_INFO(this->get_logger(),
                "[QoS] Policy: rmw_qos_profile_sensor_data");
  } else if (params_.qos_policy == "services_default") {
    qos_profile = rclcpp::ServicesQoS();
    RCLCPP_INFO(this->get_logger(),
                "[QoS] Policy: rmw_qos_profile_services_default");
  } else {
    qos_profile = rclcpp::SystemDefaultsQoS();
    RCLCPP_INFO(this->get_logger(),
                "[QoS] Policy: rmw_qos_profile_system_default");
  }

  // Create publisher using the configured QoS profile.
  scan_pub_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud", qos_profile);

  // ------------------------------------------------------------------------
  // 5. Optional static TF broadcast
  // ------------------------------------------------------------------------
  if (params_.publish_tf) {
    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->now();
    t.header.frame_id = "base_link";     // Parent frame
    t.child_frame_id = params_.frame_id; // Child frame (e.g., "laser_frame")

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(),
                "[TF] Broadcasting static transform: %s -> %s",
                t.header.frame_id.c_str(), t.child_frame_id.c_str());
  }
  // ------------------------------------------------------------------------
  // 6. Detect protocol type
  // ------------------------------------------------------------------------
  auto real_drv = dynamic_cast<RealLidarDriver *>(driver_.get());
  if (real_drv && real_drv->is_new_type()) {
    is_new_protocol_ = true;
    RCLCPP_INFO(this->get_logger(),
                "[Driver] Using new protocol, device is a new-type model.");
  }

  // ------------------------------------------------------------------------
  // 7. Diagnostics setup
  // ------------------------------------------------------------------------
  diagnostic_updater_.setHardwareID("rplidar-" + params_.serial_port);
  diagnostic_updater_.add("RPLidar Status", this,
                          &RPlidarNode::update_diagnostics);

  return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn
RPlidarNode::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "[Lifecycle] Activating node...");

  // Activate base LifecycleNode behavior.
  LifecycleNode::on_activate(state);

  scan_pub_->on_activate();
  if (params_.publish_point_cloud) {
    cloud_pub_->on_activate();
  }
  // Start scan loop thread.
  is_scanning_ = true;
  scan_thread_ = std::thread(&RPlidarNode::scan_loop, this);

  return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn
RPlidarNode::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "[Lifecycle] Deactivating node...");

  is_scanning_ = false;
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }

  scan_pub_->on_deactivate();
  if (cloud_pub_) {
    cloud_pub_->on_deactivate();
  }

  if (driver_) {
    driver_->stop_motor();
  }

  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn
RPlidarNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "[Lifecycle] Cleaning up resources...");

  scan_pub_.reset();
  cloud_pub_.reset();

  if (driver_) {
    driver_->disconnect();
    driver_.reset();
  }

  return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn
RPlidarNode::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "[Lifecycle] Shutting down...");
  return on_cleanup(state);
}

// ============================================================================
// Parameter Initialization
// ============================================================================

void RPlidarNode::init_parameters() {
  // Static parameters
  init_param("serial_port", params_.serial_port);
  init_param("serial_baudrate", params_.serial_baudrate);
  init_param("frame_id", params_.frame_id);
  init_param("angle_compensate", params_.angle_compensate);
  init_param("max_distance", params_.max_distance);
  init_param("dummy_mode", params_.dummy_mode);
  init_param("publish_tf", params_.publish_tf);
  init_param("max_retries", params_.max_retries);
  init_param("use_intensities", params_.use_intensities);
  init_param("intensities_as_angles", params_.intensities_as_angles);
  init_param("angle_offset", params_.angle_offset);
  init_param("symmetric_angle_range", params_.symmetric_angle_range);
  init_param("qos_policy", params_.qos_policy);

  // Dynamic ones, don't forget to check them in the callback
  init_param("scan_mode", params_.scan_mode);
  init_param("rpm", params_.rpm);
  init_param("publish_point_cloud", params_.publish_point_cloud);
  init_param("interpolated_rays", params_.interpolated_rays);
  init_param("computed_ray_count", params_.computed_ray_count);
}
// ============================================================================
// Scan Loop (Fault-Tolerant FSM)
// ============================================================================

/**
 * @brief Main acquisition loop running in a dedicated thread.
 *
 * The loop implements a simple fault-tolerant state machine:
 *  - CONNECTING   : Ensure driver instance exists and try to connect
 *  - CHECK_HEALTH : Verify device health before starting the motor
 *  - WARMUP       : Start motor and configure scan mode
 *  - RUNNING      : Continuously grab scan data and publish LaserScan
 *  - RESETTING    : Recreate driver instance to recover from persistent errors
 */
void RPlidarNode::scan_loop() {
  // Initialize FSM state for this thread
  current_state_.store(DriverState::CONNECTING);

  int error_count = 0;

  RCLCPP_INFO(this->get_logger(), "[FSM] Scan loop started.");

  while (rclcpp::ok() && is_scanning_) {
    // Read current FSM state from atomic variable
    DriverState state = current_state_.load();

    switch (state) {
    // -----------------------------------------------------------------
    // State 1: CONNECTING
    // -----------------------------------------------------------------
    case DriverState::CONNECTING: {
      // Ensure driver instance exists
      if (!driver_) {
        if (params_.dummy_mode) {
          driver_ = std::make_unique<DummyLidarDriver>();
        } else {
          driver_ = std::make_unique<RealLidarDriver>();
        }
      }

      // Attempt connection
      if (!driver_->isConnected()) {
        if (!driver_->connect(params_.serial_port, params_.serial_baudrate,
                              params_.angle_compensate)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "[FSM] Connection failed. Retrying...");
          std::this_thread::sleep_for(1000ms);
          break; // retry in next loop iteration
        }

        // Connection established
        RCLCPP_INFO(this->get_logger(), "[FSM] Connection established.");
      }

      // Detect hardware and cache device info string
      driver_->detect_and_init_strategy();
      {
        auto real_drv = dynamic_cast<RealLidarDriver *>(driver_.get());
        if (real_drv) {
          cached_device_info_ = real_drv->get_device_info_str();
          is_new_protocol_ = real_drv->is_new_type();
        } else {
          cached_device_info_ = "[Dummy] Virtual Driver";
          is_new_protocol_ = false;
        }
        RCLCPP_INFO(this->get_logger(), "[Hardware Detail] %s",
                    cached_device_info_.c_str());
      }

      // Transition: CONNECTING -> CHECK_HEALTH
      current_state_.store(DriverState::CHECK_HEALTH);
      break;
    }

    // -----------------------------------------------------------------
    // State 2: CHECK_HEALTH
    // -----------------------------------------------------------------
    case DriverState::CHECK_HEALTH: {
      int health = driver_->getHealth();
      if (health == 0 || health == 1) { // OK or Warning
        // Transition: CHECK_HEALTH -> WARMUP
        current_state_.store(DriverState::WARMUP);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "[FSM] Health error: %d. Disconnecting...", health);
        driver_->disconnect();
        std::this_thread::sleep_for(1000ms);

        // Transition: back to CONNECTING
        current_state_.store(DriverState::CONNECTING);
      }
      break;
    }

    // -----------------------------------------------------------------
    // State 3: WARMUP
    // -----------------------------------------------------------------
    case DriverState::WARMUP: {
      RCLCPP_INFO(this->get_logger(), "[FSM] Starting motor...");

      if (driver_->start_motor(params_.scan_mode, params_.rpm)) {
        driver_->print_summary();

        float hw_limit = driver_->get_hw_max_distance();
        if (params_.max_distance > 0.001f) {
          cached_current_max_range_ = std::min(params_.max_distance, hw_limit);
        } else {
          cached_current_max_range_ = hw_limit;
        }

        RCLCPP_INFO(this->get_logger(), "[Config] Max Range: %.2f m",
                    cached_current_max_range_);
        RCLCPP_INFO(this->get_logger(),
                    "[FSM] Motor started. Entering RUNNING state.");

        error_count = 0;
        // Transition: WARMUP -> RUNNING
        current_state_.store(DriverState::RUNNING);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[FSM] Failed to start motor.");
        // Transition: failure -> RESETTING
        current_state_.store(DriverState::RESETTING);
      }
      break;
    }

    // -----------------------------------------------------------------
    // State 4: RUNNING
    // -----------------------------------------------------------------
    case DriverState::RUNNING: {
      std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
      rclcpp::Time start_time = this->now();
      bool success = false;

      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (driver_->grab_scan_data(nodes)) {
          success = true;
          error_count = 0;
          // Apply the user-configured motor speed once the first scan confirms
          // the motor is spinning. New-type devices need this because the
          // initial setMotorSpeed() in start_motor() doesn't always stick;
          // A-series needs it because start_motor() deliberately kickstarts at
          // 600 (see wrapper), so the configured value must be replayed here.
          // (#37, #25)
          if (initial_reset_required_) {
            RCLCPP_INFO(this->get_logger(),
                        "[FSM] Applying configured motor speed (%d rpm)...",
                        params_.rpm);
            driver_->set_motor_speed(params_.rpm);
            initial_reset_required_ = false;
          }
        } else {
          error_count++;
          // Use max_retries parameter for hardware timeout behavior
          if (error_count > params_.max_retries) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[FSM] Hardware unresponsive (Over %d errors). Resetting...",
                params_.max_retries);
            // Transition: error -> RESETTING
            current_state_.store(DriverState::RESETTING);
          } else {
            std::this_thread::sleep_for(1ms);
          }
        }
      }

      if (success && !nodes.empty()) {
        rclcpp::Duration duration = (this->now() - start_time);
        publish_scan(nodes, start_time, duration);
      }
      break;
    }

    // -----------------------------------------------------------------
    // State 5: RESETTING
    // -----------------------------------------------------------------
    case DriverState::RESETTING: {
      RCLCPP_WARN(this->get_logger(),
                  "[FSM] Performing hardware reset (recreating driver)...");

      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        driver_.reset(); // Destroy existing driver
        initial_reset_required_ = true;

        if (params_.dummy_mode) {
          driver_ = std::make_unique<DummyLidarDriver>();
        } else {
          driver_ = std::make_unique<RealLidarDriver>();
        }
      }

      std::this_thread::sleep_for(2000ms);

      // Transition: RESETTING -> CONNECTING
      current_state_.store(DriverState::CONNECTING);
      error_count = 0;
      break;
    }
    }

    // Reduce CPU usage when not actively scanning
    if (current_state_.load() != DriverState::RUNNING) {
      std::this_thread::sleep_for(10ms);
    }
  }

  RCLCPP_INFO(this->get_logger(), "[FSM] Scan loop terminated.");
}

// ============================================================================
// Diagnostics
// ============================================================================

void RPlidarNode::update_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {

  // --------------------------------------------------------------------------
  // [Gatekeeper] Check Lifecycle State first
  // --------------------------------------------------------------------------
  // If the node is not in the ACTIVE state, we should report "Inactive"
  // regardless of the internal driver FSM state.
  if (this->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Node Inactive");
    stat.add("Lifecycle State", "Inactive");
    // Preserve static info if available
    stat.add("Serial Port", params_.serial_port);
    stat.add("Device Info", cached_device_info_);
    return;
  }

  // --------------------------------------------------------------------------
  // [Internal State] Report Driver FSM Status (Only when Active)
  // --------------------------------------------------------------------------
  std::lock_guard<std::mutex> lock(driver_mutex_);

  // Read current FSM state from atomic variable
  DriverState state = current_state_.load();

  if (state == DriverState::RUNNING) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Scanning");
    stat.add("Connection", "Connected");
    stat.add("Health Code", "OK (Scanning Active)");
  } else if (state == DriverState::WARMUP) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warming Up");
    stat.add("Connection", "Connected (Starting Motor)");
    stat.add("Health Code", "Warming Up");
  } else if (state == DriverState::CONNECTING ||
             state == DriverState::CHECK_HEALTH) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Connecting");
    stat.add("Connection", "Connecting...");
    stat.add("Health Code", "Initializing");
  } else if (state == DriverState::RESETTING) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Hardware Error / Resetting");
    stat.add("Connection", "Disconnected / Resetting");
    stat.add("Health Code", "Error");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Unknown State");
    stat.add("Connection", "Unknown");
    stat.add("Health Code", "Unknown");
  }

  // Additional metadata
  stat.add("Serial Port", params_.serial_port);
  stat.add("Target RPM", params_.rpm);
  stat.add("Device Info", cached_device_info_);
}

// ============================================================================
// LaserScan and PointCloud2 Publishing
// ============================================================================

void RPlidarNode::publish_scan(
    const std::vector<sl_lidar_response_measurement_node_hq_t> &nodes,
    rclcpp::Time time, rclcpp::Duration duration) {
  if (nodes.empty()) {
    return;
  }

  // ------------------------------------------------------------------------
  // 1. Pre-processing: filter and normalize measurements
  // ------------------------------------------------------------------------

  std::vector<RpPoint> points;
  // points.resize(nodes.size());

  float max_intensity = -std::numeric_limits<float>::infinity();
  float min_intensity = std::numeric_limits<float>::infinity();

  for (const auto &node : nodes) {
    double angle_rad, dist_m, x, y;
    float intensity;

    // RPLidar angle readings are clockwise 0 to 360, mathematically incorrect.
    // Angles should increase counter-clockwise. In ROS standard, X(0) is front,
    // Y is left. Also reported from the driver in counter-clockwise order.
    angle_rad = TWO_PI - (node.angle_z_q14 * Q14_TO_RAD);
    // Also They don't always start with the smallest angles closest to the
    // front. Instead, those angles are appended to the end of the array.
    // However, the angle order is usually preserved. Unfortunately, sometimes
    // The order is fully corrupted, so we need to sort them later.

    angle_rad += params_.angle_offset;
    // Loops rather than single comparisons: angle_offset is unbounded, so one
    // correction is not always enough to land back inside the range.
    // TEMPORARY: the asymmetric branch only exists to keep pre-existing
    // deployments byte-identical. See Parameters::symmetric_angle_range; the
    // whole conditional collapses into the symmetric case on the next major
    // release.
    if (params_.symmetric_angle_range) {
      while (angle_rad >= M_PI)
        angle_rad -= TWO_PI;
      while (angle_rad < -M_PI)
        angle_rad += TWO_PI;
    } else {
      while (angle_rad >= TWO_PI)
        angle_rad -= TWO_PI;
      while (angle_rad < 0.0)
        angle_rad += TWO_PI;
    }

    if (node.dist_mm_q2 == 0) {
      if (params_.interpolated_rays) {
        dist_m = std::numeric_limits<double>::infinity();
        x = std::numeric_limits<double>::infinity();
        y = std::numeric_limits<double>::infinity();
        intensity = 0.;
      } else {
        continue;
      }
    } else {
      dist_m = node.dist_mm_q2 / 4000.0;
      x = static_cast<double>(dist_m * cos(angle_rad));
      y = static_cast<double>(dist_m * sin(angle_rad));
      intensity = is_new_protocol_ ? static_cast<float>(node.quality)
                                   : static_cast<float>(node.quality >> 2);
    }

    min_intensity = std::min(min_intensity, intensity);
    max_intensity = std::max(max_intensity, intensity);

    points.push_back({angle_rad, dist_m, x, y, intensity});
  }

  // ------------------------------------------------------------------------
  // 2. Sort by angle
  // ------------------------------------------------------------------------

  std::sort(points.begin(), points.end(),
            [](const RpPoint &a, const RpPoint &b) -> bool {
              return a.angle_rad < b.angle_rad;
            });

  if (points.empty()) {
    return;
  }

  // ------------------------------------------------------------------------
  // 3. Prepare to publish messages
  // ------------------------------------------------------------------------

  uint32_t beam_count;
  if (params_.interpolated_rays) {
    beam_count = params_.computed_ray_count;
  } else {
    beam_count = points.size();
  }

  sensor_msgs::msg::LaserScan scan_msg;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  cloud_msg.width = points.size();
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(cloud_msg.width);
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud_msg, "b");

  scan_msg.angle_increment =
      static_cast<float>(TWO_PI / static_cast<double>(beam_count));
  if (params_.interpolated_rays) {
    // The interpolated grid is synthetic, so its origin has to be stated
    // explicitly. The measured branch below needs no equivalent: its bounds
    // come from the sorted samples and therefore follow whichever convention
    // the normalization above produced.
    scan_msg.angle_min =
        params_.symmetric_angle_range ? -static_cast<float>(M_PI) : 0.f;
    scan_msg.angle_max =
        scan_msg.angle_min + scan_msg.angle_increment * beam_count;
  } else {
    scan_msg.angle_min = points[0].angle_rad;
    scan_msg.angle_max = points[beam_count - 1].angle_rad;
  }

  long double dur = (duration.nanoseconds() * 1e-9);
  scan_msg.time_increment = static_cast<float>(dur / beam_count);
  scan_msg.header.stamp = time;
  scan_msg.header.frame_id = params_.frame_id;
  scan_msg.range_min = 0.01f;
  scan_msg.range_max = cached_current_max_range_;
  scan_msg.scan_time = dur;
  scan_msg.ranges.assign(beam_count, std::numeric_limits<float>::infinity());
  scan_msg.angle_increment =
      static_cast<float>(TWO_PI / static_cast<double>(beam_count));
  if (params_.use_intensities) {
    scan_msg.intensities.assign(beam_count, 0.0f);
  }

  if (params_.publish_point_cloud) {
    cloud_msg.header.stamp = time;
    cloud_msg.header.frame_id = params_.frame_id;
    cloud_msg.height = 1;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false; // there may be invalid points
  }

  float scaled;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &p = points[i];
    if (!params_.interpolated_rays) {
      int index = static_cast<int>((p.angle_rad - scan_msg.angle_min) /
                                   scan_msg.angle_increment);
      if (index >= 0 && index < static_cast<int>(beam_count)) {
        if (p.dist_m < scan_msg.ranges[index]) {
          scan_msg.ranges[index] = static_cast<float>(p.dist_m);
          if (params_.use_intensities) {
            if (!params_.intensities_as_angles) {
              scan_msg.intensities[index] =
                  scaleIntensity(p.intensity, min_intensity, max_intensity);
            } else {
              scan_msg.intensities[index] = p.angle_rad;
            }
          }
        }
      }
    }

    if (params_.publish_point_cloud) {
      *out_x = static_cast<float>(p.x);
      *out_y = static_cast<float>(p.y);
      *out_z = 0.f;
      *out_r = 0.f;
      *out_g = params_.use_intensities ? p.intensity : 255;
      *out_b = 0.f;

      ++out_x;
      ++out_y;
      ++out_z;
      ++out_r;
      ++out_g;
      ++out_b;
    }
  }

  // separate loop for interpolation
  // YAML file warns about optimization and how to do it with the parameters.
  if (params_.interpolated_rays) {
    int p_size = points.size();
    if (p_size < 2)
      return;

    for (int p = 1, r = 1; r < (beam_count - 1); r++) {
      // Anchored to angle_min, not to zero: the ray grid has to start wherever
      // the published scan starts.
      double target_angle = scan_msg.angle_min + r * scan_msg.angle_increment;

      // Find the segment [p-1, p] that contains target_angle
      // Check p < p_size-1 before incrementing p
      while (p < p_size && points[p].angle_rad < target_angle) {
        p++;
      }
      // when data ends break for prevent segfault
      if (p >= p_size) {
        break;
      }
      const auto &prev = points[p - 1];
      const auto &curr = points[p];
      if (target_angle >= prev.angle_rad && target_angle <= curr.angle_rad) {
        if (!std::isinf(prev.dist_m) && !std::isinf(curr.dist_m)) {
          scan_msg.ranges[r] = raySegmentIntersection(target_angle, prev, curr);
          if (params_.use_intensities) {
            scan_msg.intensities[r] =
                scaleIntensity(curr.intensity, min_intensity, max_intensity);
          }
        }
      }
    }
  }

  scan_pub_->publish(scan_msg);
  if (params_.publish_point_cloud) {
    cloud_pub_->publish(cloud_msg);
  }
}

// ============================================================================
// Dynamic Parameter Callback
// ============================================================================

rcl_interfaces::msg::SetParametersResult RPlidarNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  std::lock_guard<std::mutex> lock(driver_mutex_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // If the driver is not ready, reject runtime parameter updates.
  if (!driver_ || !driver_->isConnected()) {
    result.successful = false;
    result.reason = "Driver not ready";
    return result;
  }

  for (const auto &param : parameters) {
    // --------------------------------------------------------------------
    // Case 1: RPM change
    // --------------------------------------------------------------------
    if (param.get_name() == "rpm" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      int new_rpm = param.as_int();

      // Valid range: 0 = auto/default, up to a reasonable upper bound.
      if (new_rpm < 0 || new_rpm > 1200) {
        result.successful = false;
        result.reason = "RPM must be between 0 and 1200";
        RCLCPP_WARN(this->get_logger(),
                    "[Param] Rejecting RPM %d (out of range).", new_rpm);
        return result;
      }

      RCLCPP_INFO(this->get_logger(), "[Dynamic] Updating RPM to %d...",
                  new_rpm);
      driver_->set_motor_speed(static_cast<uint16_t>(new_rpm));
      params_.rpm = new_rpm;

      if (params_.interpolated_rays) {
        RCLCPP_WARN(
            this->get_logger(),
            "[Dynamic] interpolated_rays was enabled before rpm change. "
            "Do not forget to set 'computed_ray_count' parameter accordingly.");
      }
    }

    // --------------------------------------------------------------------
    // Case 2: Publishing PointCloud2 toggle
    // --------------------------------------------------------------------
    else if (param.get_name() == "publish_point_cloud" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      params_.publish_point_cloud = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[Dynamic] PointCloud2 publishing: %s",
                  params_.publish_point_cloud ? "ON" : "OFF");
    }

    // --------------------------------------------------------------------
    // Case 3: Enabling / Disabling ray interpolation
    // --------------------------------------------------------------------
    else if (param.get_name() == "interpolated_rays" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      params_.interpolated_rays = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[Dynamic] Ray Interpolation: %s",
                  params_.interpolated_rays ? "ON" : "OFF");
      RCLCPP_WARN(
          this->get_logger(),
          "[Dynamic] interpolated_rays enabled. "
          "Do not forget to set 'computed_ray_count' parameter accordingly.");
    }

    // --------------------------------------------------------------------
    // Case 4: Ray(imaginary beam) count for interpolation
    // --------------------------------------------------------------------
    else if (param.get_name() == "computed_ray_count" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      params_.computed_ray_count = param.as_int();
      RCLCPP_INFO(this->get_logger(), "[Dynamic] Ray Interpolation: %d",
                  params_.computed_ray_count);
    }

    // --------------------------------------------------------------------
    // Case 5: Scan mode change (requires motor restart)
    // --------------------------------------------------------------------
    else if (param.get_name() == "scan_mode" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      std::string new_mode = param.as_string();

      if (new_mode == params_.scan_mode) {
        continue; // No change.
      }

      RCLCPP_WARN(
          this->get_logger(),
          "[Dynamic] Switching scan mode to '%s' (device restarting...)",
          new_mode.c_str());

      // Stop the motor before switching modes.
      driver_->stop_motor();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // Attempt restart with new mode.
      if (driver_->start_motor(new_mode, params_.rpm)) {
        params_.scan_mode = new_mode;
        driver_->print_summary();
        RCLCPP_INFO(this->get_logger(), "[Dynamic] Mode switch successful.");
        if (params_.interpolated_rays) {
          RCLCPP_WARN(this->get_logger(),
                      "[Dynamic] interpolated_rays was enabled before scan "
                      "mode change. "
                      "Do not forget to set 'computed_ray_count' parameter "
                      "accordingly.");
        }
      } else {
        // Fallback to automatic mode if switch fails.
        RCLCPP_ERROR(this->get_logger(),
                     "[Dynamic] Mode switch failed. Reverting to auto mode...");
        driver_->start_motor("", params_.rpm);
        result.successful = false;
        result.reason = "Failed to switch scan mode";
      }
    }

    // --------------------------------------------------------------------
    // Case 6: In ROS2, all parameters are configurable. Warn for unsupported
    // --------------------------------------------------------------------
    else {
      RCLCPP_WARN(
          this->get_logger(),
          "[Dynamic] Changing param '%s' dynamically is not supported at the "
          "moment.",
          param.get_name().c_str());
    }
  }

  return result;
}

} // namespace rplidar_driver

// ============================================================================
// registration with rclcpp_components
// ============================================================================

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rplidar_driver::RPlidarNode)
