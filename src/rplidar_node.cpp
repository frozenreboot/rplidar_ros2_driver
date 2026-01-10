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
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

// ============================================================================
// Constructor / Destructor
// ============================================================================

RPlidarNode::RPlidarNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("rplidar_node", options),
      diagnostic_updater_(this) {
  // Declare static parameters. Values are read in init_parameters().
  this->declare_parameter<std::string>("serial_port", "/dev/rplidar");
  this->declare_parameter<int>("serial_baudrate", 1000000);
  this->declare_parameter<std::string>("frame_id", "laser_frame");
  this->declare_parameter<bool>("inverted", false);
  this->declare_parameter<bool>("angle_compensate", true);

  // Advanced / debug / mode parameters.
  this->declare_parameter<bool>("dummy_mode", false);
  this->declare_parameter<std::string>("scan_mode", "");
  this->declare_parameter<int>("rpm", 0);
  this->declare_parameter<float>("max_distance", 0.0f);

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
  this->get_parameter_or("qos_reliability", qos_policy,
                         std::string("best_effort"));

  rclcpp::QoS qos_profile(10); // History: depth 10

  if (qos_policy == "reliable") {
    qos_profile.reliable();
    RCLCPP_INFO(this->get_logger(), "[QoS] Policy set to RELIABLE (TCP-like).");
  } else {
    qos_profile.best_effort();
    RCLCPP_INFO(this->get_logger(),
                "[QoS] Policy set to BEST_EFFORT (UDP-like).");
  }
  qos_profile.durability_volatile();

  // Create publisher using the configured QoS profile.
  scan_pub_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);

  // ------------------------------------------------------------------------
  // 5. Optional static TF broadcast
  // ------------------------------------------------------------------------
  if (params_.publish_tf) {
    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
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
  // 6. Diagnostics setup
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
  this->get_parameter("serial_port", params_.serial_port);
  this->get_parameter("serial_baudrate", params_.serial_baudrate);
  this->get_parameter("frame_id", params_.frame_id);
  this->get_parameter("inverted", params_.inverted);
  this->get_parameter("angle_compensate", params_.angle_compensate);
  this->get_parameter("dummy_mode", params_.dummy_mode);
  this->get_parameter("scan_mode", params_.scan_mode);
  this->get_parameter("rpm", params_.rpm);
  this->get_parameter("max_distance", params_.max_distance);

  // Dynamic / runtime-tunable parameters with defaults.
  this->declare_parameter<bool>("scan_processing", true);
  this->get_parameter("scan_processing", params_.scan_processing);

  this->declare_parameter<int>("max_retries", 3);
  this->get_parameter("max_retries", params_.max_retries);

  this->declare_parameter<std::string>("qos_reliability", "best_effort");
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", params_.publish_tf);
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
        } else {
          cached_device_info_ = "[Dummy] Virtual Driver";
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
        if (params_.max_distance > 0.0f) {
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
        double duration = (this->now() - start_time).seconds();
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
// LaserScan Publishing
// ============================================================================

/**
 * @brief Convert raw driver data to @c sensor_msgs::msg::LaserScan and publish.
 *
 * @param nodes         Raw measurement nodes from the driver.
 * @param start_time    Timestamp corresponding to the start of the scan.
 * @param scan_duration Total duration of the scan in seconds.
 */
void RPlidarNode::publish_scan(
    const std::vector<sl_lidar_response_measurement_node_hq_t> &nodes,
    rclcpp::Time start_time, double scan_duration) {
  if (nodes.empty()) {
    return;
  }

  // ------------------------------------------------------------------------
  // 1. Pre-processing: filter and normalize measurements
  // ------------------------------------------------------------------------
  struct Point {
    float angle_rad;
    float dist_m;
    float intensity;
  };

  std::vector<Point> valid_points;
  valid_points.reserve(nodes.size());

  bool is_new_protocol = false;
  auto real_drv = dynamic_cast<RealLidarDriver *>(driver_.get());
  if (real_drv && real_drv->is_new_type()) {
    is_new_protocol = true;
  }

  for (const auto &node : nodes) {
    if (node.dist_mm_q2 == 0) {
      continue;
    }

    float angle_deg = node.angle_z_q14 * 90.0f / 16384.0f;
    float angle_rad = angle_deg * (M_PI / 180.0f);
    float dist_m = node.dist_mm_q2 / 4000.0f;
    float intensity = is_new_protocol ? static_cast<float>(node.quality)
                                      : static_cast<float>(node.quality >> 2);

    if (angle_rad < 0.0f) {
      angle_rad += 2.0f * M_PI;
    }
    if (angle_rad >= 2.0f * M_PI) {
      angle_rad -= 2.0f * M_PI;
    }

    valid_points.push_back({angle_rad, dist_m, intensity});
  }

  // ------------------------------------------------------------------------
  // 2. Sort by angle
  // ------------------------------------------------------------------------
  std::sort(
      valid_points.begin(), valid_points.end(),
      [](const Point &a, const Point &b) { return a.angle_rad < b.angle_rad; });

  if (valid_points.empty()) {
    return;
  }

  // ------------------------------------------------------------------------
  // 3. Publish LaserScan message
  // ------------------------------------------------------------------------
  sensor_msgs::msg::LaserScan scan_msg;

  scan_msg.header.stamp = start_time;
  scan_msg.header.frame_id = params_.frame_id;

  scan_msg.angle_min = 0.0f;
  scan_msg.angle_max = 2.0f * M_PI;
  scan_msg.range_min = 0.15f;
  scan_msg.range_max = cached_current_max_range_;
  scan_msg.scan_time = scan_duration;

  // ------------------------------------------------------------------------
  // 4. Mode selection: resampled vs raw mapping
  // ------------------------------------------------------------------------
  if (params_.scan_processing) {
    // Mode A: Resample into a fixed angular grid.
    size_t beam_count = valid_points.size();
    scan_msg.angle_increment =
        static_cast<float>((2.0 * M_PI) / static_cast<double>(beam_count));
    scan_msg.time_increment =
        static_cast<float>(scan_duration / static_cast<double>(beam_count));

    scan_msg.ranges.assign(beam_count, std::numeric_limits<float>::infinity());
    scan_msg.intensities.assign(beam_count, 0.0f);

    for (const auto &p : valid_points) {
      float angle = p.angle_rad;

      if (params_.inverted) {
        angle = (2.0f * M_PI) - angle;
        if (angle >= 2.0f * M_PI) {
          angle -= 2.0f * M_PI;
        }
      }

      int index = static_cast<int>((angle - scan_msg.angle_min) /
                                   scan_msg.angle_increment);

      if (index >= 0 && index < static_cast<int>(beam_count)) {
        if (p.dist_m < scan_msg.ranges[index]) {
          scan_msg.ranges[index] = p.dist_m;
          scan_msg.intensities[index] = p.intensity;
        }
      }
    }
  } else {
    // Mode B: Raw mapping with simple ordering.
    size_t count = valid_points.size();
    double denom = static_cast<double>(count > 1 ? count - 1 : 1);

    scan_msg.angle_increment = static_cast<float>((2.0 * M_PI) / denom);
    scan_msg.time_increment = static_cast<float>(scan_duration / denom);

    scan_msg.ranges.resize(count);
    scan_msg.intensities.resize(count);

    for (size_t i = 0; i < count; ++i) {
      // rplidar turn to CW
      size_t idx = params_.inverted ? i : (count - 1 - i);
      scan_msg.ranges[idx] = valid_points[i].dist_m;
      scan_msg.intensities[idx] = valid_points[i].intensity;
    }
  }

  scan_pub_->publish(scan_msg);
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
    }

    // --------------------------------------------------------------------
    // Case 2: Scan processing (software resampling) toggle
    // --------------------------------------------------------------------
    else if (param.get_name() == "scan_processing" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      params_.scan_processing = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[Dynamic] Scan processing: %s",
                  params_.scan_processing ? "ON" : "OFF");
    }

    // --------------------------------------------------------------------
    // Case 3: Scan mode change (requires motor restart)
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
      } else {
        // Fallback to automatic mode if switch fails.
        RCLCPP_ERROR(this->get_logger(),
                     "[Dynamic] Mode switch failed. Reverting to auto mode...");
        driver_->start_motor("", params_.rpm);
        result.successful = false;
        result.reason = "Failed to switch scan mode";
      }
    }
  }

  return result;
}

// ============================================================================
// registration with rclcpp_components
// ============================================================================

RCLCPP_COMPONENTS_REGISTER_NODE(RPlidarNode)