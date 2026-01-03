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
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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
 *
 */
/**
 * @file rplidar_node.hpp
 * @brief ROS 2 Lifecycle Node definition for Slamtec RPLIDAR.
 *
 * This header declares @ref RPlidarNode, a managed ROS 2 Lifecycle node
 * responsible for:
 *  - Initializing and configuring the RPLIDAR driver
 *  - Managing the device lifecycle (configure / activate / deactivate /
 * cleanup)
 *  - Running a dedicated scan thread for non-blocking data polling
 *  - Publishing @c sensor_msgs::msg::LaserScan messages
 *  - Broadcasting an optional static TF between frames
 *  - Exposing diagnostics via @c diagnostic_updater
 *
 * The actual hardware interaction is delegated to @ref LidarDriverInterface
 * implementations (e.g. @ref RealLidarDriver, @ref DummyLidarDriver).
 *
 * @author  frozenreboot (frozenreboot@gmail.com)
 * @date    2025-12-07
 */
#ifndef RPLIDAR_NODE_HPP_
#define RPLIDAR_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <atomic>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread>
#include <vector>

#include "lidar_driver_wrapper.hpp"

enum class DriverState { CONNECTING, CHECK_HEALTH, WARMUP, RUNNING, RESETTING };
/**
 * @class RPlidarNode
 * @brief Managed ROS 2 Lifecycle node for Slamtec RPLIDAR devices.
 *
 * This node wraps a @ref LidarDriverInterface implementation and exposes the
 * LIDAR as a standard @c sensor_msgs::msg::LaserScan publisher.
 *
 * Key characteristics:
 *  - Uses the LifecycleNode API to provide deterministic startup/shutdown
 *  - Spawns a dedicated background thread for scan acquisition
 *  - Supports both real hardware and dummy (simulation) backends
 *  - Publishes optional static TF between a parent frame and @c frame_id
 *  - Provides diagnostic status via @c diagnostic_updater::Updater
 *
 * Typical lifecycle:
 *  - @ref on_configure : load parameters, create driver, establish connection
 *  - @ref on_activate  : start publishers, start scan thread
 *  - @ref on_deactivate: stop scan thread, deactivate publishers
 *  - @ref on_cleanup   : release driver resources
 *  - @ref on_shutdown  : final cleanup for node destruction
 */
class RPlidarNode final : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief Construct an RPlidarNode.
   *
   * @param options Standard node options (namespace, parameters, etc.).
   */
  explicit RPlidarNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   *
   * Ensures proper shutdown of the scan thread and driver resources if
   * the node is still active during destruction.
   */
  ~RPlidarNode() override;

  // ---------------------------------------------------------------------
  // Lifecycle Callbacks
  // ---------------------------------------------------------------------

  /**
   * @brief Configure the node and driver.
   *
   * Responsibilities:
   *  - Declare and retrieve parameters
   *  - Create and configure the LIDAR driver
   *  - Initialize diagnostics
   *  - (Optionally) broadcast static TF if enabled
   *
   * @param state Current lifecycle state.
   * @return SUCCESS on successful configuration, FAILURE otherwise.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Activate the node.
   *
   * Responsibilities:
   *  - Activate the LaserScan publisher
   *  - Start the background scan thread
   *
   * @param state Current lifecycle state.
   * @return SUCCESS if activation succeeds, FAILURE otherwise.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Deactivate the node.
   *
   * Responsibilities:
   *  - Stop the background scan thread
   *  - Deactivate the LaserScan publisher
   *
   * @param state Current lifecycle state.
   * @return SUCCESS on successful deactivation, FAILURE otherwise.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Cleanup all resources associated with the node.
   *
   * Responsibilities:
   *  - Reset publishers
   *  - Destroy the driver instance
   *  - Clear cached state
   *
   * @param state Current lifecycle state.
   * @return SUCCESS on successful cleanup, FAILURE otherwise.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Shutdown hook for the node.
   *
   * Called during system shutdown or node destruction. Ensures that the
   * scan thread is stopped and any remaining resources are released.
   *
   * @param state Current lifecycle state.
   * @return SUCCESS on graceful shutdown, FAILURE otherwise.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  // ---------------------------------------------------------------------
  // Internal Helpers
  // ---------------------------------------------------------------------

  /**
   * @brief Initialize and validate node parameters.
   *
   * This method declares and reads all configuration parameters into
   * @ref params_, applying defaults where necessary.
   */
  void init_parameters();

  /**
   * @brief Background loop for continuous scan acquisition.
   *
   * This method is intended to run in a dedicated thread while the node is
   * in the ACTIVE state. It repeatedly:
   *  - Acquires scan data from the driver
   *  - Converts it into a @c sensor_msgs::msg::LaserScan
   *  - Publishes the result via @ref scan_pub_
   *
   * The loop terminates when @ref is_scanning_ is set to false.
   */
  void scan_loop();

  /**
   * @brief Publish a single @c sensor_msgs::msg::LaserScan message.
   *
   * @param nodes        Measurement nodes acquired from the driver.
   * @param start_time   Timestamp corresponding to the start of the scan.
   * @param scan_duration Duration of the scan in seconds.
   */
  void publish_scan(
      const std::vector<sl_lidar_response_measurement_node_hq_t> &nodes,
      rclcpp::Time start_time, double scan_duration);

  /**
   * @brief Diagnostic callback used by @ref diagnostic_updater_.
   *
   * This method populates the diagnostic status with the current health
   * information obtained from the driver and internal state.
   *
   * @param stat Diagnostic status wrapper to be filled.
   */
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // ---------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------

  /**
   * @brief Aggregated configuration parameters for the node.
   *
   * All node parameters are collected into this structure during
   * @ref init_parameters() for easier access and validation.
   */
  struct Parameters {
    /// Serial device path to the RPLIDAR (e.g., "/dev/ttyUSB0").
    std::string serial_port;

    /// Serial baudrate used to communicate with the RPLIDAR.
    int serial_baudrate;

    /// Frame ID used in published LaserScan messages.
    std::string frame_id;

    /// If true, invert scan angle orientation.
    bool inverted;

    /// If true, apply angle compensation to the raw scan data.
    bool angle_compensate;

    /// If true, enable additional scan post-processing logic.
    bool scan_processing;

    /// If true, use a dummy driver instead of real hardware.
    bool dummy_mode;

    /// Preferred scan mode name to be selected on the device.
    std::string scan_mode;

    /// Target motor speed in RPM (0 for device default).
    int rpm = 0;

    /// Maximum range (in meters) to be used when publishing scans.
    float max_distance = 0.0f;

    /// Number of connection retries before giving up.
    int max_retries = 3;

    /**
     * @brief Whether to publish a static TF transform for the LIDAR frame.
     *
     * When enabled, a static transform is broadcast between a parent frame
     * (configured elsewhere) and @ref frame_id.
     */
    bool publish_tf = true;
  } params_;

  // ---------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------

  /// Optional static TF broadcaster for LIDAR frame transforms.
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  /// Concrete driver implementation (real hardware or dummy).
  std::unique_ptr<LidarDriverInterface> driver_;

  /// Lifecycle-aware publisher for LaserScan messages.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_pub_;

  /**
   * @brief Dynamic parameter change callback.
   *
   * Handles runtime updates of selected parameters (if enabled).
   *
   * @param parameters List of parameters to be updated.
   * @return Result indicating whether the update is accepted or rejected.
   */
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

  /// Handle for the dynamic parameter callback registration.
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  /// Diagnostic updater instance used to report node and device health.
  diagnostic_updater::Updater diagnostic_updater_;

  /// Background thread responsible for running @ref scan_loop().
  std::thread scan_thread_;

  /// Flag indicating whether the scan loop should be running.
  std::atomic<bool> is_scanning_{false};

  /// share fsm state across threads
  std::atomic<DriverState> current_state_{DriverState::CONNECTING};

  /// Mutex protecting access to the driver instance from multiple threads.
  std::mutex driver_mutex_;

  /// Cached device information string for diagnostics and logging.
  std::string cached_device_info_ = "N/A";

  /// Cached maximum range (in meters) derived from the driver profile.
  float cached_current_max_range_ = 12.0f;
};

#endif // RPLIDAR_NODE_HPP_
