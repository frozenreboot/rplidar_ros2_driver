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

#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#elif __has_include(<tf2/LinearMath/Quaternion.h>)
#include <tf2/LinearMath/Quaternion.h>
#else
#error "No tf2 Quaternion header found"
#endif
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#elif __has_include(<tf2_ros/static_transform_broadcaster.h>)
#include <tf2_ros/static_transform_broadcaster.h>
#else
#error "No tf2_ros static_transform_broadcaster header found"
#endif

#include <algorithm>
#include <cmath>
#include <limits>

#include <atomic>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <vector>

#include "lidar_driver_wrapper.hpp"

namespace rplidar_driver {

enum class DriverState {
  CONNECTING,
  CHECK_HEALTH,
  WARMUP,
  RUNNING,
  RESETTING,
  STANDBY
};
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

  /**
   * @brief Custom point class for RPLidar points
   */
  struct RpPoint {
    double angle_rad;
    double dist_m;
    double x, y;
    float intensity;
  };

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
  static constexpr double Q14_TO_RAD = (90.0 / 16384.0) * (M_PI / 180.0);
  static constexpr double TWO_PI = 2.0 * M_PI;
  /**
   * @brief Initialize and validate node parameters.
   *
   * This method declares and reads all configuration parameters into
   * @ref params_, applying defaults where necessary.
   */
  void init_parameters();

  /**
   * @brief Declare and get a single variable
   *
   * Declare the parameter and immediately assign its value to the variable
   * The current value of 'variable' is used as the default value
   */
  template <typename T> void init_param(const std::string &name, T &variable) {
    variable = this->declare_parameter<T>(name, variable);
  }

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
   * @brief Count the subscribers the scan thread is producing data for.
   *
   * Includes the PointCloud2 subscribers when @c publish_point_cloud is
   * enabled, so that a cloud-only consumer keeps the motor spinning.
   *
   * @return Total number of connected subscribers.
   */
  size_t count_output_subscribers() const;

  /**
   * @brief Common guard shared by the two manual standby services.
   *
   * A manual request is refused when @c auto_standby owns the motor, or when
   * the node is not ACTIVE and the scan loop that applies the request is
   * therefore not running.
   *
   * @param service_name Service name, used for logging only.
   * @param[out] response Filled in with the reason when the request is
   *             refused; left untouched otherwise.
   * @return true if the request was refused and the caller should return.
   */
  bool standby_request_refused(const char *service_name,
                               std_srvs::srv::Trigger::Response &response);

  /**
   * @brief Service callback requesting the STANDBY state (motor off).
   *
   * The request is only recorded here; the actual transition is performed by
   * @ref scan_loop() so that the FSM stays owned by a single thread. The
   * response therefore reports whether the request was accepted, not whether
   * the motor has already spun down.
   */
  void handle_stop_motor(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service callback leaving the STANDBY state (motor on).
   *
   * @see handle_stop_motor()
   */
  void handle_start_motor(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Publish LaserScan & PointCloud2 (if enabled) messages.
   *
   * @param nodes         Measurement nodes acquired from the driver.
   * @param time          Timestamp of the scan.
   * @param scan_duration Duration of the scan.
   */
  void publish_scan(
      const std::vector<sl_lidar_response_measurement_node_hq_t> &nodes,
      rclcpp::Time time, rclcpp::Duration duration);

  /**
   * @brief Diagnostic callback used by @ref diagnostic_updater_.
   *
   * This method populates the diagnostic status with the current health
   * information obtained from the driver and internal state.
   *
   * @param stat Diagnostic status wrapper to be filled.
   */
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /**
   * @brief Helper to solve abstract ray intersections with the
   *
   * Basically converts two consecutive original readings to a line segment, and
   * looks for the intersection point with a fixed angle.
   *
   * @param p1 First point from the lidar measurement array
   * @param p2 Second point from the lidar measurement array, must a be bigger
   *           angle
   * @param eps A small value like EPSILON to check |d| > 0, default: 1e-9
   */

  // ---------------------------------------------------------------------
  // Inline helpers
  // ---------------------------------------------------------------------
  inline float raySegmentIntersection(double angle, const RpPoint &p1,
                                      const RpPoint &p2, double eps = 1e-9) {

    const double vx = std::cos(angle);
    const double vy = std::sin(angle);
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;

    // Determinant (v cross w)
    const double det = vx * dy - vy * dx;

    if (std::abs(det) < eps)
      return std::numeric_limits<float>::infinity();

    // t: distance along the ray, R(t) = origin + t*v
    // u: distance along the segment, S(u) = p1 + u*w
    //  t*v = p1 + u*w
    double t = (p1.x * dy - p1.y * dx) / det;
    double u = (p1.x * vy - p1.y * vx) / det;

    if (t > 0 && u >= 0.0 && u <= 1.0) {
      return static_cast<float>(t);
    }

    return std::numeric_limits<float>::infinity();
  }

  inline uint8_t scaleIntensity(float intensity, float min, float max) {
    const float denom = max - min;
    if (denom <= 0.0f) {
      return 0;
    }

    const float scaled = (intensity - min) * (254.0f / denom);

    return static_cast<uint8_t>(std::clamp(scaled, 0.0f, 254.0f));
  }

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
    /// The most natural default value without udev
    /// The YAML file default vaule is "/dev/rplidar"
    std::string serial_port;

    /// Serial baudrate used to communicate with the RPLIDAR.
    int serial_baudrate;

    /// Frame ID used in published ROS messages.
    std::string frame_id = "laser_frame";

    /// If true, apply angle compensation to the raw scan data.
    bool angle_compensate = true;

    /// If true, use a dummy driver instead of real hardware.
    bool dummy_mode = false;

    /// Preferred scan mode name to be selected on the device.
    std::string scan_mode = "";

    /// Target motor speed in RPM (0 for device default).
    int rpm = 0;

    /// Maximum range (in meters) to be used when publishing scans.
    float max_distance = 0.0f;

    /// Angle offset in degrees to compensate for hardware zero-point mismatch.
    double angle_offset = 0.0;

    /// Number of connection retries before giving up.
    int max_retries = 3;

    int computed_ray_count = 1450;

    /**
     * @brief Whether to publish a static TF transform for the LIDAR frame.
     *
     * When enabled, a static transform is broadcast between a parent frame
     * (configured elsewhere) and @ref frame_id.
     */
    bool publish_tf = true;

    // If true, publishes a PointCloud2 typed topic
    bool publish_point_cloud = false;

    // If true, uses interpolation to compute the LaserScan message
    bool interpolated_rays = false;

    // If true, fills the intensities in the LaserScan message
    bool use_intensities = false;

    // If true, uses intensities array as actual angles reported by the driver.
    // use_intensities parameter must be enabled to use this.
    bool intensities_as_angles = false;

    // QoS policy for the publishers
    std::string qos_policy = "sensor_data";

    /**
     * @brief Automatically enter STANDBY while nobody subscribes.
     *
     * When enabled, the node stops the motor as soon as the last subscriber
     * of the scan (and, if enabled, cloud) topic disconnects, and restarts it
     * when a subscriber appears again.
     *
     * While this is enabled the @c stop_motor / @c start_motor services are
     * rejected: the subscriber count is the single source of truth.
     */
    bool auto_standby = false;
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

  /// Lifecycle-aware publisher for PointCloud2 messages.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_pub_;

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

  /// Service putting the device into STANDBY.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_motor_service_;

  /// Service leaving STANDBY.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_motor_service_;

  /// Diagnostic updater instance used to report node and device health.
  diagnostic_updater::Updater diagnostic_updater_;

  /// Background thread responsible for running @ref scan_loop().
  std::thread scan_thread_;

  /// Flag indicating whether the scan loop should be running.
  std::atomic<bool> is_scanning_{false};

  /// share fsm state across threads
  std::atomic<DriverState> current_state_{DriverState::CONNECTING};

  /// Standby explicitly requested through the services, consumed by
  /// @ref scan_loop() on each iteration.
  std::atomic<bool> standby_requested_{false};

  /// Runtime-updatable mirror of @ref Parameters::auto_standby, read by the
  /// scan thread and written by the parameter callback.
  std::atomic<bool> auto_standby_enabled_{false};

  /// True while STANDBY is held because no subscriber is connected.
  /// Written by the scan thread, read by the diagnostics callback.
  std::atomic<bool> auto_standby_engaged_{false};

  /// Mutex protecting access to the driver instance from multiple threads.
  std::mutex driver_mutex_;

  /// Cached device information string for diagnostics and logging.
  std::string cached_device_info_ = "N/A";

  /// Cached maximum range (in meters) derived from the driver profile.
  float cached_current_max_range_ = 12.0f;

  // For newer models
  bool is_new_protocol_ = false;

  // For some newer models, rpm should be set after grabbing the first scan.
  bool initial_reset_required_ = true;
};

} // namespace rplidar_driver

#endif // RPLIDAR_NODE_HPP_
