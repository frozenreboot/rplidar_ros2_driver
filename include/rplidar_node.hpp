/*
 * RPLIDAR ROS2 DRIVER
 *
 * Copyright (c) 2025, frozenreboot
 * Copyright (c) 2009 - 2014 RoboPeak Team
 * Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *
 * This project is a refactored version of the original rplidar_ros package.
 * The architecture has been redesigned to support ROS2 Lifecycle and Multithreading.
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
 * @brief ROS2 Lifecycle Node definition for the Slamtec RPLidar.
 * @details Implements a managed lifecycle node that handles configuration, activation, 
 * and data publishing. It utilizes a separate thread for non-blocking data polling.
 * @author frozenreboot (frozenreboot@gmail.com)
 * @date 2025-12-07
 */

#ifndef RPLIDAR_NODE_HPP_
#define RPLIDAR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

#include "lidar_driver_wrapper.hpp" 

/**
 * @class RPlidarNode
 * @brief A ROS2 Lifecycle Node for interfacing with Slamtec RPLidar devices.
 * * This node manages the driver lifecycle (Configure -> Activate -> Deactivate -> Cleanup).
 * It delegates hardware interaction to `LidarDriverInterface` and runs the data polling 
 * loop in a dedicated background thread to maintain system responsiveness.
 */
class RPlidarNode final : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Construct a new RPlidarNode object.
     * @param options ROS2 Node options.
     */
    explicit RPlidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Destructor. Ensures the scanning thread is properly joined before destruction.
     */
    ~RPlidarNode() override;

    // --- Lifecycle Callbacks ---

    /**
     * @brief Transitions the node to the CONFIGURED state.
     * @details Reads ROS parameters and attempts to connect to the Lidar hardware 
     * using the specified serial port and baud rate.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Transitions the node to the ACTIVE state.
     * @details Starts the Lidar motor, activates the publisher, and launches 
     * the background scanning thread (`scan_loop`).
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Transitions the node to the INACTIVE state.
     * @details Stops the scanning thread and the Lidar motor, but maintains the hardware connection.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Transitions the node to the UNCONFIGURED state.
     * @details Disconnects from the hardware and releases memory resources (driver & publisher).
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Handles the SHUTDOWN transition.
     * @details Performs same cleanup operations as `on_cleanup`.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    // --- Private Implementations ---

    /**
     * @brief Initializes and declares ROS2 parameters.
     */
    void init_parameters();
    
    /**
     * @brief The worker loop function executed by the scanning thread.
     * @details Continuously polls the driver for new scan data and triggers publishing
     * as long as the node is active. Implements thread sleeping on failure
     * to prevent CPU busy-waiting.
     */
    void scan_loop(); 
    
    /**
     * @brief Converts raw driver data to ROS2 LaserScan message and publishes it.
     * @param nodes Raw scan points retrieved from the driver.
     * @param start_time Timestamp of the scan start.
     * @param scan_duration Duration of the scan in seconds.
     */
    void publish_scan(const std::vector<sl_lidar_response_measurement_node_hq_t>& nodes, 
                      rclcpp::Time start_time, double scan_duration);

    // --- Member Variables ---

    /**
     * @brief Configuration parameters structure.
     */
    struct Parameters {
        std::string serial_port;  ///< Path to the serial device (e.g., /dev/ttyUSB0).
        int serial_baudrate;      ///< Baud rate for serial communication.
        std::string frame_id;     ///< TF frame ID for the LaserScan message.
        bool inverted;            ///< Whether to invert the scan data (upside down).
        bool angle_compensate;    ///< Enable hardware angle compensation.
        bool dummy_mode;          ///< If true, uses DummyLidarDriver instead of RealLidarDriver.
        std::string scan_mode;    ///< Specific scan mode (e.g., Standard, Express).
    } params_;

    /// @brief Polymorphic pointer to the Lidar driver (Real or Dummy).
    std::unique_ptr<LidarDriverInterface> driver_; 

    /// @brief Lifecycle-managed publisher for LaserScan messages.
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    /// @brief Thread for background data polling.
    std::thread scan_thread_;

    /// @brief Atomic flag to control the execution of the scan thread.
    std::atomic<bool> is_scanning_{false}; 
};

#endif // RPLIDAR_NODE_HPP_