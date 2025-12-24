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
 * @file rplidar_node.cpp
 * @brief Implementation of the RPLidar Lifecycle Node.
 * @details Handles ROS2 parameter loading, driver instantiation (Factory Pattern),
 * and the main scanning loop within a dedicated thread.
 * @author frozenreboot (frozenreboot@gmail.com)
 * @date 2025-12-24
 */

#include "rplidar_node.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include <limits>

using namespace std::chrono_literals;

RPlidarNode::RPlidarNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("rplidar_node", options) {
    
    // Declare ROS2 Parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 1000000);
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<bool>("inverted", false);
    this->declare_parameter<bool>("angle_compensate", true);
    this->declare_parameter<bool>("dummy_mode", false); 
    this->declare_parameter<std::string>("scan_mode", "");

    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Node created. Waiting for configuration.");
}

RPlidarNode::~RPlidarNode() {
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Destroying node resources...");

    // 1. Terminate the scanning thread safely
    is_scanning_ = false;
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }

    // 2. Stop motor and disconnect driver
    if (driver_) {
        RCLCPP_INFO(this->get_logger(), "[Driver] Stopping motor during destruction...");
        driver_->stop_motor();
        driver_->disconnect();
    }
    
    // driver_ is a unique_ptr, so memory is automatically released here.
}

// ========================================================================================
// Lifecycle Callbacks
// ========================================================================================

RPlidarNode::CallbackReturn RPlidarNode::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Configuring node...");
    init_parameters(); 

    // Factory Pattern: Instantiate the appropriate driver based on configuration
    if (params_.dummy_mode) {
        RCLCPP_WARN(this->get_logger(), "!!! DUMMY MODE ENABLED - USING SYNTHETIC DATA !!!");
        driver_ = std::make_unique<DummyLidarDriver>();
    } else {
        driver_ = std::make_unique<RealLidarDriver>();
    }

    // Initial connection attempt (Non-blocking)
    // The internal FSM will handle reconnection if this fails.
    if (!driver_->connect(params_.serial_port, params_.serial_baudrate)) {
        RCLCPP_WARN(this->get_logger(), 
            "[Driver] Initial connection failed at %s. FSM will retry in the loop.", 
            params_.serial_port.c_str());
    }

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_activate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Activating node...");
    LifecycleNode::on_activate(state); // Activate Publisher

    // Launch the background scanning thread
    is_scanning_ = true;
    scan_thread_ = std::thread(&RPlidarNode::scan_loop, this);

    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_deactivate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Deactivating node...");
    
    // Stop the scanning thread
    is_scanning_ = false; 
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }

    // Stop the motor to save power and safety
    if (driver_) {
        driver_->stop_motor();
    }

    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Cleaning up resources...");
    scan_pub_.reset();
    if (driver_) {
        driver_->disconnect();
        driver_.reset();
    }
    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_shutdown(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "[Lifecycle] Shutting down...");
    return on_cleanup(state);
}

// ========================================================================================
// Private Methods & Fault Tolerance FSM
// ========================================================================================

void RPlidarNode::init_parameters() {
    this->get_parameter("serial_port", params_.serial_port);
    this->get_parameter("serial_baudrate", params_.serial_baudrate);
    this->get_parameter("frame_id", params_.frame_id);
    this->get_parameter("inverted", params_.inverted);
    this->get_parameter("angle_compensate", params_.angle_compensate);
    this->get_parameter("dummy_mode", params_.dummy_mode);
    this->get_parameter("scan_mode", params_.scan_mode);
}

/**
 * @brief The main acquisition loop running in a separate thread.
 * Implements a Finite State Machine (FSM) to handle connection, health checks, 
 * motor warmup, and error recovery (reset) automatically.
 */
void RPlidarNode::scan_loop() {
    enum class DriverState { 
        CONNECTING,     ///< Attempting to open serial port
        CHECK_HEALTH,   ///< Querying device health before operation
        WARMUP,         ///< Starting motor and stabilizing rotation
        RUNNING,        ///< Normal data acquisition
        RESETTING       ///< Recovering from hardware errors
    };

    DriverState state = DriverState::CONNECTING;
    int error_count = 0;
    const int MAX_ERROR_COUNT = 10;

    RCLCPP_INFO(this->get_logger(), "[FSM] Scan Loop Started.");

    while (rclcpp::ok() && is_scanning_) {
        switch (state) {
            // ---------------------------------------------------------
            // State 1: CONNECTING
            // ---------------------------------------------------------
            case DriverState::CONNECTING:
                if (driver_->isConnected()) {
                    RCLCPP_INFO(this->get_logger(), "[FSM] Connection verified. Transition to HEALTH_CHECK.");
                    state = DriverState::CHECK_HEALTH; 
                } else {
                    if (driver_->connect(params_.serial_port, params_.serial_baudrate)) {
                        RCLCPP_INFO(this->get_logger(), "[FSM] Connection Established. Transition to HEALTH_CHECK.");
                        state = DriverState::CHECK_HEALTH;
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "[FSM] Connection failed. Retrying...");
                        std::this_thread::sleep_for(1000ms);
                    }
                }
                break;

            // ---------------------------------------------------------
            // State 2: CHECK_HEALTH
            // ---------------------------------------------------------
            case DriverState::CHECK_HEALTH:
                {
                    // Check health before starting the motor to avoid damaging the device
                    int health = driver_->getHealth(); 
                    if (health == 0 || health == 1) { // 0: OK, 1: Warning
                        RCLCPP_INFO(this->get_logger(), "[FSM] Health OK. Transition to WARMUP.");
                        state = DriverState::WARMUP; 
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "[FSM] Device Health Error! Status: %d. Initiating Reset.", health);
                        state = DriverState::RESETTING;
                    }
                }
                break;

            // ---------------------------------------------------------
            // State 3: WARMUP
            // ---------------------------------------------------------
            case DriverState::WARMUP:
                RCLCPP_INFO(this->get_logger(), "[FSM] Starting Motor & Configuring Scan Mode...");
                
                if (driver_->start_motor()) {
                    // S-Series needs stabilization time for voltage/speed
                    std::this_thread::sleep_for(1000ms);
                    RCLCPP_INFO(this->get_logger(), "[FSM] Motor Started. Transition to RUNNING.");
                    state = DriverState::RUNNING; 
                    error_count = 0;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[FSM] Failed to start motor. Retrying via Reset.");
                    state = DriverState::RESETTING;
                }
                break;

            // ---------------------------------------------------------
            // State 4: RUNNING
            // ---------------------------------------------------------
            case DriverState::RUNNING:
                {
                    std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
                    rclcpp::Time start_time = this->now();

                    // Blocking call with timeout
                    if (driver_->grab_scan_data(nodes)) {
                        double duration = (this->now() - start_time).seconds();
                        publish_scan(nodes, start_time, duration);
                        error_count = 0; 
                    } else {
                        error_count++;
                        if (error_count > MAX_ERROR_COUNT) {
                            RCLCPP_ERROR(this->get_logger(), "[FSM] Excessive grab failures (%d). Hardware Unresponsive!", error_count);
                            state = DriverState::RESETTING;
                        } else {
                            // Slight yield to prevent CPU spinning on fast failures
                            std::this_thread::sleep_for(1ms);
                        }
                    }
                }
                break;

            // ---------------------------------------------------------
            // State 5: RESETTING
            // ---------------------------------------------------------
            case DriverState::RESETTING:
                RCLCPP_WARN(this->get_logger(), "[FSM] Performing Hardware Reset...");
                driver_->stop_motor(); 
                driver_->reset();      
                
                // Wait for MCU reboot
                std::this_thread::sleep_for(2000ms);
                
                driver_->disconnect();
                
                // Re-initialize from the beginning
                state = DriverState::CONNECTING;
                error_count = 0;
                break;
        }

        // FSM Loop Throttle (Prevent CPU Hogging in non-running states)
        if (state != DriverState::RUNNING) {
             std::this_thread::sleep_for(10ms);
        }
    }
    RCLCPP_INFO(this->get_logger(), "[FSM] Scan Loop Terminated.");
}

void RPlidarNode::publish_scan(const std::vector<sl_lidar_response_measurement_node_hq_t>& nodes,
                               rclcpp::Time start_time, double scan_duration) {
    if (nodes.empty()) return;

    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    size_t count = nodes.size();
    
    scan_msg->header.stamp = start_time;
    scan_msg->header.frame_id = params_.frame_id;
    scan_msg->angle_min = 0.0;
    scan_msg->angle_max = 2.0 * M_PI;
    
    // Calculate angle increment dynamically based on actual point count
    scan_msg->angle_increment = (2.0 * M_PI) / (double)(count > 1 ? count - 1 : 1);
    scan_msg->scan_time = scan_duration;
    scan_msg->time_increment = scan_duration / (double)(count > 1 ? count - 1 : 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = 12.0;

    scan_msg->ranges.resize(count);
    scan_msg->intensities.resize(count);

    for (size_t i = 0; i < count; ++i) {
        // Convert SDK units (dist_mm_q2) to Meters
        float dist = nodes[i].dist_mm_q2 / 4000.0f; 
        scan_msg->ranges[i] = (dist == 0.0f) ? std::numeric_limits<float>::infinity() : dist;
        scan_msg->intensities[i] = (float)(nodes[i].quality >> 2);
    }

    scan_pub_->publish(std::move(scan_msg));
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPlidarNode>();
    
    // Note: Since this is a LifecycleNode, it usually requires an external
    // Lifecycle Manager to transition states (Configure -> Activate).
    // For standalone testing, you might need to auto-activate here or use command line tools.
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}