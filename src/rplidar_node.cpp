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
 * @date 2025-12-07
 */

#include "rplidar_node.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

RPlidarNode::RPlidarNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("rplidar_node", options) {
    
    // Declare parameters with default values
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 115200);
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<bool>("inverted", false);
    this->declare_parameter<bool>("angle_compensate", true);
    this->declare_parameter<bool>("dummy_mode", false); 
    this->declare_parameter<std::string>("scan_mode", "");

    RCLCPP_INFO(this->get_logger(), "RPlidarNode created. Waiting for lifecycle configuration...");
}

RPlidarNode::~RPlidarNode() {
    // Ensure the scanning thread is properly joined before destruction
    if (scan_thread_.joinable()) {
        is_scanning_ = false;
        scan_thread_.join();
    }
}

// ========================================================================================
// Lifecycle Callbacks
// ========================================================================================

/**
 * @brief [Step 1] Configure: Load parameters and connect to hardware.
 */
RPlidarNode::CallbackReturn RPlidarNode::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Configuring node...");

    init_parameters(); 

    // Instantiate driver using Factory Pattern based on 'dummy_mode' parameter
    if (params_.dummy_mode) {
        RCLCPP_WARN(this->get_logger(), "!!! DUMMY MODE ENABLED - No hardware required !!!");
        driver_ = std::make_unique<DummyLidarDriver>();
    } else {
        driver_ = std::make_unique<RealLidarDriver>();
    }

    // Attempt hardware connection
    if (!driver_->connect(params_.serial_port, params_.serial_baudrate)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to LIDAR at %s", params_.serial_port.c_str());
        return CallbackReturn::FAILURE;
    }

    // Initialize publisher (Data transmission is not yet active)
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    RCLCPP_INFO(this->get_logger(), "Configuration successful.");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief [Step 2] Activate: Start motor and launch scanning thread.
 */
RPlidarNode::CallbackReturn RPlidarNode::on_activate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "Activating node...");
    
    // 1. Activate LifecyclePublisher (Must call parent method)
    LifecycleNode::on_activate(state);

    // 2. Start Lidar Motor
    if (!driver_->start_motor()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start motor.");
        return CallbackReturn::FAILURE;
    }

    // 3. Launch the scanning worker thread
    is_scanning_ = true;
    scan_thread_ = std::thread(&RPlidarNode::scan_loop, this);

    RCLCPP_INFO(this->get_logger(), "Lidar scanning started.");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief [Step 3] Deactivate: Stop motor and join scanning thread.
 */
RPlidarNode::CallbackReturn RPlidarNode::on_deactivate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "Deactivating node...");

    // 1. Stop scanning thread
    is_scanning_ = false;
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }

    // 2. Stop Lidar Motor
    if (driver_) {
        driver_->stop_motor();
    }

    // 3. Deactivate LifecyclePublisher
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(this->get_logger(), "Lidar stopped.");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief [Step 4] Cleanup: Release resources and disconnect driver.
 */
RPlidarNode::CallbackReturn RPlidarNode::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
    
    scan_pub_.reset();
    if (driver_) {
        driver_->disconnect();
        driver_.reset();
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief [Step 5] Shutdown: Final cleanup.
 */
RPlidarNode::CallbackReturn RPlidarNode::on_shutdown(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    return on_cleanup(state);
}

// ========================================================================================
// Private Methods
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

void RPlidarNode::scan_loop() {
    // Worker loop running in a separate thread
    while (rclcpp::ok() && is_scanning_) {
        std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
        rclcpp::Time start_scan_time = this->now();

        // 1. Blocking I/O: Fetch data from the driver
        if (driver_->grab_scan_data(nodes)) {
            double scan_duration = (this->now() - start_scan_time).seconds();
            // 2. Publish as ROS2 message
            publish_scan(nodes, start_scan_time, scan_duration);
        } else {
            // Sleep briefly to prevent CPU busy-waiting on failure
            std::this_thread::sleep_for(10ms);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to grab scan data from Lidar.");
        }
    }
}

void RPlidarNode::publish_scan(const std::vector<sl_lidar_response_measurement_node_hq_t>& nodes,
                               rclcpp::Time start_time, double scan_duration) {
    if (nodes.empty()) return;

    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    
    size_t count = nodes.size();
    scan_msg->header.stamp = start_time;
    scan_msg->header.frame_id = params_.frame_id;
    
    // Assuming 360-degree scan
    scan_msg->angle_min = 0.0;
    scan_msg->angle_max = 2.0 * M_PI;
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(count - 1);
    scan_msg->scan_time = scan_duration;
    scan_msg->time_increment = scan_duration / (double)(count - 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = 12.0;

    scan_msg->ranges.resize(count);
    scan_msg->intensities.resize(count);

    for (size_t i = 0; i < count; ++i) {
        float read_dist = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
        
        // Note: Real application might need angle-based indexing mapping here.
        // For MVP, we map sequentially.
        if (read_dist == 0.0) {
            scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
        } else {
            scan_msg->ranges[i] = read_dist;
        }
        scan_msg->intensities[i] = (float)(nodes[i].quality >> 2);
    }

    scan_pub_->publish(std::move(scan_msg));
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RPlidarNode>();
    
    // MultiThreadedExecutor is recommended for Lifecycle nodes handling service callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}