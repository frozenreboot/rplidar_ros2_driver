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
 * @date 2025-12-13
 */

#include "rplidar_node.hpp"
#include <chrono>
#include <cmath>
#include <thread>

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

    RCLCPP_INFO(this->get_logger(), "RPlidarNode created. Ready for Lifecycle.");
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

RPlidarNode::CallbackReturn RPlidarNode::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Configuring node...");
    init_parameters(); 

    // Factory Pattern: Create Real or Dummy driver based on parameter
    if (params_.dummy_mode) {
        RCLCPP_WARN(this->get_logger(), "!!! DUMMY MODE ENABLED !!!");
        driver_ = std::make_unique<DummyLidarDriver>();
    } else {
        driver_ = std::make_unique<RealLidarDriver>();
    }

    // Initial connection check (Non-blocking failure permitted)
    // Even if this fails, the FSM in scan_loop will keep retrying.
    if (!driver_->connect(params_.serial_port, params_.serial_baudrate)) {
        RCLCPP_ERROR(this->get_logger(), "Initial connection failed at %s. Will retry in scan loop.", params_.serial_port.c_str());
    }

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_activate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "Activating node...");
    LifecycleNode::on_activate(state); // Activate Publisher

    // Launch the background scanning thread
    is_scanning_ = true;
    scan_thread_ = std::thread(&RPlidarNode::scan_loop, this);

    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_deactivate(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "Deactivating node...");
    
    // Stop the thread
    is_scanning_ = false; 
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }

    // Stop the motor
    if (driver_) {
        driver_->stop_motor();
    }

    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
    scan_pub_.reset();
    if (driver_) {
        driver_->disconnect();
        driver_.reset();
    }
    return CallbackReturn::SUCCESS;
}

RPlidarNode::CallbackReturn RPlidarNode::on_shutdown(const rclcpp_lifecycle::State & state) {
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    return on_cleanup(state);
}

// ========================================================================================
// Private Methods & The Zombie Loop
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
// rplidar_node.cpp 의 scan_loop 함수 전체를 이걸로 교체하세요.

void RPlidarNode::scan_loop() {
    // [The Zombie State Machine]
    enum class State { 
        CONNECTING,     // 1. 연결 시도
        CHECK_HEALTH,   // 2. 건강 체크 (순서 변경됨!)
        WARMUP,         // 3. 모터 가동 및 스캔 시작
        RUNNING,        // 4. 데이터 수신
        RESETTING       // 5. 에러 시 리셋
    };

    State state = State::CONNECTING;
    int error_count = 0;
    const int MAX_ERROR_COUNT = 10;

    RCLCPP_INFO(this->get_logger(), ">>> Scan Loop & FSM Started <<<");

    while (rclcpp::ok() && is_scanning_) {
        switch (state) {
            // ---------------------------------------------------------
            // State 1: CONNECTING
            // ---------------------------------------------------------
            case State::CONNECTING:
                if (driver_->isConnected()) {
                    // [수정 1] 연결되면 바로 WARMUP으로 가지 말고, CHECK_HEALTH로 보냅니다.
                    RCLCPP_INFO(this->get_logger(), "[FSM] Connected. Checking Health FIRST...");
                    state = State::CHECK_HEALTH; 
                  } else {
                    if (driver_->connect(params_.serial_port, params_.serial_baudrate)) {
                        // [수정 2] 여기서도 연결 성공하면 CHECK_HEALTH로 보냅니다.
                        RCLCPP_INFO(this->get_logger(), "[FSM] Connected. Checking Health FIRST...");
                        state = State::CHECK_HEALTH;
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "[FSM] Connection failed. Retrying...");
                        std::this_thread::sleep_for(1000ms);
                    }
                }
                break;

            // ---------------------------------------------------------
            // State 2: CHECK_HEALTH (위치가 여기로 옴)
            // ---------------------------------------------------------
            case State::CHECK_HEALTH:
                {
                    // 아직 스캔 명령을 안 내렸으니 라이다는 조용한 상태입니다. 이때 물어봐야 합니다.
                    int health = driver_->getHealth(); 
                    if (health == 0 || health == 1) { 
                        RCLCPP_INFO(this->get_logger(), "[FSM] Health OK. Now Warming up...");
                        // [수정 3] 건강하면 이제 WARMUP(스캔 시작)으로 보냅니다.
                        state = State::WARMUP; 
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "[FSM] Health ERROR! Initiating Reset.");
                        state = State::RESETTING;
                    }
                }
                break;

            // ---------------------------------------------------------
            // State 3: WARMUP (모터 켜고 스캔 명령 내리는 곳)
            // ---------------------------------------------------------
            case State::WARMUP:
                RCLCPP_INFO(this->get_logger(), "[FSM] Starting Motor & Scan...");
                // 여기서 start_motor(내부적으로 startScanExpress)를 호출합니다.
                if (driver_->start_motor()) {
                    // S3는 기동 시 전압 안정화가 필요합니다. 1초 대기.
                    std::this_thread::sleep_for(1000ms);
                    RCLCPP_INFO(this->get_logger(), "[FSM] Scan Started. Entering Running State.");
                    // [수정 4] 스캔 켜졌으니 이제 데이터 받으러 RUNNING으로 갑니다.
                    state = State::RUNNING; 
                    error_count = 0;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[FSM] Failed to start motor. Resetting.");
                    state = State::RESETTING;
                }
                break;

            // ---------------------------------------------------------
            // State 4: RUNNING (데이터 줍줍)
            // ---------------------------------------------------------
            case State::RUNNING:
                {
                    std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
                    rclcpp::Time start_time = this->now();

                    // Poll for data
                    if (driver_->grab_scan_data(nodes)) {
                        double duration = (this->now() - start_time).seconds();
                        publish_scan(nodes, start_time, duration);
                        error_count = 0; 
                    } else {
                        error_count++;
                        if (error_count > MAX_ERROR_COUNT) {
                            RCLCPP_ERROR(this->get_logger(), "[FSM] Too many grab failures (%d). Hardware Unresponsive!", error_count);
                            state = State::RESETTING;
                        } else {
                            // CPU 점유율 방지용 미세 딜레이
                            std::this_thread::sleep_for(1ms);
                        }
                    }
                }
                break;

            // ---------------------------------------------------------
            // State 5: RESETTING
            // ---------------------------------------------------------
            case State::RESETTING:
                RCLCPP_WARN(this->get_logger(), "[FSM] Resetting Hardware...");
                driver_->stop_motor(); 
                driver_->reset();      
                
                // MCU 재부팅 대기
                std::this_thread::sleep_for(2000ms);
                
                driver_->disconnect();
                // [수정 5] 리셋 후에는 다시 연결부터 시도
                state = State::CONNECTING;
                error_count = 0;
                break;
        }

        // 루프 전체 딜레이 (CPU 과부하 방지)
        if (state != State::RUNNING) {
             std::this_thread::sleep_for(10ms);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Scan Loop Terminated.");
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
    
    // Calculate increment based on actual data count
    scan_msg->angle_increment = (2.0 * M_PI) / (double)(count > 1 ? count - 1 : 1);
    scan_msg->scan_time = scan_duration;
    scan_msg->time_increment = scan_duration / (double)(count > 1 ? count - 1 : 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = 12.0;

    scan_msg->ranges.resize(count);
    scan_msg->intensities.resize(count);

    for (size_t i = 0; i < count; ++i) {
        float dist = nodes[i].dist_mm_q2 / 4000.0f; // mm to meters
        scan_msg->ranges[i] = (dist == 0.0f) ? std::numeric_limits<float>::infinity() : dist;
        scan_msg->intensities[i] = (float)(nodes[i].quality >> 2);
    }

    scan_pub_->publish(std::move(scan_msg));
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPlidarNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}