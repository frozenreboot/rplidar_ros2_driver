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
 * @file lidar_driver_wrapper.cpp
 * @brief Implementation of the LidarDriverInterface for Slamtec RPLidar.
 * @author frozenreboot (frozenreboot@gmail.com)
 * @date 2025-12-07
 */

#include "lidar_driver_wrapper.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// ==========================================
// Real Lidar Driver Implementation
// ==========================================

RealLidarDriver::RealLidarDriver() {
    // Instantiate the underlying SDK driver
    drv_ = *sl::createLidarDriver();
}

RealLidarDriver::~RealLidarDriver() {
    // Clean up SDK resources
    if (drv_) {
        delete drv_;
        drv_ = nullptr;
    }
}

bool RealLidarDriver::connect(const std::string& port, sl_u32 baudrate) {
    if (!drv_) return false;

    // Create a serial channel and attempt connection
    sl::IChannel* channel = *sl::createSerialPortChannel(port, baudrate);
    if (SL_IS_FAIL(drv_->connect(channel))) {
        return false;
    }
    return true;
}

void RealLidarDriver::disconnect() {
    if (drv_) {
        drv_->disconnect();
    }
}

bool RealLidarDriver::start_motor() {
    if (!drv_) return false;

    // Set motor speed (PWM) and start standard scanning mode
    drv_->setMotorSpeed(); 
    sl_result ans = drv_->startScan(0, 1);
    return SL_IS_OK(ans);
}

void RealLidarDriver::stop_motor() {
    if (drv_) {
        drv_->stop();
        drv_->setMotorSpeed(0);
    }
}

bool RealLidarDriver::grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) {
    if (!drv_) return false;

    // Buffer for raw SDK data (8192 is a safe upper bound for single scan points)
    sl_lidar_response_measurement_node_hq_t nodes_array[8192];
    size_t count = sizeof(nodes_array) / sizeof(nodes_array[0]);

    // Blocking call to fetch scan data
    sl_result op_result = drv_->grabScanDataHq(nodes_array, count);

    if (SL_IS_OK(op_result)) {
        // Optimize data (sort by angle, etc.)
        drv_->ascendScanData(nodes_array, count);
        
        // Copy to output vector
        nodes.clear();
        nodes.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            nodes.push_back(nodes_array[i]);
        }
        return true;
    }
    return false;
}


// ==========================================
// Dummy Lidar Driver Implementation
// ==========================================

bool DummyLidarDriver::connect(const std::string& port, sl_u32 baudrate) {
    // Suppress unused parameter warnings
    (void)port; (void)baudrate;
    // Always succeed for dummy mode
    return true; 
}

void DummyLidarDriver::disconnect() {
    // No-op for dummy driver
}

bool DummyLidarDriver::start_motor() {
    // Immediately "start"
    return true; 
}

void DummyLidarDriver::stop_motor() {
    // No-op
}

bool DummyLidarDriver::grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) {
    // Generate synthetic data (Sine wave wall pattern)
    nodes.clear();
    int count = 360; // 1 point per degree
    nodes.reserve(count);

    // Static phase to animate the wave over time
    static float phase = 0.0f;
    phase += 0.1f;

    for (int i = 0; i < count; ++i) {
        sl_lidar_response_measurement_node_hq_t node;
        
        // Calculate angle in SDK units (Q14 fixed point)
        // SDK: angle_q14 = degree * 16384 / 90
        node.angle_z_q14 = ((float)i * 16384.0f / 90.0f); 
        
        // Generate distance: 2 meters + sine wave fluctuation
        float dist_meters = 2.0f + 0.5f * sin((float)i * M_PI / 180.0f + phase);
        
        // Calculate distance in SDK units (mm * 4)
        node.dist_mm_q2 = (sl_u32)(dist_meters * 1000.0f * 4.0f);
        
        node.quality = 10 << 2; // Arbitrary quality > 0
        nodes.push_back(node);
    }
    
    // Simulate hardware scanning delay (10Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}