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
 * @details Implements secure and optimized hardware interactions using the Slamtec SDK.
 * @author frozenreboot (frozenreboot@gmail.com)
 * @date 2025-12-13
 */

#include "lidar_driver_wrapper.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

using namespace sl;

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

    // Create serial channel
    IChannel* channel = *createSerialPortChannel(port, baudrate);
    if (!channel) return false;

    // Attempt connection
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

bool RealLidarDriver::isConnected() {
    if (!drv_) return false;
    return drv_->isConnected();
}

int RealLidarDriver::getHealth() {
    if (!isConnected()) return 2; // Error

    sl_lidar_response_device_health_t health_info;
    auto res = drv_->getHealth(health_info);

    if (SL_IS_FAIL(res)) return 2; // Communication Failed

    if (health_info.status == SL_LIDAR_STATUS_ERROR) return 2;
    if (health_info.status == SL_LIDAR_STATUS_WARNING) return 1;
    return 0; // OK
}

void RealLidarDriver::reset() {
    if (isConnected()) {
        drv_->reset();
    }
}

bool RealLidarDriver::start_motor() {
    // Strict check: Must be connected to start motor
    if (!isConnected()) return false;

    // Set motor speed (PWM) and start standard scanning mode
    drv_->setMotorSpeed(); 
    auto res = drv_->startScan(0, 1);
    return SL_IS_OK(res);
}

void RealLidarDriver::stop_motor() {
    if (isConnected()) {
        drv_->stop();
        drv_->setMotorSpeed(0);
    }
}

bool RealLidarDriver::grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) {
    // Strict check
    if (!isConnected()) return false;

    // Buffer for raw SDK data (8192 is a safe upper bound for single scan points)
    sl_lidar_response_measurement_node_hq_t raw_nodes[8192];
    size_t count = sizeof(raw_nodes) / sizeof(raw_nodes[0]);

    // Blocking call to fetch scan data (with internal timeout)
    auto res = drv_->grabScanDataHq(raw_nodes, count);

    if (SL_IS_OK(res)) {
        // Optimize data order (sort by angle, etc.)
        drv_->ascendScanData(raw_nodes, count);
        
        // [Optimization] Mass copy using assign instead of loop
        // This utilizes memory copy for POD types, much faster than push_back loop.
        nodes.assign(raw_nodes, raw_nodes + count);
        return true;
    }
    return false;
}


// ==========================================
// Dummy Lidar Driver Implementation
// ==========================================

bool DummyLidarDriver::connect(const std::string& port, sl_u32 baudrate) {
    (void)port; (void)baudrate; // Suppress unused warnings
    return true; // Always succeed
}

void DummyLidarDriver::disconnect() {
    // No-op
}

bool DummyLidarDriver::isConnected() {
    return true;
}

int DummyLidarDriver::getHealth() {
    return 0; // Always Healthy
}

void DummyLidarDriver::reset() {
    // No-op
}

bool DummyLidarDriver::start_motor() {
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

    static float phase = 0.0f;
    phase += 0.1f;

    for (int i = 0; i < count; ++i) {
        sl_lidar_response_measurement_node_hq_t node;
        
        // SDK Unit: angle_q14 = degree * 16384 / 90
        node.angle_z_q14 = ((float)i * 16384.0f / 90.0f); 
        
        // SDK Unit: dist_mm_q2 = mm * 4
        // Logic: 2 meters + sine wave fluctuation
        float dist_meters = 2.0f + 0.5f * sin((float)i * M_PI / 180.0f + phase);
        node.dist_mm_q2 = (sl_u32)(dist_meters * 1000.0f * 4.0f);
        
        node.quality = 10 << 2; 
        nodes.push_back(node);
    }
    
    // Simulate hardware scanning delay (10Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}