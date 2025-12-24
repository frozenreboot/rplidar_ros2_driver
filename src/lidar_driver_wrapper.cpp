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
 * @date 2025-12-24
 */


#include "lidar_driver_wrapper.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>

using namespace sl;

// ============================================================================
// Real Lidar Driver Implementation
// ============================================================================

RealLidarDriver::RealLidarDriver() {
    // Instantiate the underlying SDK driver
    drv_ = *sl::createLidarDriver();
}

RealLidarDriver::~RealLidarDriver() {
    // Ensure clean shutdown of the driver instance
    if (drv_) {
        // 1. Stop the motor if connected
        if (drv_->isConnected()) {
            drv_->stop();
            drv_->setMotorSpeed(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }

        // 2. Disconnect from the serial port
        drv_->disconnect();

        // 3. Release memory
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
    auto ans = drv_->connect(channel);
    if (SL_IS_FAIL(ans)) return false;

    // Retrieve device information
    auto res = drv_->getDeviceInfo(devinfo_);
    if (SL_IS_FAIL(res)) {
        std::cerr << "[Driver] Failed to get device info!" << std::endl;
        return false;
    }
    
    std::cout << "[Driver] Connected to RPLIDAR Model ID: " << (int)(devinfo_.model >> 4) << std::endl;
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
    if (!isConnected()) return 2; // Error state

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

/**
 * @brief Starts the motor and configures the scan mode.
 * Automatically selects 'DenseBoost' for S-Series or falls back to standard modes.
 * @return true if motor started and scan mode set successfully.
 */
bool RealLidarDriver::start_motor() {
    if (!isConnected()) return false;

    // --------------------------------------------------------
    // Step 1. Motor Control Strategy
    // --------------------------------------------------------
    if (is_s_series()) {
        // [S/T Series] Internal closed-loop control
        drv_->setMotorSpeed(); 
    } else {
        // [A Series] PWM control (Default: 600)
        drv_->setMotorSpeed(600); 
    }

    // --------------------------------------------------------
    // Step 2. Scan Mode Strategy
    // --------------------------------------------------------
    
    // Safety fallback for A-Series to avoid complicated mode queries
    if (!is_s_series()) {
        std::cout << "[Driver] Detected A-Series. Using Legacy Scan Mode." << std::endl;
        return SL_IS_OK(drv_->startScan(0, 1)); // 0: Auto, 1: Typical
    }

    // --- S/T Series High-Performance Logic ---
    
    std::vector<LidarScanMode> allSupportedScanModes;
    sl_result op_result = drv_->getAllSupportedScanModes(allSupportedScanModes);

    if (SL_IS_FAIL(op_result)) {
        // Fallback if mode query fails
        return SL_IS_OK(drv_->startScan(0, 1));
    }

    // Select optimum mode (Priority: DenseBoost > Last Available > Standard)
    sl_u16 selectedScanMode = sl_u16(-1);
    
    for (const auto& mode : allSupportedScanModes) {
        std::string mode_name = mode.scan_mode;
        // Priority 1: DenseBoost (Max Performance for S3)
        if (mode_name.find("DenseBoost") != std::string::npos) { 
             selectedScanMode = mode.id;
             break;
        }
    }

    // Priority 2: Use the last mode in the list (Usually the newest/best)
    if (selectedScanMode == sl_u16(-1) && !allSupportedScanModes.empty()) {
        selectedScanMode = allSupportedScanModes.back().id;
    }

    // Priority 3: Legacy Fallback
    if (selectedScanMode == sl_u16(-1)) {
        return SL_IS_OK(drv_->startScan(0, 1));
    }

    // Use Express Scan for S-Series
    LidarScanMode scanParams;
    op_result = drv_->startScanExpress(false, selectedScanMode, 0, &scanParams);

    if (SL_IS_OK(op_result)) {
        std::cout << "[Driver] S-Series Mode ID: " << selectedScanMode 
                  << " | Max Dist: " << scanParams.max_distance << "m" << std::endl;
        return true;
    } 
    
    // Final Fallback
    return SL_IS_OK(drv_->startScan(0, 1));
}

void RealLidarDriver::stop_motor() {
    if (isConnected()) {
        drv_->stop();
        drv_->setMotorSpeed(0);
    }
}

/**
 * @brief Retrieves scan data from the LiDAR.
 * Uses a static heap buffer to handle high-density data (e.g., S3 DenseBoost).
 * @param nodes Vector to populate with scan data.
 * @return true if data successfully grabbed.
 */
bool RealLidarDriver::grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) {
    if (!isConnected()) return false;

    // Allocate a large static buffer to prevent stack overflow and frequent reallocations.
    // S3 DenseBoost can generate >8k points per rotation.
    static std::vector<sl_lidar_response_measurement_node_hq_t> raw_nodes_buffer;
    if (raw_nodes_buffer.size() < 16384) {
        raw_nodes_buffer.resize(16384);
    }

    size_t count = raw_nodes_buffer.size();

    // Blocking call to fetch scan data
    auto res = drv_->grabScanDataHq(raw_nodes_buffer.data(), count);

    if (SL_IS_OK(res)) {
        drv_->ascendScanData(raw_nodes_buffer.data(), count);
        nodes.assign(raw_nodes_buffer.begin(), raw_nodes_buffer.begin() + count);
        return true;
    } 
    
    // Log error code for debugging (Timeout or Invalid Data)
    std::cerr << "[Driver Error] grabScanDataHq failed! Code: 0x" 
              << std::hex << res << std::dec << std::endl;
              
    return false;
}


// ============================================================================
// Dummy Lidar Driver Implementation
// ============================================================================

bool DummyLidarDriver::connect(const std::string& port, sl_u32 baudrate) {
    (void)port; (void)baudrate; // Suppress unused parameter warnings
    return true; 
}

void DummyLidarDriver::disconnect() { /* No-op */ }

bool DummyLidarDriver::isConnected() { return true; }

int DummyLidarDriver::getHealth() { return 0; } // Always Healthy

void DummyLidarDriver::reset() { /* No-op */ }

bool DummyLidarDriver::start_motor() { return true; }

void DummyLidarDriver::stop_motor() { /* No-op */ }

bool DummyLidarDriver::grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) {
    // Generate synthetic data (Sine wave wall pattern)
    nodes.clear();
    int count = 360; // 1 point per degree resolution
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
    
    // Simulate hardware scanning delay (approx. 10Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}