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
    
    auto ans = drv_->connect(channel);
    if (SL_IS_FAIL(ans)) return false;
    // Attempt connection
    auto res = drv_->getDeviceInfo(devinfo_);
    if (SL_IS_FAIL(res)) {
        std::cerr << "Failed to get device info!" << std::endl;
        return false;
    }
    
    std::cout << "Connected to RPLIDAR Model ID: " << (int)(devinfo_.model >> 4) << std::endl;
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

// lidar_driver_wrapper.cpp 의 start_motor 함수를 이걸로 교체하세요

bool RealLidarDriver::start_motor() {
    if (!isConnected()) return false;

    // 1. 모터 회전 시작 (S시리즈는 내부 제어, A시리즈는 PWM)
    if (is_s_series()) {
        drv_->setMotorSpeed(); 
    } else {
        drv_->setMotorSpeed(600); 
    }

    // 2. 지원하는 스캔 모드 탐색
    std::vector<LidarScanMode> allSupportedScanModes;
    sl_result op_result = drv_->getAllSupportedScanModes(allSupportedScanModes);

    if (SL_IS_FAIL(op_result)) {
        // 모드 조회가 안 되면 구형(Legacy) 방식으로 시도
        auto res = drv_->startScan(0, 1); 
        return SL_IS_OK(res);
    }

    // 3. 가장 적절한 모드 선택 (S3는 보통 DenseBoost나 Standard를 씀)
    // 여기서는 단순히 목록의 마지막(보통 성능이 제일 좋은) 모드를 선택하거나,
    // 특정 이름("DenseBoost")을 찾도록 할 수 있습니다.
    
    sl_u16 selectedScanMode = sl_u16(-1);
    
    // 로그로 지원 모드 출력 (디버깅용)
    std::cout << "--- Supported Scan Modes ---" << std::endl;
    for (const auto& mode : allSupportedScanModes) {
        std::cout << "Mode: " << mode.scan_mode << ", ID: " << mode.id << std::endl;
        // 만약 'Standard'나 'DenseBoost'를 선호한다면 여기서 string 비교로 id를 찾으면 됨.
    }
    std::cout << "----------------------------" << std::endl;

// [수정 후] DenseBoost 모드 선호 (S3 권장 모드)
    for (const auto& mode : allSupportedScanModes) {
        std::string mode_name = mode.scan_mode;
        // "DenseBoost"가 포함된 모드를 찾습니다.
        if (mode_name.find("DenseBoost") != std::string::npos) { 
             selectedScanMode = mode.id;
             break;
        }
    }

    // DenseBoost 못 찾았으면 리스트의 마지막 거(보통 최신 모드) 사용
    if (selectedScanMode == sl_u16(-1) && !allSupportedScanModes.empty()) {
        selectedScanMode = allSupportedScanModes.back().id;
    }

    // Standard 못 찾았으면 그냥 리스트의 마지막 거(보통 최신 모드) 사용
    if (selectedScanMode == sl_u16(-1) && !allSupportedScanModes.empty()) {
        selectedScanMode = allSupportedScanModes.back().id;
    }

    // 4. 선택된 모드로 스캔 시작 (Express Scan)
    LidarScanMode scanParams;
    op_result = drv_->startScanExpress(false, selectedScanMode, 0, &scanParams);

    if (SL_IS_OK(op_result)) {
        std::cout << "Selected Mode ID: " << selectedScanMode << " | Max Dist: " << scanParams.max_distance << std::endl;
        return true;
    } else {
        // Express 실패시 일반 스캔 시도 (최후의 수단)
        return SL_IS_OK(drv_->startScan(0, 1));
    }
}

void RealLidarDriver::stop_motor() {
    if (isConnected()) {
        drv_->stop();
        drv_->setMotorSpeed(0);
    }
}

// lidar_driver_wrapper.cpp 수정

bool RealLidarDriver::grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) {
    if (!isConnected()) return false;

    // [수정 1] S3는 데이터가 많을 수 있으니 버퍼를 넉넉하게 잡고, 스택 대신 vector(Heap)를 씁니다.
    // S3 DenseBoost 모드 등에서 한 바퀴에 8192개 넘을 수도 있습니다. 안전하게 16k 잡으세요.
    static std::vector<sl_lidar_response_measurement_node_hq_t> raw_nodes_buffer;
    if (raw_nodes_buffer.size() < 16384) {
        raw_nodes_buffer.resize(16384);
    }

    size_t count = raw_nodes_buffer.size();

    // Blocking call (Timeout usually 2s)
    // 여기서 count는 입력(버퍼크기)이자 출력(받은 개수)입니다.
    auto res = drv_->grabScanDataHq(raw_nodes_buffer.data(), count);

    if (SL_IS_OK(res)) {
        drv_->ascendScanData(raw_nodes_buffer.data(), count);
        nodes.assign(raw_nodes_buffer.begin(), raw_nodes_buffer.begin() + count);
        return true;
    } 
    
    // [수정 2] 도대체 왜 실패했는지 헥사 코드를 봐야 합니다.
    // RESULT_OPERATION_TIMEOUT (0x80000008) 인지, RESULT_INVALID_DATA 인지 확인 필요
    // printf는 버퍼링 때문에 ROS2 로그에 늦게 뜰 수 있으니 stderr로 뱉습니다.
    std::cerr << "[Driver Error] grabScanDataHq failed! Code: " 
              << std::hex << res << std::dec << std::endl;
              
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