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
 * @file lidar_driver_wrapper.hpp
 * @brief Abstract interface and implementation for Slamtec RPLidar driver.
 * @details This file defines a common interface for Lidar drivers to decouple 
 * the hardware SDK from the ROS2 node logic. It includes a real driver 
 * implementation using the Slamtec SDK and a dummy driver for simulation.
 * @author frozenreboot (frozenreboot@gmail.com)
 * @date 2025-12-13
 */

#ifndef LIDAR_DRIVER_WRAPPER_HPP_
#define LIDAR_DRIVER_WRAPPER_HPP_

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <vector>
#include <string>
#include <memory>

/**
 * @class LidarDriverInterface
 * @brief Abstract base class defining the standard interface for Lidar interactions.
 * * This interface allows the ROS2 node to interact with the hardware agnostically.
 * Implementations can be swapped (e.g., Real vs. Dummy) without changing the node logic.
 */
class LidarDriverInterface {
public:
    virtual ~LidarDriverInterface() = default;

    /**
     * @brief Connects to the Lidar hardware.
     * @param port The serial port path (e.g., "/dev/ttyUSB0").
     * @param baudrate Communication baud rate (e.g., 115200, 256000).
     * @return true if connection is successful, false otherwise.
     */
    virtual bool connect(const std::string& port, sl_u32 baudrate) = 0;

    /**
     * @brief Disconnects from the Lidar hardware and releases resources.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Checks if the driver is currently connected to hardware.
     * @return true if connected.
     */
    virtual bool isConnected() = 0;

    /**
     * @brief Checks the health status of the device.
     * @return 0: OK, 1: Warning, 2: Error
     */
    virtual int getHealth() = 0;

    /**
     * @brief Performs a hardware reset (used for recovery).
     */
    virtual void reset() = 0;

    /**
     * @brief Starts the Lidar motor and begins the scanning process.
     * @return true if the motor started successfully, false otherwise.
     */
    virtual bool start_motor() = 0;

    /**
     * @brief Stops the Lidar motor and halts scanning.
     */
    virtual void stop_motor() = 0;

    /**
     * @brief Retrieves a complete scan data set from the driver.
     * @param nodes Reference to a vector where the scan points will be stored.
     * @return true if a full scan was retrieved successfully, false otherwise.
     */
    virtual bool grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) = 0;
};

/**
 * @class RealLidarDriver
 * @brief Concrete implementation of LidarDriverInterface using the Slamtec SDK.
 * * This class wraps the `sl::ILidarDriver` to communicate with actual hardware via serial/UART.
 */
class RealLidarDriver : public LidarDriverInterface {
public:
    RealLidarDriver();
    ~RealLidarDriver() override;

    bool connect(const std::string& port, sl_u32 baudrate) override;
    void disconnect() override;
    bool isConnected() override;
    int getHealth() override;
    void reset() override;
    bool start_motor() override;
    void stop_motor() override;
    bool grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) override;

private:
    /// @brief Raw pointer to the Slamtec SDK driver instance.
    sl::ILidarDriver *drv_ = nullptr;
    sl_lidar_response_device_info_t devinfo_; 
    bool is_s_series() { return (devinfo_.model >> 4) >= 5; }
};

/**
 * @class DummyLidarDriver
 * @brief Mock implementation of LidarDriverInterface for testing and simulation.
 * * Generates synthetic scan data (e.g., a sine wave pattern) to allow node logic verification
 * without physical hardware.
 */
class DummyLidarDriver : public LidarDriverInterface {
public:
    DummyLidarDriver() = default;
    ~DummyLidarDriver() override = default;

    bool connect(const std::string& port, sl_u32 baudrate) override;
    void disconnect() override;
    bool isConnected() override;
    int getHealth() override;
    void reset() override;
    bool start_motor() override;
    void stop_motor() override;
    bool grab_scan_data(std::vector<sl_lidar_response_measurement_node_hq_t>& nodes) override;
};

#endif // LIDAR_DRIVER_WRAPPER_HPP_