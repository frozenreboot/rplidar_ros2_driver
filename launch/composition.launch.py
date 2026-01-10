#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RPLIDAR ROS2 DRIVER

Copyright (c) 2025 - 2026 frozenreboot
Copyright (c) 2009 - 2014 RoboPeak Team
Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.

This project is a refactored version of the original rplidar_ros package.
The architecture has been redesigned to support ROS 2 Lifecycle and
multithreading.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS)
OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.

@file   composition.launch.py
@brief  Launch file for RPLIDAR ROS 2 Component Container.

This launch description:
  - Initializes a ComposableNodeContainer (Component Container)
  - Loads the `RPlidarNode` component into the container via Intra-Process Communication
  - Enables Zero-Copy data transfer for high-performance applications

Note:
  Unlike the standalone launch, this file does NOT automatically configure/activate
  the node by default. It is intended for advanced users or system integrators
  who prefer manual lifecycle management or composition-based deployments.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share_dir = get_package_share_directory("rplidar_ros2_driver")
    params_file = os.path.join(share_dir, "param", "rplidar.yaml")

    # 1. component definition
    rplidar_component = ComposableNode(
        package="rplidar_ros2_driver",
        plugin="RPlidarNode",  # name of macro-registered c++
        name="rplidar_node",
        parameters=[params_file],
        extra_arguments=[{"use_intra_process_comms": True}],  # Zero-Copy enabled
    )

    # 2. container definition
    container = ComposableNodeContainer(
        name="rplidar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[rplidar_component],
        output="screen",
    )

    return LaunchDescription([container])
