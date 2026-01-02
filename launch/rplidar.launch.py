#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RPLIDAR ROS2 DRIVER

Copyright (c) 2025, frozenreboot
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

@file   rplidar.launch.py
@brief  Launch file for the Slamtec RPLIDAR ROS 2 Lifecycle node.

This launch description:
  - Declares configurable launch arguments (params file, RPM, scan mode, etc.)
  - Starts the Lifecycle-based RPLIDAR node
  - Automatically drives the node through CONFIGURE -> ACTIVATE transitions

The node itself is implemented in `rplidar_node` and uses a managed lifecycle:
  - On startup the process is launched
  - Once the process is up, a CONFIGURE transition is emitted
  - When the node reaches the `inactive` state, an ACTIVATE transition is emitted
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    """
    Generate the LaunchDescription for the RPLIDAR Lifecycle node.

    Launch arguments:
      - params_file     : Path to a YAML file containing node parameters
      - rpm             : Motor RPM (0 = let the driver choose a default)
      - scan_mode       : Scan mode name (empty = let the driver choose a default)
      - max_distance    : Maximum range clip in meters (0.0 = use hardware limit)
      - publish_tf      : Whether the node should broadcast a static TF transform
      - qos_reliability : Reliability QoS policy for the LaserScan publisher
                          (e.g., 'best_effort' or 'reliable')
    """
    share_dir = get_package_share_directory("rplidar_ros2_driver")
    node_name = "rplidar_node"

    # -------------------------------------------------------------------------
    # 1. Launch Arguments (user-configurable options)
    # -------------------------------------------------------------------------
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "param", "rplidar.yaml"),
        description="Path to the ROS 2 parameters file to use.",
    )

    rpm_arg = DeclareLaunchArgument(
        "rpm",
        default_value="0",
        description="Motor RPM (0 = use driver default).",
    )

    scan_mode_arg = DeclareLaunchArgument(
        "scan_mode",
        default_value="",
        description="Scan mode name (empty = use driver default).",
    )

    max_dist_arg = DeclareLaunchArgument(
        "max_distance",
        default_value="0.0",
        description="Maximum range clip in meters (0.0 = use hardware limit).",
    )

    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="If true, broadcast static TF internally.",
    )

    qos_arg = DeclareLaunchArgument(
        "qos_reliability",
        default_value="best_effort",
        description="Reliability QoS policy for LaserScan publisher.",
    )
    max_retries_arg = DeclareLaunchArgument(
        "max_retries",
        default_value="3",
        description="Number of allowed communication errors before reset.",
    )
    # -------------------------------------------------------------------------
    # 2. Lifecycle Node Definition
    # -------------------------------------------------------------------------
    driver_node = LifecycleNode(
        package="rplidar_ros2_driver",
        executable="rplidar_node",
        name=node_name,
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "rpm": LaunchConfiguration("rpm"),
                "scan_mode": LaunchConfiguration("scan_mode"),
                "max_distance": LaunchConfiguration("max_distance"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "qos_reliability": LaunchConfiguration("qos_reliability"),
                "max_retries": LaunchConfiguration("max_retries"),
            },
        ],
    )

    # -------------------------------------------------------------------------
    # 3. Auto-Configure on process start
    # -------------------------------------------------------------------------
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_node,
            on_start=[
                LogInfo(msg="[Lifecycle] Process started, triggering CONFIGURE..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    # -------------------------------------------------------------------------
    # 4. Auto-Activate when node reaches 'inactive'
    # -------------------------------------------------------------------------
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver_node,
            goal_state="inactive",
            entities=[
                LogInfo(msg="[Lifecycle] Transitioning to ACTIVE..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # -------------------------------------------------------------------------
    # 5. Assemble LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription(
        [
            params_file_arg,
            rpm_arg,
            scan_mode_arg,
            max_dist_arg,
            publish_tf_arg,
            qos_arg,
            max_retries_arg,
            driver_node,
            configure_event,
            activate_event,
        ]
    )


# Local import to avoid circular dependency at top-level
from launch_ros.events.lifecycle import ChangeState  # noqa: E402
