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
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    # -------------------------------------------------------------------------
    # 1. Launch Arguments
    # -------------------------------------------------------------------------
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace of the ROS node, and its topics, services etc.",
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="rplidar_node",
        description="Name of the ROS node, can be e.g. front_c1_node",
    )
    output_arg = DeclareLaunchArgument(
        "output",
        default_value="screen",
        description="Output mode of the node.",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/rplidar",
        description="Connected serial port of the lidar (e.g., '/dev/ttyUSB0'"
        " or '/dev/front_lidar')",
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="1000000",
        description="Baudrate Settings (model-dependent)",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="laser_frame",
        description="TF link frame of the published sensor messages",
    )
    angle_compensate_arg = DeclareLaunchArgument(
        "angle_compensate",
        default_value="false",
        description="Hardware / SDK Level: Geometric / angle compensation in the SDK. ",
    )
    scan_mode_arg = DeclareLaunchArgument(
        "scan_mode",
        default_value="",
        description="Scan mode name (empty = use driver default).",
    )
    rpm_arg = DeclareLaunchArgument(
        "rpm",
        default_value="0",
        description="Motor RPM (0 = use driver default).",
    )
    max_distance_arg = DeclareLaunchArgument(
        "max_distance",
        default_value="0.0",
        description=(
            "Declared range_max in meters (0.0 = use hardware limit). "
            "Data is not clipped to this value yet."
        ),
    )
    dummy_mode_arg = DeclareLaunchArgument(
        "dummy_mode",
        default_value="false",
        description="true  : Run in simulation mode (no hardware required), "
        "false : Use real hardware via the Slamtec SDK.",
    )
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="If true, broadcast static TF internally, base_link->"
        "[frame_id value], with x,y,z = (0,0,0)",
    )
    qos_policy_arg = DeclareLaunchArgument(
        "qos_policy",
        default_value="sensor_data",
        description="RMW QoS policy for LaserScan publisher. ('sensor_data' "
        "or 'services_default', all other values defaults to "
        "'system_default')",
    )
    max_retries_arg = DeclareLaunchArgument(
        "max_retries",
        default_value="3",
        description="Number of allowed communication errors before reset.",
    )
    publish_point_cloud_arg = DeclareLaunchArgument(
        "publish_point_cloud",
        default_value="false",
        description="Publish a Pointcloud2 topic",
    )
    computed_ray_count_arg = DeclareLaunchArgument(
        "computed_ray_count",
        default_value="1450",
        description="Ranges size of the LaserScan topic for the computed rays",
    )
    interpolated_rays_arg = DeclareLaunchArgument(
        "interpolated_rays",
        default_value="true",
        description="Uses a fixed angle data from intersections with the line"
        "segments between hardware reported points if enabled",
    )
    use_intensities_arg = DeclareLaunchArgument(
        "use_intensities",
        default_value="false",
        description="Whether to use intensities array in the LaserScan msg",
    )
    intensities_as_angles_arg = DeclareLaunchArgument(
        "intensities_as_angles",
        default_value="true",
        description="Allows users to directly read the angle values reported "
        "by the hardware. Use it together with use_intensities:=true",
    )
    angle_offset_arg = DeclareLaunchArgument(
        "angle_offset",
        default_value="0.0",
        description="Angle offset in radian (e.g., 3.1415926 for A1 fix). Overrides YAML.",
    )
    # -------------------------------------------------------------------------
    # 2. Lifecycle Node Definition
    # -------------------------------------------------------------------------
    driver_node = LifecycleNode(
        package="rplidar_driver",
        executable="rplidar_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output=LaunchConfiguration("output"),
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": LaunchConfiguration("serial_baudrate"),
                "frame_id": LaunchConfiguration("frame_id"),
                "angle_compensate": LaunchConfiguration("angle_compensate"),
                "scan_mode": LaunchConfiguration("scan_mode"),
                "rpm": LaunchConfiguration("rpm"),
                "max_distance": LaunchConfiguration("max_distance"),
                "dummy_mode": LaunchConfiguration("dummy_mode"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "qos_policy": LaunchConfiguration("qos_policy"),
                "max_retries": LaunchConfiguration("max_retries"),
                "publish_point_cloud": LaunchConfiguration("publish_point_cloud"),
                "computed_ray_count": LaunchConfiguration("computed_ray_count"),
                "interpolated_rays": LaunchConfiguration("interpolated_rays"),
                "use_intensities": LaunchConfiguration("use_intensities"),
                "intensities_as_angles": LaunchConfiguration("intensities_as_angles"),
                "angle_offset": LaunchConfiguration("angle_offset"),
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
            # Standard ROS node arguments
            node_name_arg,
            namespace_arg,
            output_arg,
            # rplidar_node arguments
            serial_port_arg,
            serial_baudrate_arg,
            frame_id_arg,
            angle_compensate_arg,
            scan_mode_arg,
            rpm_arg,
            max_distance_arg,
            dummy_mode_arg,
            publish_tf_arg,
            qos_policy_arg,
            max_retries_arg,
            publish_point_cloud_arg,
            computed_ray_count_arg,
            interpolated_rays_arg,
            use_intensities_arg,
            intensities_as_angles_arg,
            angle_offset_arg,
            driver_node,
            configure_event,
            activate_event,
        ]
    )
