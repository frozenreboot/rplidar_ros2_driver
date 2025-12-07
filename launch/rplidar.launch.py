"""
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
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    """
    Launch description for the RPLidar Lifecycle Node.
    This launch file handles:
    1. Loading parameters from YAML.
    2. Starting the Lifecycle Node in 'Unconfigured' state.
    3. Automatically transitioning to 'Configured' upon process start.
    4. Automatically transitioning to 'Active' upon successful configuration.
    """
    
    # 1. Parameter file setup
    share_dir = get_package_share_directory("rplidar_ros2_driver")
    parameter_file = LaunchConfiguration("params_file")
    node_name = "rplidar_node"

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "param", "rplidar.yaml"),
        description="Path to the ROS2 parameters file to use.",
    )

    # 2. Lifecycle Node Definition
    driver_node = LifecycleNode(
        package="rplidar_ros2_driver",
        executable="rplidar_node",
        name=node_name,
        namespace="",
        output="screen",
        parameters=[parameter_file],
    )

    # 3. Auto-Configure: Trigger 'configure' transition when node process starts
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_node,
            on_start=[
                LogInfo(msg="[Lifecycle] Process started! Triggering CONFIGURE..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    # 4. Auto-Activate: Trigger 'activate' transition when node reaches 'Inactive' state
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

    # 5. Return Launch Description
    return LaunchDescription(
        [
            params_declare,
            driver_node,
            configure_event,
            activate_event,
        ]
    )