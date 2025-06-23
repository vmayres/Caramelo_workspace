#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    caramelo_description = get_package_share_directory('caramelo_description')
    caramelo_controller = get_package_share_directory('caramelo_controller')
    
    # Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(caramelo_description, 'launch', 'gazebo.launch.py')
        ])
    )
    
    # Load controllers with delay to ensure Gazebo is ready
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen"
            )
        ]
    )

    mecanum_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mecanum_controller",
                    "--controller-manager",
                    "/controller_manager"
                ],
                output="screen"
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
    ])
