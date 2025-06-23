import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30"
        ],
        output="screen"
    )

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout", 
            "30"
        ],
        output="screen",
        remappings=[
            ('/mecanum_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/mecanum_controller/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
    ])
