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
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",  # Ensure this matches the controller name
            "--controller-manager",
            "/controller_manager",
            "--ros-args",
            "--params-file",
            "/home/victor/Caramelo_workspace/src/caramelo_controller/config/robot_controllers.yaml"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
    ])
