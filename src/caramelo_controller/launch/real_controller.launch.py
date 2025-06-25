import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Carrega o URDF para HARDWARE REAL (do caramelo_bringup)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("caramelo_bringup"),
            "urdf",
            "caramelo_real.urdf.xacro"
        ])
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Arquivo de configuração dos controladores
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("caramelo_bringup"),
        "config",
        "robot_controllers.yaml"
    ])
    
    # Controller Manager (OBRIGATÓRIO!)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )
    
    # Robot State Publisher  
    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

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

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout", 
            "30"
        ],
        output="screen",
        remappings=[
            ('/mecanum_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/mecanum_drive_controller/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        mecanum_drive_controller_spawner,
    ])
