import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    robot_controller_pkg = get_package_share_directory('robot_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True", description="Use simulated time")

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(
                get_package_share_directory("robot_controller"),
                "config",
                "joy_teleop.yaml",
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(
                get_package_share_directory("robot_controller"),
                "config",
                "joy_config.yaml",
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    
    teleop_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[],
        remappings=[
            # Remapeia /cmd_vel para /key_vel para funcionar com twist_mux
            ("/cmd_vel", "/key_vel"),
        ],
    )
    
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py",
            )
        ),
        launch_arguments={
            "cmd_vel_out": "robot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(robot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(robot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(robot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="robot_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_twist_topic": "robot_controller/cmd_vel_unstamped"},
            {"output_twist_stamped_topic": "/mecanum_controller/reference"},
            {"frame_id": "base_footprint"},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay_node,
            teleop_keyboard_node
        ]
    )