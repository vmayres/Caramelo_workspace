#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
											description="Use simulated time"
	)

	joy_node = Node(
		package="joy",
		executable="joy_node",
		name="joystick",
		parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "joy_config.yaml"),
					{"use_sim_time": LaunchConfiguration("use_sim_time")}]
	)
	
	joy_teleop = Node(
		package="joy_teleop",
		executable="joy_teleop",
		parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "joy_teleop.yaml"),
					{"use_sim_time": LaunchConfiguration("use_sim_time")}],
	)

	twist_converter_node = Node(
		package='robot_controller',
		executable='twist_converter_node',
		name='twist_converter',
		output='screen',
	)

	return LaunchDescription([
		use_sim_time_arg,
		joy_node,
		joy_teleop,
		twist_converter_node,
	])

