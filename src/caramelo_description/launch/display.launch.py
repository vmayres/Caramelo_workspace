import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    #! make sure to change this to the name of your package
    package_name='caramelo_description'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory(package_name), 'URDF', 'robot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    join_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'display.rviz')]
    )

    # Launch them all!
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        join_state_publisher_gui,
        rviz_node
    ])