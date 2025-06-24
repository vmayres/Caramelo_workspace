from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    caramelo_controller_dir = os.path.join(
        get_package_share_directory('caramelo_controller'), 'launch')
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_controller_dir, 'controler.launch.py')
        )
    )
    hw_interface_node = Node(
        package='caramelo_bringup',
        executable='caramelo_hw_interface_node',
        name='caramelo_hw_interface_node',
        output='screen',
    )
    return LaunchDescription([
        control_launch,
        hw_interface_node
    ])
