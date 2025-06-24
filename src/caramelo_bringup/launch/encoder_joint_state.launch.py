from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='caramelo_bringup',
            executable='encoder_joint_state_node',
            name='encoder_joint_state_node',
            output='screen',
        )
    ])
