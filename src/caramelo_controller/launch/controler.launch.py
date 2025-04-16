from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Spawner para o joint_state_broadcaster
    joint_state_publisher_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    # Spawner para o mecanum_drive_controller
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_spawner,
        mecanum_controller_spawner,
    ])
