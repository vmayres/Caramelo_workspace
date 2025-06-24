from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # RViz com configuração padrão (pode customizar depois)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '' ]  # pode colocar um .rviz customizado depois
        ),
        # Static transform publisher para base_footprint -> base_link (caso necessário)
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        )
    ])
