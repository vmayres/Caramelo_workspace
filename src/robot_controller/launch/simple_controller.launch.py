#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

"""
simple_controller.launch.py
- Inicia seus controladores próprios: simple_controller e noisy_controller (opcional).
- Inclui o spawner do simple_velocity_controller e o conversor Twist->TwistStamped.
- Mantém parâmetros de cinemática mecanum configuráveis por argumentos.

Uso (apenas simple):
  ros2 launch robot_controller simple_controller.launch.py use_noisy_controller:=False

Uso (simple + noisy):
  ros2 launch robot_controller simple_controller.launch.py use_noisy_controller:=True
"""


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    wheel_radius_arg = DeclareLaunchArgument('wheel_radius', default_value='0.0325')
    wheel_base_arg = DeclareLaunchArgument('wheel_base', default_value='0.08')        # metade do comprimento (L)
    wheel_separation_arg = DeclareLaunchArgument('wheel_separation', default_value='0.0845')  # metade da largura efetiva (W)
    use_noisy_controller_arg = DeclareLaunchArgument('use_noisy_controller', default_value='False')

    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    wheel_separation = LaunchConfiguration('wheel_separation')
    use_noisy_controller = LaunchConfiguration('use_noisy_controller')

    # Spawner do Joint State Broadcaster (necessário para publicar /joint_states)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen'
    )

    # Spawner do group velocity controller (para acionar as 4 rodas via Float64MultiArray)
    simple_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'simple_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', os.path.join(
                get_package_share_directory('robot_controller'), 'config', 'robot_controllers.yaml'
            )
        ],
        output='screen'
    )

    # Seu controlador simples (cinemática mecanum)
    simple_controller_node = Node(
        package='robot_controller',
        executable='simple_controller',
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'wheel_separation': wheel_separation,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Conversor Twist->TwistStamped para alimentar o mecanum_controller quando você estiver usando teleops
    twist_converter_node = Node(
        package='robot_controller',
        executable='twist_converter_node',
        name='twist_converter_for_simple',
    output='screen',
    )

    # Controlador ruidoso (opcional)
    noisy_controller_node = Node(
        package='robot_controller',
        executable='noisy_controller',
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'wheel_separation': wheel_separation,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(use_noisy_controller),
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_base_arg,
        wheel_separation_arg,
        use_noisy_controller_arg,
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        simple_controller_node,
        twist_converter_node,
        noisy_controller_node,
    ])
