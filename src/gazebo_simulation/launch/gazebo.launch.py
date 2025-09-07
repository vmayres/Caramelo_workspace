#!/usr/bin/env python3
"""Launch do Gazebo (ros_gz_sim) e spawn do robô, no estilo do exemplo base.

Requisitos:
- Escolher o robô entre 'yahboom_description' (default) e 'caramelo_description'.
- Escolher o mundo (default: 'empty.sdf').
- Não iniciar controllers; apenas Gazebo + spawn.
"""

import os
from launch import LaunchDescription
from pathlib import Path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    robot_pkg = LaunchConfiguration('robot').perform(context)
    world_name = LaunchConfiguration('world').perform(context)

    # Resolver caminho do Xacro conforme o pacote selecionado
    # Nota: caramelo_description usa 'URDF', yahboom_description usa 'urdf'
    if robot_pkg == 'caramelo_description':
        robot_share = get_package_share_directory('caramelo_description')
        xacro_path = os.path.join(robot_share, 'URDF', 'robot.urdf.xacro')
    else:
        robot_share = get_package_share_directory('yahboom_description')
        xacro_path = os.path.join(robot_share, 'urdf', 'robot.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    # GZ_SIM_RESOURCE_PATH similar ao base: pai do pacote de descrição selecionado
    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(robot_share).parent.resolve())]
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Iniciar Gazebo com o mundo solicitado (se existir no resource path)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[(
            'gz_args', [' -v 4', ' -r', f' {world_name}']
        )]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', robot_pkg],
        output='screen'
    )

    # Bridge como no base (opcional, útil para clock/IMU)
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[('/imu', '/imu/out')]
    )

    return [gz_resource, rsp, gazebo, gz_spawn_entity, gz_ros2_bridge]


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='yahboom_description',
        description='Pacote do robô a spawnar (yahboom_description ou caramelo_description)'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Mundo do Gazebo (nome encontrado no resource path, ex: empty.sdf)'
    )

    return LaunchDescription([
        robot_arg,
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])
