#!/usr/bin/env python3
"""Launch Gazebo (gz sim) com o robô Yahboom.

Fica no pacote 'gazebo_simulation' e usa o URDF/Xacro em 'yahboom_description'.

Argumentos:
world_name: nome do mundo (arquivo .world em gazebo_simulation/worlds)
robot_model: caminho absoluto OU relativo (dentro de yahboom_description) para o xacro
use_rviz: abrir RViz
rviz_config: caminho para config do RViz (opcional)
"""

import os
from os import pathsep
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def declare_args():
    return [
        DeclareLaunchArgument(
            'world_name',
            default_value='empty',
            description='Nome do mundo (arquivo .world em gazebo_simulation/worlds)'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='urdf/robots/robot_3d.urdf.xacro',
            description='Caminho para Xacro (relativo ao pacote yahboom_description ou absoluto)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Abrir RViz (default true)'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare('yahboom_description'), 'rviz', 'mec_mobile_description.rviz']),
            description='Caminho para arquivo .rviz'
        ),
    ]


def resolve_robot_model(context):
    model_input = LaunchConfiguration('robot_model').perform(context)
    if os.path.isabs(model_input):
        return model_input
    yahboom_share = FindPackageShare('yahboom_description').find('yahboom_description')
    candidate = os.path.join(yahboom_share, model_input)
    if not os.path.exists(candidate):
        raise RuntimeError(f'Modelo não encontrado: {candidate}')
    return candidate


def launch_setup(context, *args, **kwargs):
    yahboom_share = FindPackageShare('yahboom_description').find('yahboom_description')
    gz_share = FindPackageShare('gazebo_simulation').find('gazebo_simulation')

    world_name = LaunchConfiguration('world_name').perform(context)
    world_path = os.path.join(gz_share, 'worlds', f'{world_name}.world')
    if not os.path.exists(world_path):
        raise RuntimeError(f'Mundo {world_name} não encontrado: {world_path}')

    robot_model_path = resolve_robot_model(context)

    resource_paths = [
        os.path.join(gz_share, 'models'),
        os.path.join(yahboom_share, 'models'),
        os.path.join(yahboom_share, 'meshes'),
    ]
    resource_paths = [p for p in resource_paths if os.path.isdir(p)]
    gz_resource_env = pathsep.join(resource_paths)

    set_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_env)

    ros_distro = os.environ.get('ROS_DISTRO', '')
    is_ignition = 'True' if ros_distro == 'humble' else 'False'

    robot_description = ParameterValue(
        Command([
            'xacro ', robot_model_path, ' ',
            'is_ignition:=', is_ignition, ' ',
            'use_gazebo:=true'
        ]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'yahboom_robot'],
        output='screen'
    )

    bridges = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridges,
        output='screen'
    )

    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    rviz_args = ['-d', rviz_config] if rviz_config else []
    rviz = None
    if use_rviz == 'true':
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

    actions = [set_resource, gz_sim, rsp, spawn, bridge_node]
    if rviz:
        actions.append(rviz)
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    for a in declare_args():
        ld.add_action(a)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
