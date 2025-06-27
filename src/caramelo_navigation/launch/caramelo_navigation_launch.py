#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Configurações
    package_name = 'caramelo_navigation'
    nav2_params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declara argumentos de launch
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'maps', 'map.yaml'),
        description='Full path to the map yaml file to use')

    # Include SLAM launch file
    slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name),
                         'launch', 'slam_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # Include navigation launch file
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name),
                         'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav2_params_file,
                          'use_sim_time': use_sim_time}.items())

    # Map server para navegação (quando não usar SLAM)
    start_map_server_cmd = Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_yaml_file}])

    # AMCL para localização (quando não usar SLAM)
    start_amcl_cmd = Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Lifecycle manager para map server e AMCL
    start_lifecycle_manager_localization_cmd = Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}])

    # RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name),
                                      'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Cria a descrição de launch
    ld = LaunchDescription()

    # Declara argumentos de launch
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Adiciona ações
    ld.add_action(slam_launch_cmd)
    ld.add_action(navigation_launch_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_lifecycle_manager_localization_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
