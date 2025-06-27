#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch para visualização RViz do robô Caramelo.
    
    Este launch inicia:
    1. RViz com configuração específica para o robô
    2. TF estático do robô (base_link → laser_frame)
    3. Robot State Publisher para o modelo URDF
    4. Visualização de: scan, odometria, mapa, TF tree
    """
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Caminho para os arquivos de configuração
    pkg_caramelo_bringup = get_package_share_directory('caramelo_bringup')
    pkg_caramelo_description = get_package_share_directory('caramelo_description')
    rviz_config_file = os.path.join(pkg_caramelo_bringup, 'config', 'caramelo_complete.rviz')
    
    # Caminho para o URDF do robô
    xacro_file = os.path.join(pkg_caramelo_description, 'URDF', 'robot.urdf.xacro')
    
    # TF estático: base_link → laser_frame (sintaxe nova)
    # Ajuste as coordenadas conforme a posição real do LIDAR no robô
    # Assumindo LIDAR no centro, 20cm acima da base
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.2', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        output='screen'
    )
    
    # TF estático: odom → base_link (será sobrescrito pela odometria real)
    # Este é um fallback caso não tenha odometria ativa
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        output='screen'
    )
    
    # TF estático: map → odom (será usado pelo SLAM)
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )
    
    # Robot State Publisher (usa o URDF real do robô)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz para visualização completa
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # TF Publishers
        static_tf_laser,
        static_tf_odom,
        static_tf_map,
        
        # Robot model
        robot_state_publisher,
        
        # Visualization
        rviz_node,
    ])
