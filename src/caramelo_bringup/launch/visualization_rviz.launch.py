#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    rviz_config_file = os.path.join(pkg_caramelo_bringup, 'config', 'caramelo_complete.rviz')
    
    # TF estático: base_link → laser_frame
    # Ajuste as coordenadas conforme a posição real do LIDAR no robô
    # Assumindo LIDAR no centro, 20cm acima da base
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # TF estático: odom → base_link (será sobrescrito pela odometria real)
    # Este é um fallback caso não tenha odometria ativa
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    # TF estático: map → odom (será usado pelo SLAM)
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Robot State Publisher (publica TF do modelo do robô)
    # Vai usar um URDF simples se não existir um específico
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': """
            <?xml version="1.0"?>
            <robot name="caramelo">
              <link name="base_link">
                <visual>
                  <geometry>
                    <box size="0.4 0.3 0.1"/>
                  </geometry>
                  <material name="blue">
                    <color rgba="0 0 1 1"/>
                  </material>
                </visual>
              </link>
              <link name="laser_frame">
                <visual>
                  <geometry>
                    <cylinder radius="0.05" length="0.1"/>
                  </geometry>
                  <material name="red">
                    <color rgba="1 0 0 1"/>
                  </material>
                </visual>
              </link>
              <joint name="laser_joint" type="fixed">
                <parent link="base_link"/>
                <child link="laser_frame"/>
                <origin xyz="0 0 0.2" rpy="0 0 0"/>
              </joint>
            </robot>
            """,
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
