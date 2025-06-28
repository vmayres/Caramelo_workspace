#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Caminhos dos pacotes
    caramelo_bringup_path = FindPackageShare('caramelo_bringup')
    
    # ===============================================
    # 1. URDF/Robot Description
    # ===============================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            caramelo_bringup_path,
            "urdf",
            "caramelo_real.urdf.xacro"
        ])
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # ===============================================
    # 2. Robot State Publisher
    # ===============================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        respawn=True
    )
    
    # ===============================================
    # 3. Base Footprint TF Publisher - REMOVIDO
    # ===============================================
    # A transformação base_footprint -> base_link já está definida no URDF
    # através do joint "base_joint", não precisa de static transform publisher
    
    # Opção 2: Usando nó customizado (comentado - use se precisar de funcionalidades avançadas)
    # odom_tf_publisher = Node(
    #     package='caramelo_bringup',
    #     executable='odom_tf_publisher_node',
    #     name='odom_tf_publisher',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution([caramelo_bringup_path, "config", "odom_tf_config.yaml"]),
    #         {'use_sim_time': use_sim_time}
    #     ],
    #     respawn=True
    # )
    
    # ===============================================
    # 4. Encoder Node (Odometria real)
    # ===============================================
    encoder_node = Node(
        package='caramelo_bringup',
        executable='encoder_joint_state_node',
        name='encoder_joint_state_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Ordem de inicialização:
        robot_state_publisher,           # 1. URDF primeiro (inclui base_footprint -> base_link)
        encoder_node,                   # 2. Inicia node dos encoders (odom -> base_footprint dinâmico)
    ])
