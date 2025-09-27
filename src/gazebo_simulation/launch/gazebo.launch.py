#!/usr/bin/env python3
"""
Launch do Gazebo (ros_gz_sim) e spawn do robô Yahboom, inspirado no projeto do professor.

Objetivo deste pacote: apenas subir o simulador e carregar mundos, sem
interferir no pacote de descrição (mantemos a arquitetura própria do projeto).

Parâmetros principais:
- world: mundo a ser carregado. Aceita caminho absoluto para um .world ou
    apenas o nome do arquivo localizado em share/gazebo_simulation/worlds.
    Default: 'empty.world'.

Exemplos:
- Arquivo em worlds do pacote:
    ros2 launch gazebo_simulation gazebo.launch.py world:=small_warehouse.world
- Caminho absoluto:
    ros2 launch gazebo_simulation gazebo.launch.py \
        world:=$(ros2 pkg prefix gazebo_simulation)/share/gazebo_simulation/worlds/small_warehouse.world
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # Lê parâmetros fornecidos pela linha de comando
    world_arg = LaunchConfiguration('world').perform(context)
    # Nome fixo para a entidade no Gazebo, já que há um único robô
    robot_name = 'robot'

    # 1) Resolver caminho do Xacro do robô Yahboom
    robot_share = get_package_share_directory('yahboom_description')
    # Yahboom: arquivo principal está em urdf/robots/robot_3d.urdf.xacro
    xacro_path = os.path.join(robot_share, 'urdf', 'robots', 'robot_3d.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    # 2) Resolver caminho do mundo: aceita absoluto ou nome relativo à pasta 'worlds' deste pacote
    sim_share = get_package_share_directory('gazebo_simulation')
    worlds_dir = os.path.join(sim_share, 'worlds')
    models_dir = os.path.join(sim_share, 'models')

    # Se for caminho absoluto existente, usa direto. Caso contrário, tenta em worlds_dir.
    if os.path.isabs(world_arg) and os.path.exists(world_arg):
        world_path = world_arg
    else:
        candidate = os.path.join(worlds_dir, world_arg)
        # Aceitar nome sem extensão priorizando .world (ex.: "empty" -> "empty.world")
        if not os.path.splitext(candidate)[1]:
            if os.path.exists(candidate + '.world'):
                candidate = candidate + '.world'
        world_path = candidate

    # 3) Exportar GZ_SIM_RESOURCE_PATH com múltiplos diretórios úteis:
    #    - share do robô (para meshes e urdf)
    #    - pasta worlds deste pacote (para localizar mundos relativos)
    #    - pasta models deste pacote (para recursos salvos pelo Gazebo)
    #    Obs.: múltiplos paths separados por ':'
    gz_paths = f"{robot_share}:{worlds_dir}:{models_dir}"
    gz_resource = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_paths)

    # 4) Publicar descrição do robô no TF (sim_time) – igual ao padrão do professor
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # 5) Iniciar Gazebo passando argumentos claramente (verbose + run + mundo)
    gz_args = ['-v', '4', '-r', world_path]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items()
    )

    # 6) Spawn da entidade a partir do tópico 'robot_description'
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', robot_name],
        output='screen'
    )

    # 7) Bridge: clock + sensores (IMU, LiDAR, RGBD)
    #    Mantido simples; ajuste os nomes conforme sua URDF/Xacro publicar sensores.
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU", 
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",             
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked", 
            "/cam_1/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/cam_1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked", 
        ],
        remappings=[]
    )

    return [gz_resource, rsp, gazebo, gz_spawn_entity, gz_ros2_bridge]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Mundo do Gazebo (arquivo .world salvo em share/gazebo_simulation/worlds, ex: empty.world)'
    )

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])
