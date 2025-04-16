import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Nome do pacote com os arquivos do robô
    package_name = 'caramelo_description'
    ros_distro = os.environ['ROS_DISTRO']
    is_ignition = "true" if ros_distro == "humble" else "false"

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory(package_name), 'URDF', 'robot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    # Gera a descrição do robô a partir do Xacro
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration("model"), " is_ignition:=", is_ignition]),
        value_type=str
    )

    # Publicador do estado do robô com tempo simulado ativado
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    # Define onde estão os mundos do Gazebo
    gazebo_resouce_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory(package_name), 'worlds')
    )

    # Lança o Gazebo Ignition com mundo vazio
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf"
        }.items()
    )

    # Spawner do robô na simulação
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=["-topic", "robot_description", "-name", "caramelo"],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resouce_path,
        gazebo,
        spawn_entity,
    ])
