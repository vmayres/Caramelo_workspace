import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

"""
controller.launch.py
- Exclusivo para ativar o controlador mecanum (ros2_control) e o joint_state_broadcaster.
- Sem flags condicionais; simples de usar junto com os seus teleops/conversor.

Uso:
  ros2 launch robot_controller controller.launch.py
"""


def generate_launch_description():
    # configurable sim time
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value="True", description="Use simulated time"
    )
    # 1) Joint State Broadcaster (necessário para publicar /joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output='screen'
    )

    # 2) Controlador mecanum (ros2_control)
    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", os.path.join(get_package_share_directory('robot_controller'), 'config', 'robot_controllers.yaml')
        ],
        output='screen'
    )

    # 3) Broadcast TF odom -> base_footprint from mecanum odometry
    odom_tf_broadcaster = Node(
        package='robot_controller',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        parameters=[
            {"odom_topic": "/mecanum_controller/odometry"},
            {"parent_frame": "odom"},
            {"child_frame": "base_footprint"},
            {"use_parent_from_msg": False},
            {"use_child_from_msg": False},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
        odom_tf_broadcaster,
    ])