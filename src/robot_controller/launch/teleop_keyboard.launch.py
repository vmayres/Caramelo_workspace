#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch para controle via teclado do robô Caramelo.
    
    Este launch inicia:
    1. Conversor Twist → TwistStamped 
    2. Teleop via teclado
    
    O conversor recebe /cmd_vel (Twist) do teleop e converte para
    /mecanum_drive_controller/cmd_vel (TwistStamped) esperado pelo controlador.
    """
    
    # Nó conversor Twist → TwistStamped
    twist_converter_node = Node(
        package='robot_controller',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    # Nó teleop_twist_keyboard
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Abrir em terminal separado para interação
        parameters=[],
        remappings=[
            # Publica em /cmd_vel (que será convertido pelo conversor)
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        twist_converter_node,
        teleop_keyboard_node,
    ])
