import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        # 1. Rosbridge WebSocket (Port 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'address':'0.0.0.0'}],
            output='screen'
        ),
        # 2. Backend Custom (Logique + Serveur Web Port 8000)
        Node(
            package='web_control',
            executable='backend_node',
            name='backend_node',
            output='screen'
        )
    ])
