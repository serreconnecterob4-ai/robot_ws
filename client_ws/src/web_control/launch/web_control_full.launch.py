import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Localisation du fichier de configuration YAML
    # On cherche le fichier dans le dossier 'config' du package 'web_control'
    pkg_share = get_package_share_directory('web_control')
    config_path = os.path.join(pkg_share, 'config', 'configuration.yml')
    
    # Note : Si ton fichier est ailleurs (ex: sur le Bureau), remplace par :
    # config_path = os.path.expanduser('~/Bureau/configuration.yml')

    return LaunchDescription([
        # 1. Rosbridge WebSocket (Port 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[config_path] # Passage des paramètres (IP, etc.)
        ),
        
        # 2. Camera Publisher (Lit l'IP dans le YAML pour se connecter au RTSP)
        Node(
            package='web_control',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[config_path] # Indispensable pour l'URL dynamique
        ),
        
        # 3. Backend Custom (Logique + Serveur Web Port 8000)
        Node(
            package='web_control',
            executable='backend_node',
            name='backend_node',
            output='screen',
            parameters=[config_path] # Indispensable pour afficher l'IP dans les logs
        )
    ])