from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('gps_package')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    return LaunchDescription([
        # TF statique: base_link -> frame GPS Gazebo (navigation_pkg/gps_link/navsat_sensor)
        # Position du GPS sur le robot: x=0, y=0, z=base_height/2+0.01 = 0.10
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            output='screen',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'curt_mini/base_link/imu_sensor']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_tf',
            output='screen',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.10',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'gps_link']
        ),
        # EKF Local: roues + IMU ONLY -> odom->base_link (rapide, dérive)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_config],
            remappings=[('odometry/filtered', '/odometry/local')]
        ),
        # Navsat Transform: GPS -> /odometry/gps (délai 3s pour laisser les EKF démarrer)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform',
                    output='screen',
                    parameters=[navsat_config]
                ),
            ]
        ),
        # EKF Global: /odometry/local + /odometry/gps -> map->odom (précis, stable)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_config]
        )
    ])

