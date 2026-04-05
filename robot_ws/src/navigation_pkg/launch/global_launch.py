import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_pkg')
    gps_share = get_package_share_directory('gps_package')

    nav2_launch = os.path.join(pkg_share, 'launch', 'nav2_minimal.launch.py')
    ekf_launch = os.path.join(gps_share, 'launch', 'ekf_launch.py')



    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch)
    )

    ekf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch)
    )

    # Launch the waypoint action server module from the installed package
    waypoint_process = ExecuteProcess(
        cmd=['python3', '-m', 'navigation_pkg.waypoint_action_server'],
        output='screen'
    )

    gate_process = ExecuteProcess(
        cmd=['python3', '-m', 'navigation_pkg.cmd_vel_gate'],
        output='screen'
    )

    rosbridge_websocket = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'rosbridge_server',
            'rosbridge_websocket_launch.xml',
            'address:=0.0.0.0',
        ],
        output='screen',
    )

    odom_relay_process = ExecuteProcess(
        cmd=[
            'python3',
            '-m',
            'navigation_pkg.odom_rosbridge_relay',
            '--ros-args',
            '-p',
            'bridge_host:=100.123.147.56',
            '-p',
            'bridge_port:=9090',
        ],
        output='screen'
    )

    # Stagger the start of each included launch / process to avoid race conditions
    ekf_timer = TimerAction(period=6.0, actions=[ekf_include])
    odom_relay_timer = TimerAction(period=8.0, actions=[odom_relay_process])
    rosbridge_websocket_timer = TimerAction(period=10.0, actions=[rosbridge_websocket])
    nav2_timer = TimerAction(period=30.0, actions=[nav2_include])
    waypoint_timer = TimerAction(period=40.0, actions=[waypoint_process])
    gate_timer = TimerAction(period=41.0, actions=[gate_process])

    return LaunchDescription([
        ekf_timer,
        odom_relay_timer,
        rosbridge_websocket_timer,
        nav2_timer,
        waypoint_timer,
        gate_timer
    ])
