import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share =get_package_share_directory('navigation_pkg')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'curt_mini.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    models_path = os.path.join(pkg_share, 'models')

    robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    gz_server = GzServer(
    world_sdf_file=world_path,
    container_name='ros_gz_container',
    create_own_container='True',
    use_composition='True',
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'navigation_pkg',
            'x': '3.849',
            'y': '13.486',
            'z': '0.65',
            'yaw': '0.0',
        }.items(),
    )


    return LaunchDescription([
        # Expose the package's models/ directory to gz-sim so that
        # model://map_plane/... URIs are resolved correctly.
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        robot_state_publisher_node,
        rviz_node,
    ])

