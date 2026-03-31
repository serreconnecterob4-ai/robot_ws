import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # chemin vers le dossier du package 'camera'
    package_share = get_package_share_directory('camera')
    # Try several plausible locations for the mediamtx binary
    candidates = []
    # relative to installed package share (install/camera/share -> .../mediamtx)
    candidates.append(os.path.normpath(os.path.join(package_share, '..', 'mediamtx')))
    # relative to this source file: src/camera/launch -> src/camera -> ../mediamtx
    this_file_dir = os.path.dirname(__file__)
    candidates.append(os.path.normpath(os.path.join(this_file_dir, '..', 'mediamtx')))
    # also try one level up (in case of different layouts)
    candidates.append(os.path.normpath(os.path.join(this_file_dir, '..', '..', 'mediamtx')))
    # workspace-level mediamtx (one level up from package share)
    candidates.append(os.path.normpath(os.path.join(package_share, '..', '..', 'mediamtx')))
    # also try workspace src layout from current working directory (when running from workspace)
    try:
        workspace_src_candidate = os.path.normpath(os.path.join(os.getcwd(), 'src', 'camera', 'mediamtx'))
        candidates.append(workspace_src_candidate)
    except Exception:
        pass

    mediamtx_dir = None
    for c in candidates:
        if os.path.isdir(c):
            mediamtx_dir = c
            break

    if mediamtx_dir is None:
        raise FileNotFoundError(
            'mediamtx directory not found. Tried:\n' + '\n'.join(candidates) + '\n\nPlease place the mediamtx folder next to the package or adjust the launch file.'
        )

    mediamtx_exec = os.path.join(mediamtx_dir, 'mediamtx')
    if not os.path.isfile(mediamtx_exec):
        raise FileNotFoundError(f"mediamtx executable not found at {mediamtx_exec}. Make sure it's present and executable.")

    return LaunchDescription([
        # Lance MediaMTX (doit être exécutable dans ../mediamtx)
        # ExecuteProcess(
        #     cmd=[mediamtx_exec],
        #     cwd=mediamtx_dir,
        #     output='screen'
        # ),
        Node(
            package='camera',
            executable='camera_control_node',
            name='camera_control_node',
            output='screen'
        ),
        Node(
            package='camera',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        )
    ])
