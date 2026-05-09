import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('swarm_controller')
    main_yaml = os.path.join(pkg_dir, 'config', 'params_pacemaker.yaml')

    # Optional CLI override of the `trajectory_file` parameter. When passed,
    # we set the parameter directly — the node itself does path resolution
    # (relative paths → swarm_controller/config/, see
    # `pacemaker_controller._resolve_trajectory_path`). This keeps the
    # resolution rule in one place, identical whether the value comes from
    # params_pacemaker.yaml or from the CLI.
    traj_file = LaunchConfiguration('trajectory_file').perform(context).strip()
    params = [main_yaml]
    if traj_file:
        params.append({'trajectory_file': traj_file})

    return [Node(
        package='swarm_controller',
        executable='pacemaker_controller',
        parameters=params,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'trajectory_file',
            default_value='',
            description=(
                'Optional path to a yaml that overrides waypoints_x/y and '
                'sets trajectory: lanelet (e.g. saved from trajectory_drawer)'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
