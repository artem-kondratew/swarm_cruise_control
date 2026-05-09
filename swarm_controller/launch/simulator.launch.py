import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'swarm_controller'
    pkg_dir = get_package_share_directory(package_name)
    launch_dir = os.path.join(pkg_dir, 'launch')

    # ── launch arguments ────────────────────────────────────────────────────
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='sliding',
        description=(
            "Follower controller type, propagated to swarm_controller{2,3}.launch.py:\n"
            "  'sliding'    → Python sliding-mode (default, baseline)\n"
            "  'mpc_newton' → Newton + force-lag MPC (physical, 6-state)\n"
            "  'mpc_kin'    → Kinematic adas-style MPC (5-state, 1 param τ)"
        ),
    )
    controller_type = LaunchConfiguration('controller_type')

    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value='',
        description=(
            'Optional yaml with custom waypoints (e.g. saved from '
            'trajectory_drawer); forwarded to pacemaker_controller.launch.py'
        ),
    )
    trajectory_file = LaunchConfiguration('trajectory_file')

    # ── nodes ───────────────────────────────────────────────────────────────
    simulator = Node(
        package=package_name,
        executable='simulator',
        parameters=[os.path.join(pkg_dir, 'config', 'params_simulator.yaml')],
        output='screen',
    )

    pacemaker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'pacemaker_controller.launch.py')
        ),
        launch_arguments={'trajectory_file': trajectory_file}.items(),
    )

    swarm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'swarm_controller2.launch.py')
        ),
        launch_arguments={'controller_type': controller_type}.items(),
    )

    swarm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'swarm_controller3.launch.py')
        ),
        launch_arguments={'controller_type': controller_type}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'config.rviz')],
        output='screen',
    )

    logging = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2parquet'),
                'launch',
                'debug_multi_ros2parquet.launch.py',
            )
        )
    )

    return LaunchDescription([
        controller_type_arg,
        trajectory_file_arg,
        simulator,
        pacemaker,
        swarm2,
        swarm3,
        logging,
        rviz,
    ])
