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
    sliding_mode_arg = DeclareLaunchArgument(
        'sliding_mode',
        default_value='true',
        description=(
            'If true (default): Python sliding-mode controller for followers. '
            'If false: ACC MPC controller (swarm_acc_mpc_node). '
            'Propagated to swarm_controller2/3.launch.py.'
        ),
    )
    sliding_mode = LaunchConfiguration('sliding_mode')

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
        )
    )

    swarm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'swarm_controller2.launch.py')
        ),
        launch_arguments={'sliding_mode': sliding_mode}.items(),
    )

    swarm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'swarm_controller3.launch.py')
        ),
        launch_arguments={'sliding_mode': sliding_mode}.items(),
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
        sliding_mode_arg,
        simulator,
        pacemaker,
        swarm2,
        swarm3,
        logging,
        rviz,
    ])
