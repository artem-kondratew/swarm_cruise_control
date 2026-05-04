import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'swarm_controller'
robot_id = 2


def generate_launch_description():
    pkg_dir = get_package_share_directory(package_name)

    sliding_mode_arg = DeclareLaunchArgument(
        'sliding_mode',
        default_value='true',
        description='If true: Python sliding-mode controller. If false: ACC MPC.',
    )
    sliding_mode = LaunchConfiguration('sliding_mode')

    params_topics = os.path.join(pkg_dir, 'config', f'params{robot_id}.yaml')
    params_acc = os.path.join(pkg_dir, 'config', 'params_swarm_acc.yaml')

    # peer_localization always runs — it publishes Telemetry that both
    # controllers consume.
    peer_localization = Node(
        package=package_name,
        executable='peer_localization',
        parameters=[params_topics],
        output='screen',
    )

    # sliding-mode follower (default)
    sliding_node = Node(
        package=package_name,
        executable='swarm_controller',
        parameters=[params_topics],
        condition=IfCondition(sliding_mode),
        output='screen',
    )

    # MPC follower (when sliding_mode:=false)
    mpc_node = Node(
        package=package_name,
        executable='swarm_acc_mpc_node',
        parameters=[
            params_topics,
            params_acc,
            {
                'telemetry_topic': f'/swarm_controller/telemetry{robot_id}',
                'cmd_vel_topic':   f'/robot{robot_id}/cmd_vel',
                'robot_id':        robot_id,
            },
        ],
        condition=UnlessCondition(sliding_mode),
        output='screen',
    )

    return LaunchDescription([
        sliding_mode_arg,
        peer_localization,
        sliding_node,
        mpc_node,
    ])
