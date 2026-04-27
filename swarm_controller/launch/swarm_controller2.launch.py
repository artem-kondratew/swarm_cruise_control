import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'swarm_controller'
    
    parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params2.yaml')]

    swarm_controller = Node(
        package=package_name,
        executable='swarm_controller',
        parameters=[parameters],
        output='screen',
    )
    
    kalman_filter = Node(
        package=package_name,
        executable='peer_localization',
        parameters=[parameters],
        output='screen',
    )

    return LaunchDescription([
        swarm_controller,
        kalman_filter,
    ])
