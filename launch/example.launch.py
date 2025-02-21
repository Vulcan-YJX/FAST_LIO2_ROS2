import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fast_lio2'),
        'config',
        'avia.yaml'
    )

    return LaunchDescription([
        Node(
            package='fast_lio2',
            executable='fastlio2_mapping',
            name='fastlio2_mapping',
            output='screen',
            parameters=[config]
        )
    ])
