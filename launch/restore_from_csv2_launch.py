
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csv_recorder',
            executable='restore_from_csv2',
            name='restore_from_csv2',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
