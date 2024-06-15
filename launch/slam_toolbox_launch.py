from launch_ros.actions import Node
    
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csv_recorder',
            executable='restore_from_csv',
            name='csv_restorer'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': 'restored_scan',
                'mode': 'mapping',
                'queue_size': 1000
            }]
        )
    ])
