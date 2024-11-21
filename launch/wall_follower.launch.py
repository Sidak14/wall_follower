from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower'
        ),
        Node(
            package='wall_follower',
            executable='see_marker.py',
            name='see_marker'
        ),
        Node(
            package='wall_follower',
            executable='point_transformer.py',
            name='point_transformer',
        )
    ])