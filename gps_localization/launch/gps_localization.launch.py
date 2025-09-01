from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_localization',
            executable='gps_localization',
            name='gps_localization',
            output='screen'
        )
    ])






