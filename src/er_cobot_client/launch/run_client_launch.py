from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='er_cobot_client',
            executable='node_platform_client',
            name='node_platform_client'
        ),
        Node(
            package='er_cobot_client',
            executable='node_listener',
            name='node_listener'
        )
    ])