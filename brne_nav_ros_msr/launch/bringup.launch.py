from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='brne_nav_ros',
        executable='brne_nav_ros',
        output='screen',
    ))

    return ld
