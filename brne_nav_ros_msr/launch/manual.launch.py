from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='brne_nav_ros',
        executable='brne_nav_ros',
        output='screen',
    ))

    ld.add_action(Node(
        package='brne_nav_ros',
        executable='manual',
        output='screen',
    ))

    pkg_path = get_package_share_path('brne_nav_ros')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(pkg_path / 'im.rviz')],
        output='log',
    ))

    return ld
