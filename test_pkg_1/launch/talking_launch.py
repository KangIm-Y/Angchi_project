import os

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        Node(
            package='test_pkg_1',
            executable='test_talker1',
            name='name_test1',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
            ),

        Node(
            package='test_pkg_1',
            executable='test_talker2',
            name='name_test2',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
            ),
    ])
