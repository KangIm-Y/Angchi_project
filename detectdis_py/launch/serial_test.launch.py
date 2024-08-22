from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detectdis_py',
            executable='serial_test',
            output='screen'),
    ])