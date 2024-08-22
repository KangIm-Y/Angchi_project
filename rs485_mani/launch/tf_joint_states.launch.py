from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rs485_mani',
            executable='tf_joint_states',
            output='screen'),
    ])