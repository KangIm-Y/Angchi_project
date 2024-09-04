from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  
            executable='joy_drive',  
            output='screen'  
        ),
        Node(
            package='gukbang',  
            executable='odrive',  
            output='screen'  
        ),
        Node(
            package='gukbang',  
            executable='joy_drive_sub_cam',  
            output='screen'  
        ),
    ])
