from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  
            executable='summer_track_checker',  
            output='screen'  
        ),
        Node(
            package='gukbang',  
            executable='odrive',  
            output='screen'  
        ),
        
        ## 대회때 제거
        # Node(
        #     package='joy',  
        #     executable='joy_node',  
        #     output='screen'  
        # )
    ])
