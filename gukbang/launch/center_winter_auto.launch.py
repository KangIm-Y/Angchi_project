from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  
            executable='center_display_dep',  
            output='screen'  
        ),
        
        ## 대회때 제거
        Node(
            package='joy',  
            executable='joy_node',  
            output='screen'  
        )
    ])
