from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  
            executable='center_display_dep',  
            output='screen'  
        ),
        
        # Node(
        #     package='joy',  
        #     executable='joy_node',  
        #     output='screen',
        #     remappings=[
        #         ('/joy', 'joy_drive'),
        # ]  
        # )
    ])
