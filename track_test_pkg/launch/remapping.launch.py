from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',  
            executable='joy_node',  
            output='screen',
            remappings=[
                ('/joy', '/joy/controller'),
                # ('/remapped_joy', 'joy'),
        ]  
        ),
        Node(
            package='track_test_pkg',  
            executable='joy_remapping',  
            output='screen',
            remappings=[
                # ('/joy', '/joy/controller'),
                ('/remapped_joy', 'joy'),
        ]  
        ),
        
    ])
