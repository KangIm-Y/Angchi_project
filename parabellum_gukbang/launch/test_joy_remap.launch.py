from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',  
            executable='joy_node',  
            output='screen',
            remappings=[
                ('/joy', '/joy/playstation'),
        ]  
        ),
        Node(
            package='gukbang',  
            executable='joy_remapper',  
            output='screen',
            remappings=[
                ('/remapped_joy', 'joy'),
        ]  
        ),
        
    ])
