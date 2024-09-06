from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='gukbang',  
        #     executable='center_joy_drive',  
        #     output='screen'  
        # ),
        
        Node(
            package='gukbang',  
            executable='center_summer_joy_drive',  
            output='screen'  
        ),
        
        
        ### moveit servo에서 킬꺼임. joy_drive 로다가
        # Node(
        #     package='joy',  
        #     executable='joy_node',  
        #     output='screen'  
        # ),
        # Node(
        #     package='gukbang',  
        #     executable='center_joy_drive_sub_cam',  
        #     output='screen'  
        # ),
    ])
