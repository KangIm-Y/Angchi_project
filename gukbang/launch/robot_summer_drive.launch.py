from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  
            executable='robot_summer_joy_drive',  
            output='screen'  
        ),
        Node(
            package='gukbang',  
            executable='odrive',  
            output='screen'  
        ),

    #     Node(
    #         package='dynamixel_MJ',  
    #         executable='Gripper_kokomk4',  
    #         output='screen'  
    #     ),
    ])
