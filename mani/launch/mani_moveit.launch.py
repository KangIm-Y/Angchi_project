from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mani', # 실행할 ROS2 프로그램의 코드가 포함된 패키지의 이름
            executable='mani_moveit', # 실행하려는 Python 실행 파일의 이름
            output='screen'), # 프로그램의 출력을 출력할 채널
    ])
