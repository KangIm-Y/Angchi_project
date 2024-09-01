from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  # 첫 번째 노드의 패키지 이름
            executable='army_detection',  # 첫 번째 노드의 실행 파일 이름
            output='screen'  # 첫 번째 노드의 출력 설정
        ),
        Node(
            package='gukbang',  
            executable='spring_col_checker',  
            output='screen'  
        ),
        Node(
            package='joy',  
            executable='joy_node',  
            output='screen'  
        )
    ])
