from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  # 첫 번째 노드의 패키지 이름
            executable='center_aruco_detection',  # 첫 번째 노드의 실행 파일 이름
            output='screen'  # 첫 번째 노드의 출력 설정
        ),
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
