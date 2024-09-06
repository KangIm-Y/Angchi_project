from launch import LaunchDescription
from launch_ros.actions import Node
import serial
import time

ser = serial.Serial('/dev/ttyArduino', 9600, timeout=5)
time.sleep(2)
ser.write(b'a')        
ser.write('a'.encode())


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gukbang',  # 첫 번째 노드의 패키지 이름
            executable='army_detection',  # 첫 번째 노드의 실행 파일 이름
            output='screen'  # 첫 번째 노드의 출력 설정
        ),
        Node(
            package='gukbang',  
            executable='joy_drive',  
            output='screen'  
        ),
        Node(
            package='gukbang',  
            executable='odrive',  
            output='screen'  
        ),
    ])
