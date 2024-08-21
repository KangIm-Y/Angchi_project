#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArrowSubscriber(Node):
    def __init__(self):
        super().__init__('arrow_subscriber')

        # 서브스크라이버 초기화
        self.create_subscription(
            String,
            'arrow',
            self.callback,
            10
        )

    def callback(self, msg):
        # 수신한 메시지를 터미널에 출력
        self.get_logger().info(f'Received Arrow Info: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArrowSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 정리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
