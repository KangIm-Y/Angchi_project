import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyRelayNode(Node):
    def __init__(self):
        super().__init__('joy_relay_node')
        
        # /joy 토픽 구독자 설정
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # /joy_drive 토픽 퍼블리셔 설정
        self.publisher_ = self.create_publisher(Joy, '/joy_drive', 10)

    def joy_callback(self, msg):
        # 받은 /joy 메시지를 그대로 /joy_drive 토픽으로 퍼블리시
        self.publisher_.publish(msg)
        #self.get_logger().info('Relayed /joy message to /joy_drive')

def main(args=None):
    rclpy.init(args=args)

    node = JoyRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
