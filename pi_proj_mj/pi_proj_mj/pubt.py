import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class LeftPublisher(Node):

    def __init__(self):
        super().__init__('left_publisher_node')
        qos_profile = QoSProfile(depth=10)
        self.left_publisher = self.create_publisher(String, 'left', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_left_msg)  
        self.count = 0

    def publish_left_msg(self):

        user_input = input('MESSAGE: ')
        msg = String()
        msg.data = 'left: {0}'.format(user_input)
        self.left_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1

class RightPublisher(Node):

    def __init__(self):
        super().__init__('right_publisher_node')
        qos_profile = QoSProfile(depth=10)
        self.right_publisher = self.create_publisher(String, 'right', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_right_msg)  
        self.count = 0

    def publish_right_msg(self):
        user_input = input('MESSAGE: ')
        msg = String()

        msg.data = 'right: {0}'.format(user_input)
        self.right_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    
    Left_p = LeftPublisher()
    Right_p = RightPublisher()

    try:
        rclpy.spin(Left_p)
        rclpy.spin(Right_p)

    except KeyboardInterrupt:
        left_node.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        Left_p.destroy_node()
        Right_p.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
