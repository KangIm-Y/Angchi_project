import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class Test_Listener(Node):

    def __init__(self):
        super().__init__('test_listener')
        qos_profile = QoSProfile(depth=10)
        self.test_listener1 = self.create_subscription(
            String,
            'test_topic1',
            self.subscribe_topic_message1,
            qos_profile)
        self.test_listener2 = self.create_subscription(
            String,
            'test_topic2',
            self.subscribe_topic_message2,
            qos_profile)

    def subscribe_topic_message1(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
        
    def subscribe_topic_message2(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = Test_Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()