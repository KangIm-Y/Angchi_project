import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class Test_Talker1(Node):

    def __init__(self):
        super().__init__('test_talker1')
        qos_profile = QoSProfile(depth=10)
        self.test_talker1 = self.create_publisher(String, 'test_topic1', qos_profile)
        self.timer = self.create_timer(1, self.publish_helloworld_msg)

    def publish_helloworld_msg(self):
        msg = String()
        msg.data = "Im TALKER 1"
        self.test_talker1.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Test_Talker1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()