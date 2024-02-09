import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from std_msgs.msg import String
from sensor_msgs.msg import Joy


class TestJoystickSub(Node):

    def __init__(self):
        super().__init__('Test_joynode')
        qos_profile = QoSProfile(depth=10)
        self.TestJoystickSub1 = self.create_subscription(
            Joy,
            'joy',
            self.subscribe_joy_message,
            qos_profile)

    def subscribe_joy_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
        


def main(args=None):
    rclpy.init(args=args)
    node = TestJoystickSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()