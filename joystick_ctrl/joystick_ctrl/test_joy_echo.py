import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from std_msgs.msg import String
from sensor_msgs.msg import Joy


class TestJoystickEcho(Node):

    def __init__(self):
        super().__init__('joy_data_echo')
        qos_profile = QoSProfile(depth=10)
        self.TestJoystickSub1 = self.create_subscription(
            Joy,
            'joy',
            self.echo_joy_message,
            qos_profile)

    def echo_joy_message(self, msg):
        axes = msg.axes
        btn = msg.buttons

        self.get_logger().info(f'Axes : {axes[0],axes[1]}')
        self.get_logger().info(f'buttons : {btn[0],btn[1],btn[2],btn[3]}')
        


def main(args=None):
    rclpy.init(args=args)
    node = TestJoystickEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()