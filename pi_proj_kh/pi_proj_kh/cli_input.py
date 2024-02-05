import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class CLIInputNode(Node):

    def __init__(self):
        super().__init__('cli_input_pub')
        qos_profile = QoSProfile(depth=10)
        self.cli_input_pub = self.create_publisher(String, 'Motor_control', qos_profile)

    def input_callbacker(self):
        self.user_input = input("input string for test motor : ")
        msg = String()
        msg.data = self.user_input
        self.cli_input_pub.publish(msg)
        self.get_logger().info(f'published : {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CLIInputNode()
    try:
        while rclpy.ok() :
            node.input_callbacker()
            rclpy.spin_once(node, timeout_sec = 1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()