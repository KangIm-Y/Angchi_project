import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class CLI_Input_node(Node):

    def __init__(self):
        super().__init__('cli_input_pub')
        qos_profile = QoSProfile(depth=10)
        self.cli_input_pub = self.create_publisher(String, 'Motor_control', qos_profile)
        self.timer = self.create_timer(1, self.input_callback)

    def input_callback(self):
        user_input = input("input string for test motor")
        
        msg = String()
        msg.data = user_input
        self.cli_input_pub.publish(msg)
        self.get_logger().info(f'발행된 메시지 : {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CLI_Input_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()