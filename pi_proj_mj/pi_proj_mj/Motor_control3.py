import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import subprocess

class LeftNode(Node):

    def __init__(self):
        super().__init__('left_node')
        qos_profile = QoSProfile(depth=10)
        self.left_subscriber = self.create_subscription(
            String,
            'left',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
        if msg.data == 'left':
            subprocess.Popen(['ros2', 'run', 'pi_proj_mj', 'left_node'])
		        checksum = ~(0x00 + 0x07 + 0x01 + 0x00 + 0x46 + 0x50 + 0x00 + 0x32) & 0xFF
		        control_L = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x00, 0x46, 0x50, 0x00, 0x32]
		        self.ser.write(bytes(control_L))
		        rclpy.get_logger().info('LEFT')


class RightNode(Node):

    def __init__(self):
        super().__init__('right_node')
        qos_profile = QoSProfile(depth=10)
        self.right_subscriber = self.create_subscription(
            String,
            'right',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
        if msg.data == 'right':
            subprocess.Popen(['ros2', 'run', 'pi_proj_mj', 'right_node'])
		        checksum = ~(0x00 + 0x07 + 0x01 + 0x01 + 0x46 + 0x50 + 0x00 + 0x32) & 0xFF
		        control_R = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x01, 0x46, 0x50, 0x00, 0x32]
		        self.ser.write(bytes(control_R))
		        rclpy.get_logger().info('RIGHT')

def main(args=None):
    rclpy.init(args=args)
    left_node = LeftNode()
    right_node = RightNode()
		ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    try:
        rclpy.spin(left_node)
        rclpy.spin(right_node)

    except KeyboardInterrupt:
        left_node.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        left_node.destroy_node()
        right_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
