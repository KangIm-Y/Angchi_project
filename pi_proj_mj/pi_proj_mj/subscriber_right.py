import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class RightNode(Node):

    def __init__(self):
        super().__init__('right_node')
        qos_profile = QoSProfile(depth=10)
        self.motor_control_sub = self.create_subscription(
            String,
            'Motor_control',
            self.subscribe_topic_message,
            qos_profile)
        self.ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))

        if msg.data == 'right':
            checksum = ~(0x00 + 0x07 + 0x01 + 0x01 + 0x46 + 0x50 + 0x00 + 0x32) & 0xFF
            control_R = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x01, 0x46, 0x50, 0x00, 0x32]
            self.ser.write(control_R)
            rclpy.get_logger().info('RIGHT')

def main():
    rclpy.init()
    Right = RightNode()

    try:
        while rclpy.ok():
            rclpy.spin(Right)

    except KeyboardInterrupt:
        Right.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        Right.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

