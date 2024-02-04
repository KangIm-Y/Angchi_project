import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        qos_profile = QoSProfile(depth=10)
        self.motor_controll_listener = self.create_publisher(String, 'Motor_control', qos_profile)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5)
        
        self.header1 = 0xFF
        self.header2 = 0xFE
        self.motor_id = 0x00
        self.datasize = 0x07
        self.mode = 0x01
        self.direction = 0x00
        self.position1 = 0x46
        self.position2 = 0x50
        self.velocity1 = 0x00
        self.velocity2 = 0x32

        self.checksum = (~(self.motor_id + self.datasize + self.mode + self.direction + self.position1 + self.position2 + self.velocity1 + self.velocity2) & 0xFF)

        self.data_array = bytes([self.header1, self.header2, self.motor_id, self.datasize, self.checksum, self.mode, self.direction, self.position1, self.position2, self.velocity1, self.velocity2])

        self.timer = self.create_timer(1, self.motor_controller)
        self.count = 0

    def motor_controller(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.count)
        self.motor_controll_listener.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1

        for data in self.data_array:
            self.ser.write(data.to_bytes(1, byteorder='big'))

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()