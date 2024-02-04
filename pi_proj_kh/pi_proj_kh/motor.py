import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

header1 = 0xFF
header2 = 0xFE
motor_id = 0x00
datasize = 0x07
mode = 0x01
direction = 0x00
position1 = 0x46
position2 = 0x50
velocity1 = 0x00
velocity2 = 0x32

checksum = (~(motor_id + datasize + mode + direction + position1 + position2 + velocity1 + velocity2)&0xFF)

data_array = bytes([header1, header2, motor_id, datasize, checksum, mode, direction, position1, position2, velocity1, velocity2])

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        qos_profile = QoSProfile(depth=10)
        self.motor_controll_listener = self.create_publisher(String, 'Motor_control', qos_profile)
        
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5)

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