import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        qos_profile = QoSProfile(depth=10)
        self.motor_controller_node = self.create_subscription(String, 'Motor_control', self.motor_controller,qos_profile)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5)
        
        self.header1 = 0xFF
        self.header2 = 0xFE
        self.motor_id = 0x00
        self.datasize = 0x06
        self.mode = 0x03
        self.direction = 0x00
        self.velocity1 = 0x00
        self.velocity2 = 0x64
        self.duration = 0xff

    def motor_controller(self, msg):
        self.get_logger().info(f'{msg.data} is recieved')
        
        if msg.data == "LEFT" : #CW is 1 ,, CCW is 0
            self.get_logger().info(f'{msg.data} is recieved')
            self.mode = 0x00
        elif msg.data == "RIGHT" :
            self.get_logger().info(f'{msg.data} is recieved')
            self.mode = 0x01
        else :
            self.get_logger().info(f'{msg.data} is not defined')
            return
        self.update_data_array()
        self.send_data_array()
        
            
    def update_data_array(self) :
        self.checksum = (~(self.motor_id + self.datasize + self.mode + self.direction + self.velocity1 + self.velocity2 + self.duration) & 0xFF)
        self.data_array = bytes([self.header1, self.header2, self.motor_id, self.datasize, self.checksum, self.mode, self.direction, self.velocity1, self.velocity2, self.duration])
        
    def send_data_array(self) :
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