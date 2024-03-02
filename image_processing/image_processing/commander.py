import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('Arduino_commander_node')
        
        qos_profile = QoSProfile(depth=10)
        self.Arduino_commander_node = self.create_subscription(String, 'arduino_command', self.arduino_controller,qos_profile)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5)
        

    def arduino_controller(self, msg):
        
        if msg.data == "none" : #cannot detect
            self.get_logger().info(f'{msg.data} is recieved')
            self.direction = 0x56
        elif msg.data == "nope" : #rock hand
            self.get_logger().info(f'{msg.data} is recieved')
            self.direction = 0x60
        elif msg.data == "yeah" : #V
            self.get_logger().info(f'{msg.data} is recieved')
            self.direction = 0x58
        elif msg.data == "Hello" : #high five
            self.get_logger().info(f'{msg.data} is recieved')
            self.direction = 0x59
        else :
            self.get_logger().info(f'{msg.data} is not defined')
            self.direction = 0x60
            return
        
        self.send_data()
        
    def send_data(self) :
        self.ser.write(self.direction.to_bytes(1, byteorder='big'))
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommander()
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