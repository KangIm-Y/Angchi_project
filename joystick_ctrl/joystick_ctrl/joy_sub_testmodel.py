import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
# from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import serial


class JoySubTestmodel(Node):
    def __init__(self):
        super().__init__('sub_for_testmodel')
        qos_profile = QoSProfile(depth=10)
        self.JoySubTestmodel1 = self.create_subscription(
            Float32MultiArray,
            'joy_data',
            self.circulate_joys,
            qos_profile)
        
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5)
        
        self.Lstick_data = 0x80
        self.Rstick_data = 0x00

    def circulate_joys(self, msg):
        Lstick = msg.data[0]
        Rstick = msg.data[1]
        L_mapped_data = 0
        R_mapped_data = 0
        if Lstick < 0 :
            self.Lstick_data = 0x80
            self.Lstick_data |= 0x40
            # self.Lstick_data |= 0x00
            L_mapped_data = np.uint8(((Lstick+1) / 2) * 32 )
        elif Lstick >0 : 
            self.Lstick_data = 0x80
            self.Lstick_data |= 0x40
            self.Lstick_data |= 0x20
            L_mapped_data = np.uint8(((Lstick+1) / 2) * 32 )
        else :
            self.Lstick_data = 0x80
            # self.Lstick_data |= 0x00
            # self.Lstick_data |= 0x00
            L_mapped_data =0x00
            pass
        self.Lstick_data |= L_mapped_data
        self.Lstick_data = int(self.Lstick_data)
            
        if Rstick < 0 :
            self.Rstick_data = 0x00
            self.Rstick_data |= 0x40
            self.Rstick_data |= 0x00
            R_mapped_data = np.uint8(((Lstick+1) / 2) * 32 )
        elif Rstick >0 : 
            self.Rstick_data = 0x00
            self.Rstick_data |= 0x40
            self.Rstick_data |= 0x20
            R_mapped_data = np.uint8(((Lstick+1) / 2) * 32 )
        else :
            self.Rstick_data = 0x00
            pass
        self.Rstick_data |= R_mapped_data
        self.Rstick_data = int(self.Rstick_data)
        
        self.send_rs485()
            
    def send_rs485(self) :
        self.ser.write(self.Lstick_data.to_bytes(1, byteorder='big'))
        self.ser.write(self.Rstick_data.to_bytes(1, byteorder='big'))
    
        
        


def main(args=None):
    rclpy.init(args=args)
    node = JoySubTestmodel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()