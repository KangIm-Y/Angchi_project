import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

header1 = 0xFF
header2 = 0xFE
id = 0x00
dataSize = 0x07
mode = 0x01
dirCCW = 0x00
dirCW = 0x01
position1 = 0x23
position2 = 0x28
position3 = 0x46
position4 = 0x50
velocity1 = 0x00
velocity2 = 0x32


class CamYoloSubscriber(Node):

    def __init__(self):
        super().__init__('sub_cam_yolo')
        qos_profile = QoSProfile(depth=10)
        self.cam_yolo_sub = self.create_subscription(
            String,
            'Gesture',
            self.subscribe_topic_message,
            qos_profile)
        self.last_msg_data = ""
        self.timer = self.create_timer(1.0, self.RS485_commu)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=3)
        self.trig = 0

    def subscribe_topic_message(self, msg):
        self.last_msg_data = msg.data
        self.trig = 1
        print("Topic Data Input: ", self.last_msg_data)

    def RS485_commu(self):
        input_data = self.last_msg_data

        if input_data == "Left":
            self.send_serial_data("Left") 

        elif input_data == "Right":
            self.send_serial_data("Right")

        elif input_data == "Go":
            self.send_serial_data("Go")

        elif input_data == "Stop":
            self.send_serial_data("Stop")

        elif input_data == "Feedback":
            self.send_serial_data("Feedback")

        else:
            print("Data Empty \n")
            return

    def send_serial_data(self, data):
        if self.trig == 1:
            if data == "Left":                
                checkSum = (~ (id + dataSize + mode + dirCCW + position1 + position2 + velocity1 + velocity2)) & 0xFF
                rs485_send = bytes([header1])+bytes([header2])+bytes([id])+bytes([dataSize])+bytes([checkSum])+\
                    bytes([mode])+bytes([dirCCW])+bytes([position1])+bytes([position2])+bytes([velocity1])+bytes([velocity2])
                print('send data=' + rs485_send.hex())
                self.ser.write(rs485_send)
                self.trig = 0

            elif data == "Right":                
                checkSum = (~ (id + dataSize + mode + dirCW + position3 + position4 + velocity1 + velocity2)) & 0xFF
                rs485_send = bytes([header1])+bytes([header2])+bytes([id])+bytes([dataSize])+bytes([checkSum])+\
                    bytes([mode])+bytes([dirCW])+bytes([position3])+bytes([position4])+bytes([velocity1])+bytes([velocity2])
                print('send data=' + rs485_send.hex())
                self.ser.write(rs485_send)
                self.trig = 0

            elif data == "Go":                
                checkSum = (~ (id + dataSize + mode + dirCW + position3 + position4 + velocity1 + velocity2)) & 0xFF
                rs485_send = bytes([header1])+bytes([header2])+bytes([id])+bytes([dataSize])+bytes([checkSum])+\
                    bytes([mode])+bytes([dirCW])+bytes([position3])+bytes([position4])+bytes([velocity1])+bytes([velocity2])
                print('send data=' + rs485_send.hex())
                self.ser.write(rs485_send)
                self.trig = 0

            elif data == "Stop":                
                checkSum = (~ (id + dataSize + mode + dirCW + position3 + position4 + velocity1 + velocity2)) & 0xFF
                rs485_send = bytes([header1])+bytes([header2])+bytes([id])+bytes([dataSize])+bytes([checkSum])+\
                    bytes([mode])+bytes([dirCW])+bytes([position3])+bytes([position4])+bytes([velocity1])+bytes([velocity2])
                print('send data=' + rs485_send.hex())
                self.ser.write(rs485_send)
                self.trig = 0

            elif data == "Feedback":                
                checkSum = (~ (id + dataSize + mode + dirCW + position3 + position4 + velocity1 + velocity2)) & 0xFF
                rs485_send = bytes([header1])+bytes([header2])+bytes([id])+bytes([dataSize])+bytes([checkSum])+\
                    bytes([mode])+bytes([dirCW])+bytes([position3])+bytes([position4])+bytes([velocity1])+bytes([velocity2])
                print('send data=' + rs485_send.hex())
                self.ser.write(rs485_send)
                self.trig = 0
            else:
                return

            
        else:
            return

def main(args=None):
    rclpy.init(args=args)
    node = CamYoloSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

