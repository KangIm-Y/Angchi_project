import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial
import threading


global header1
global header2
global id
global dataSize
global mode

header1 = 0xFF
header2 = 0xFE
id = 0x00
dataSize = 0x02
mode = 0x00



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

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
        self.trig = 0
        
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()

    def subscribe_topic_message(self, msg):
        self.last_msg_data = msg.data
        self.trig = 1
        print("Topic Data Input: ", self.last_msg_data)

    def RS485_commu(self):
        input_data = self.last_msg_data

        if input_data == "Go":
            self.send_serial_data(0)

        elif input_data == "Left":
            self.send_serial_data(1) 

        elif input_data == "Right":
            self.send_serial_data(2)

        elif input_data == "Stop":
            self.send_serial_data(3)

        elif input_data == "Feedback":
            self.send_serial_data(4)

        else:
            print("Data Empty \n")
            return

    def send_serial_data(self, data):
        if self.trig == 1:
            mode = int(hex(data),16)
            checkSum = (~ (id + dataSize + mode)) & 0xFF
            rs485_send = bytes([header1])+bytes([header2])+bytes([id])+bytes([dataSize])+bytes([checkSum])+bytes([mode])
            print('send data=' + rs485_send.hex())
            self.ser.write(rs485_send)
            self.trig = 0
        else:
            return
        
    def serial_reader(self):
        while True:
            received_data = self.ser.readline().strip()
            if received_data:
                hex_data = ' '.join([f'{byte:02X}' for byte in received_data])
                print("RS485 Receive data:", hex_data)


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

