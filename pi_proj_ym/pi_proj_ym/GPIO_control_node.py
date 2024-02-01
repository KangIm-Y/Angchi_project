import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import serial
import threading

led_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

class GPIO_control(Node):

    def __init__(self):
        super().__init__('GPIO_control_node')
        qos_profile = QoSProfile(depth=10)
        self.GPIO_control_node = self.create_subscription(
            String,
            'Motor_control',
            self.subscribe_topic_message,
            qos_profile)
        self.last_msg_data = ""
        self.timer = self.create_timer(1.0, self.LED_control)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=3)

    def subscribe_topic_message(self, msg):
        self.last_msg_data = msg.data
        print("Terminal Data Input: ", self.last_msg_data)

    def LED_control(self):
        input_data = self.last_msg_data

        if input_data == "Left":
            print("LED_control HIGH")
            GPIO.output(led_pin, GPIO.HIGH)
            self.send_serial_data("Left")  # RS485로 "Left" 데이터 송신

        elif input_data == "Right":
            print("LED_control LOW")
            GPIO.output(led_pin, GPIO.LOW)
            self.send_serial_data("Right")  # RS485로 "Right" 데이터 송신

        else:
            print("Data Empty \n")
            return

    def send_serial_data(self, data):
        str_cmd = '[' + data + ']'
        print('send data=' + str_cmd)
        self.ser.write(str_cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = GPIO_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        GPIO.cleanup()
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()