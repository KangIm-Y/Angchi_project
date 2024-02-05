import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def left2_motor_callback(msg, ser):
    command = msg.data

    if command == 'Left':
        checksum = ~(0x00 + 0x07 + 0x01 + 0x00 + 0x46 + 0x50 + 0x00 + 0x32) & 0xFF
        control_L = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x00, 0x46, 0x50, 0x00, 0x32]
        ser.write(bytes(control_L))
        rclpy.get_logger().info('LEFT')

def right2_motor_callback(msg, ser):
    command = msg.data

    if command == 'Right':
        checksum = ~(0x00 + 0x07 + 0x01 + 0x01 + 0x46 + 0x50 + 0x00 + 0x32) & 0xFF
        control_R = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x01, 0x46, 0x50, 0x00, 0x32]
        ser.write(bytes(control_R))
        rclpy.get_logger().info('RIGHT')



def main():
    rclpy.init()

    node = rclpy.create_node('motor_control_node')

    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    left_received = False  
    right_received = False  

    def check_messages():
        nonlocal left_received, right_received
        if left_received and right_received:
            rclpy.get_logger().info('Both LEFT and RIGHT messages received.')
        else:
            rclpy.get_logger().info('False')

    left_subscriber = node.create_subscription(
        String,
        'Motor_control',
        lambda msg: left2_motor_callback(msg, ser),
        10
    )

    right_subscriber = node.create_subscription(
        String,
        'Motor_control',
        lambda msg: right2_motor_callback(msg, ser),
        10
    )

    rclpy.spin_once(node)  

    check_messages() 

    while not (left_received and right_received):
        rclpy.spin_once(node)
        check_messages()

    rclpy.shutdown()
