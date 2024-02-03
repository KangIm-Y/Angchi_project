import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def left_motor_callback(msg, ser):

    command = msg.data

    if command == 'Left':

        checksum = ~(0xFFFE + 0x03 + 0xD8) & 0xFF
        control_L = [0xFF, 0xFE, 0x00, 0x03, checksum, 0xD8]
        ser.write(bytes(control_L))

        rclpy.get_logger().info('LEFT')

def right_motor_callback(msg, ser):
    command = msg.data
    if command == 'Right':

        checksum = ~(0xFFFE + 0x03 + 0xD8) & 0xFF
        control_R = [0xFF, 0xFE, 0x00, 0x02, checksum, 0xD8]
        ser.write(bytes(control_R))

        rclpy.get_logger().info('RIGHT')

def main():

    rclpy.init()

    node = rclpy.create_node('motor_control_node')

    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    left_subscriber = node.create_subscription(
        String,
        'Motor_control',

        lambda msg: left_motor_callback(msg, ser),
        10
    )

    right_subscriber = node.create_subscription(
        String,
        'Motor_control',

        lambda msg: right_motor_callback(msg, ser),
        10
    )

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
