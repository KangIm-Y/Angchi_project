import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class MotorPublisherNode(Node):

    def __init__(self):
        super().__init__('motor_publisher_node')
        qos_profile = QoSProfile(depth=10)
        self.left_publisher = self.create_publisher(String, 'left', qos_profile)
        self.right_publisher = self.create_publisher(String, 'right', qos_profile)

    def publish_left_message(self):
        msg = String()
        msg.data = 'left'
        self.left_publisher.publish(msg)
        self.get_logger().info('Publishing LEFT message.')

    def publish_right_message(self):
        msg = String()
        msg.data = 'right'
        self.right_publisher.publish(msg)
        self.get_logger().info('Publishing RIGHT message.')

def main(args=None):
    rclpy.init(args=args)
    motor_publisher_node = MotorPublisherNode()

    try:
        while rclpy.ok():
            user_input = input("Enter 'left' or 'right' : ")
            if user_input.lower() == 'left':
                motor_publisher_node.publish_left_message()
            elif user_input.lower() == 'right':
                motor_publisher_node.publish_right_message()
            else:
                motor_publisher_node.get_logger().info('Fail')
    except KeyboardInterrupt:
        pass

    motor_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
