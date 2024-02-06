import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Publisher(Node):

    def __init__(self):
        super().__init__('pubt')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'Motor_control', qos_profile)
        self.get_logger().info('Publisher created')

    def publish_msg(self):
        user_input = input('MESSAGE: ')
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))

def main():
    rclpy.init()
    
    pubt = Publisher()

    try:
            pubt.publish_msg()
            rclpy.spin(pubt)

    except KeyboardInterrupt:
        pubt.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        pubt.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
