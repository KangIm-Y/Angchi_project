import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Publisher(Node):

    def __init__(self):
        super().__init__('pubt')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'Motor_control', qos_profile)
        self.subscription = self.create_subscription(
            String,
            'Motor_control',
            self.subscribe_topic_message,
            qos_profile)
        self.get_logger().info('Publisher created')

    def publish_msg(self):
        user_input = input('MESSAGE: ')
        msg = String()
        msg.data = 'input: {0}'.format(user_input)
        self.publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))

    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    
    pubt = Publisher()

    try:
        rclpy.spin(pubt)

    except KeyboardInterrupt:
        pubt.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        pubt.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
