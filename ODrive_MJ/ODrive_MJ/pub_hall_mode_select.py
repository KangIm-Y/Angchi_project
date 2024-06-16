#pub_hall_mode_select.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class Subs(Node):

    def __init__(self):
        super().__init__('sub_hall')

        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String,
            'HALL',
            self.control,
            qos_profile)

    def control(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
        mode = msg.data
        if mode == 'A':
            user_input = input('MESSAGE: ')
            pubA = PublisherA()  
            pubA.publish_msg(user_input)
            pubA.destroy_node() 

        elif mode == 'B':
            user_input = input('MESSAGE: ')
            pubB = PublisherB()  
            pubB.publish_msg(user_input)
            pubB.destroy_node()  

        else:
            self.get_logger().error('Invalid mode: {0}'.format(mode))


class PublisherA(Node):

    def __init__(self):
        super().__init__('pubA')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'modeA', qos_profile)
        self.get_logger().info('Publisher A created')

    def publish_msg(self, user_input):
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)
        self.get_logger().info('Published message from A: {0}'.format(msg.data))


class PublisherB(Node):

    def __init__(self):
        super().__init__('pubB')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'modeB', qos_profile)
        self.get_logger().info('Publisher B created')

    def publish_msg(self, user_input):
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)
        self.get_logger().info('Published message from B: {0}'.format(msg.data))


def main():
    rclpy.init()
    subs = Subs()
    rclpy.spin(subs)
    subs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
