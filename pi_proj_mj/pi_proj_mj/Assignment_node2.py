import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Node2(Node):

    def __init__(self):
        super().__init__('pubsub_node2')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'study', qos_profile)
        self.subscription = self.create_subscription(String, 'master', self.callback, qos_profile)
        self.get_logger().info('Publisher and Subscriber created')

    def publish_msg(self):
        user_input = input('MESSAGE: ')
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)
        self.get_logger().info(format(msg.data))

    def callback(self, msg):
        if msg.data == '2':
            self.get_logger().info('Node1 START')
            start_node1()

def start_node1():
    rclpy.init()
    node1 = Node1()

    try:
        rclpy.spin(node1)
    finally:
        node1.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node2 = Node2()

    try:
        rclpy.spin(node2)
    finally:
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
