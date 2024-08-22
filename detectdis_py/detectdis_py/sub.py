import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32


class Sub(Node):
    def __init__(self):
         super().__init__('sub')
         qos_profile = QoSProfile(depth=10)
         self.sub = self.create_subscription(
              Int32,
              'distance',
              self.listener_callback,
              qos_profile
         )
         
    def listener_callback(self, msg):
         self.get_logger().info('Received message: {0}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = Sub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt  (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        