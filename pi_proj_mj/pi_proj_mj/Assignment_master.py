import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MasterNode(Node):

    def __init__(self):
        super().__init__('master')
        qos_profile = rclpy.qos.qos_profile_system_default
        #self.subscription = self.create_subscription(String, 'study', self.callback, qos_profile)
        self.get_logger().info('master node')

    def callback(self, msg):
        if msg.data == 'stop':
            self.get_logger().info('Stopping all nodes')
            rclpy.shutdown()

def main():
    rclpy.init()  
    master_node = MasterNode()

    try:
        rclpy.spin(master_node)

    except KeyboardInterrupt:
        master_node.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
