import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class SemiProjMaster(Node):

    def __init__(self):
        super().__init__('master_order')
        qos_profile = QoSProfile(depth=10)
        self.master_publisher1 = self.create_publisher(
            String, 
            'order', 
            qos_profile
        )
        
        self.data = 0
        
    def cli_input(self):
        self.user_input = input("input master command : ")
        
        msg = String()
        msg.data = self.user_input
        self.master_publisher1.publish(msg)
        
        self.get_logger().info(f'{msg.data} is published')
        
def main(args=None):
    rclpy.init(args=args)
    node = SemiProjMaster()
    try:
        while rclpy.ok() :
            node.cli_input()
            rclpy.spin_once(node, timeout_sec = 1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()