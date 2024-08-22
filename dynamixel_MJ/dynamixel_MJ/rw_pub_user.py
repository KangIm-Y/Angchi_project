import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile

class DynamixelPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Int32, 'dynamixel_goal_position', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback) 

    def timer_callback(self):
        user_input = int(input("ANGLE: "))  
        msg = Int32()
        #msg.data = -int(user_input*11.375)
        msg.data = int(user_input)
        self.publisher.publish(msg)
        self.get_logger().info(f'Current Position: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

