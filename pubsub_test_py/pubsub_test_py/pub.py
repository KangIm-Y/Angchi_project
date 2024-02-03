import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Pub(Node):
   
    def __init__(self):
        super().__init__('pub')
        qos__profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(String, 'topic' ,qos_profile)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
    	msg = String()
    	msg.data = 'Hello World: %d' % self.count
    	self.publisher_.publish(msg)
    	self.get_logger().info('Publishing: "%s"' % msg.data)
    	self.count += 1
 
def main(args=None):
    rclpy.init(args=args)
    node = Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
    	node.destroy_node()
    	rclpy.shutdown()

if __name__ == '__main__':
    main()
