import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile

class TripPub(Node):
    def __init__(self):
        super().__init__('dynamixel_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Int32MultiArray, 'dynamixel_double', qos_profile)

    def callback(self, ID4, ID5, ID6):       
   
        msg = Int32MultiArray()
        msg.data = [ID4, ID5, ID6]

        #msg.data = -int(user_input/0.088)
        self.publisher.publish(msg)
        #self.get_logger().info('Published message: {0}'.format(msg.data))
        self.get_logger().info(f'ID4: {msg.data[0]}')
        self.get_logger().info(f'ID5: {msg.data[1]}')
        self.get_logger().info(f'ID6: {msg.data[2]}')



def main(args=None):
    rclpy.init(args=args)
    node = TripPub()
    try:
        while rclpy.ok():
            try:
                ID4, ID5, ID6 = map(int, input('INPUT: ').split())
                node.callback(ID4, ID5, ID6)
            except ValueError: 
                print("FAIL")
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

