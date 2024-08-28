#Testcar_pub_mode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray

class Testcar_pub(Node):
    def __init__(self):
        super().__init__('test_car_pub') 
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Float32MultiArray, 'Odrive_control', qos_profile)

    def publish_msg(self, mode, vel_l, vel_r):
        msg = Float32MultiArray()
        msg.data = [float(mode), float(vel_l), float(vel_r)]
        self.publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))  

def main(args=None):
    rclpy.init(args=args)
    pub = Testcar_pub()

    try:
        while rclpy.ok():
            try:
                mode, vel_l, vel_r = map(float, input('CONTROL GOGO: ').split())
                pub.publish_msg(mode, vel_l, vel_r)
            except ValueError: 
                print("FAIL")
    except KeyboardInterrupt:
        pub.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
