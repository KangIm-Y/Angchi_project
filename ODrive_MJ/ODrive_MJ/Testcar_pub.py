#Testcar_pub.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray

class Testcar_pub(Node):
    def __init__(self):
        super().__init__('TEST_CAR_PUB')
        qos_profile = QoSProfile(depth=10)
        self.count = 0
        self.publisher = self.create_publisher(Int32MultiArray, 'ANG', qos_profile)

    def publish_msg(self, vel_l,vel_r):
        msg = Int32MultiArray()
        msg.data = [vel_l,vel_r]
        self.publisher.publish(msg)
        self.get_logger().info('pub message: {0}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    pub = Testcar_pub()

    try:
        while rclpy.ok():
            vel_l,vel_r = map(int,input('CONTROL GOGO: ').split())


            print(vel_l,vel_r)
            print(type(vel_l),type(vel_r))

            pub.publish_msg(vel_l,vel_r)

    except KeyboardInterrupt:
        pub.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

