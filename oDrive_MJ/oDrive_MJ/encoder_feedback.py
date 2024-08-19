#Testcar_sub_mode.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray

class encoder_sub(Node):
    def __init__(self):
        super().__init__('encoder_subscriber')

        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'encoder',
            self.subscribe_encoder,
            qos_profile)


    def subscribe_encoder(self, msg):
        self.get_logger().info('Encoder Value : axis0 = {}, axis1 = {}'.format(msg.data[0], msg.data[1]))

def main(args=None):
    rclpy.init(args=args)
    encoder_subs = encoder_sub()
    rclpy.spin(encoder_subs)
    encoder_subs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
