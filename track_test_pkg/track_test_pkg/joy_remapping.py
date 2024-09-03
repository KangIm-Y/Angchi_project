import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


class JoyRemapper(Node):
    def __init__(self):
        super().__init__('joy_remapper')
        
        qos_profile = QoSProfile(depth=10)
        
        self.joy_publisher = self.create_publisher(
            Float32MultiArray,
            "remapped_joy",
            qos_profile
        )
        self.joy_sub = self.create_subscription(
            Joy,
            "joy/controller",
            self.joy_callback,
            qos_profile
        )
        
        self.joy_stick_data = [0.,0.]
        self.joy_status = False
        #############################################
        
        
        self.get_logger().info("ininininininit")

    def joy_callback(self, msg) :
        pub_msgs = Float32MultiArray()
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] == 1 :
            self.joy_status = False
        
        else :
            self.joy_status = True
            self.joy_stick_data = [axes[1], axes[4]]

        pub_msgs.data = self.joy_stick_data
        self.joy_publisher.publish(pub_msgs)


def main(args=None):
    rclpy.init(args=args)
    node = JoyRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()