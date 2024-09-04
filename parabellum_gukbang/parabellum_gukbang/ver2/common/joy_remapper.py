import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


class JoyRemapper(Node):
    def __init__(self):
        super().__init__('joy_remapper')
        
        qos_profile = QoSProfile(depth=10)
        
        self.remap_joy_publisher = self.create_publisher(
            Joy,
            "remapped_joy",
            qos_profile
        )
        self.indyros_joy_publisher = self.create_publisher(
            Joy,
            "joy/indy_ros",
            qos_profile
        )
        self.joy_sub = self.create_subscription(
            Joy,
            "joy/playstation",
            self.joy_callback,
            qos_profile
        )
        
        self.joy_stick_data = [0.,0.]
        self.joy_status = False
        #############################################

        ### button 9
        self.option_key_flag = False
        self.before_option_key_flag = False
        self.before_btn9 = 0


        ##False is controll,,,, True is indy-ros
        self.toggle_publish = False
        
        
        self.get_logger().info("ininininininit")

    def joy_callback(self, msg) :
        pub_msgs = Joy()
        pub_msgs = msg
        # axes = msg.axes
        btn = msg.buttons

        if self.before_btn9 != btn[9] :
            self.option_key_flag = not self.option_key_flag
            self.get_logger().info("btn toggled")

        if (self.option_key_flag == True) & (self.before_option_key_flag == False ) :
            self.toggle_publish = not self.toggle_publish
            self.get_logger().info("key option toggled")


        self.before_btn9 = btn[9]
        self.before_option_key_flag = self.option_key_flag


        if self.toggle_publish == False :
            self.remap_joy_publisher.publish(pub_msgs)
        elif self.toggle_publish == True :
            self.indyros_joy_publisher.publish(pub_msgs)
        else :
            self.remap_joy_publisher.publish(pub_msgs)
            self.get_logger().info(f'incorrect access')




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