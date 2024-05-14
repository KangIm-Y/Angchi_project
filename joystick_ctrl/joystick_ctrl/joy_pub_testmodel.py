import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
# from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class JoyPubTestmodel(Node):
    def __init__(self):
        super().__init__('pub_for_testmodel')
        qos_profile = QoSProfile(depth=10)
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        
        self.joy_pub_testmodel= self.create_publisher(Float32MultiArray, 'joy_data', qos_profile)
        
        ## [L수직, R수직]
        self.joy_stick_data = []
        

    def joy_msg_sampling(self, msg):
        axes = msg.axes
        btn = msg.buttons
        
        
        if axes[4] != -1 :
            self.joy_stick_data = [0. , 0. ]
            self.joy_data_publish()
        else :
            self.joy_stick_data = [axes[1], axes[3]]
            self.joy_data_publish()
        
        ##L2 stick :    button [6]
        ##joystick L :  axes 0, 1 왼위가 플라스
        ##joystick R :  axes 3, 4 왼위가 플라스
        
        # self.get_logger().info(str(np.array(axes)))
        # self.get_logger().info(str(np.array(btn)))
        
    
    def joy_data_publish(self):
        msg = Float32MultiArray()
        msg.data = self.joy_stick_data
        self.joy_pub_testmodel.publish(msg)
        self.get_logger().info(str(msg.data))
        


def main(args=None):
    rclpy.init(args=args)
    node = JoyPubTestmodel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()