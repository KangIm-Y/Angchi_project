import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, Image


import cv2
from cv_bridge import CvBridge



class SpringColorChecker(Node):
    def __init__(self):
        super().__init__('spring_color_checker')
        
        qos_profile = QoSProfile(depth=10)
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=1)
        
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        self.img_subscriber = self.create_subscription(
            Image,
            'col_img',
            self.img_indicater,
            img_qos_profile)
        
        self.max_speed = 10
        self.odrive_mode = 1.
        self.cvbrid = CvBridge()
    
    def img_indicater(self, msg) :
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        y,x,c = current_img.shape
        resized = cv2.resize(current_img, (int(x*1.5),int(y*1.5)), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("col_img", resized)
        cv2.waitKey(1)


    def joy_msg_sampling(self, msg):
        axes = msg.axes
        btn = msg.buttons

        if not (axes[2] == 1) :
            self.max_speed = 10
            if btn[2] == 1 :
                self.go()
            elif btn[1] == 1 :
                self.turn_right()
            elif btn[3] == 1:
                self.turn_left()
            elif btn[0] == 1 :
                self.back()
            else : 
                self.joy_stick_data = [axes[1], axes[4]]
                self.joy_pub()
        elif not (axes[5] == 1) :
            self.max_speed = 18
            if btn[2] == 1 :
                self.go()
            elif btn[1] == 1 :
                self.turn_right()
            elif btn[3] == 1:
                self.turn_left()
            elif btn[0] == 1 :
                self.back()
            else : 
                self.joy_stick_data = [axes[1], axes[4]]
                self.joy_pub()
        else :
            self.stop()
            
    def joy_pub(self) :
        msg = Float32MultiArray()
        msg.data = [self.odrive_mode,self.joy_stick_data[0] * self.max_speed ,self.joy_stick_data[1] * self.max_speed ]
        self.control_publisher.publish(msg)
        
        
        
    ########################################
    ############ control preset ############
    ########################################
    def turn_left(self) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * 0.5
        self.L_joy = - self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def turn_right(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.5
        self.L_joy = self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def go(self) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * 0.5
        self.L_joy = self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def back(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.5
        self.L_joy = - self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
        
    def stop(self) :
        msg = Float32MultiArray()
        self.R_joy = 0.
        self.L_joy = 0.
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
        
            
            
        

def main(args=None):
    rclpy.init(args=args)
    node = SpringColorChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()