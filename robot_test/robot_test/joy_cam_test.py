import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

import numpy as np
import pyrealsense2 as rs

class JoyCamTest(Node) :
    def __init__(self):
        super().__init__('BlueRatio')
        qos_profile = QoSProfile(depth=10)
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        
        
        ### parameters ###
        self.img_size_x = 1280
        self.img_size_y = 720
        self.depth_size_x = 1280
        self.depth_size_y = 720
        
        
        self.odrive_mode = 1.
        self.joy_status = False
        self.joy_stick_data = [0, 0]
        
        ### dont touch parameters ###
        self.before_R_joy = 0.
        self.before_L_joy = 0.
        
        ##################
        
        ### realsense setting ###
        self.get_logger().info("try acecess to rs")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, self.img_size_x,     self.img_size_y,    rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.depth_size_x,   self.depth_size_y,  rs.format.z16, 30)
        depth_profile = self.pipeline.start(self.config)
        
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # clipping_distance_in_meters = 1 #1 meter
        # self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.hole_filling_filter = rs.hole_filling_filter()
        self.get_logger().info("finish acecess to rs")
        
        #########################
    
        self.timer1 = self.create_timer(1/30, self.joy_msg_sampling)
        self.timer2 = self.create_timer(1/24, self.camera_capture)
        
        
    def camera_capture(self) :
        
        
        
        return 0
    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        btn = msg.buttons

        if axes[2] != -1 :
            self.joy_status = False
        else :
            if btn[2] == 1 :
                self.joy_status = True
                self.joy_stick_data = [5.,5.]
                self.get_logger().info(f'\033[92msoft go\033[0m')
            elif btn [1] == 1 :
                self.joy_status = True
                self.joy_stick_data = [5., -5.]
                self.get_logger().info(f'\033[92msoft turn left\033[0m')
            elif btn [3] == 1 :
                self.joy_status = True
                self.joy_stick_data = [-5., 5.]
                self.get_logger().info(f'\033[92msoft turn right\033[0m')
            elif btn [0] == 1 :
                self.joy_status = True
                self.joy_stick_data = [-5., -5.]
                self.get_logger().info(f'\033[92msoft back\033[0m')
            else :
                self.joy_status = True
                self.joy_stick_data = [axes[1], axes[4]]
            
    def joy_pub(self) :
        msg = Float32MultiArray()
        
        if self.joy_status == False :
            msg.data = [self.odrive_mode ,0., 0.]
        else :
            msg.data = [self.odrive_mode ,self.joy_stick_data[0], self.joy_stick_data[1]]
            
        self.control_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = JoyCamTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()