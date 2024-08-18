import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, Joy

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge


class BlueRatioCirculator(Node):
    def __init__(self):
        super().__init__('BlueRatio')
        
        qos_profile = QoSProfile(depth=10)
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        self.img_publisher = self.create_publisher(
            Image, 
            'img_data', 
            qos_profile)
        
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        
        
        self.capture_timer = self.create_timer(1/24, self.image_capture)
        self.process_timer = self.create_timer(1/24, self.image_processing)
        self.pub_controll = self.create_timer(1/24, self.track_tracking)
        
        
        
        
        ### parameters ###
        # cam_num = 4
        self.U_detection_threshold = 130 ## 0~255
        self.img_size_x = 1280
        self.img_size_y = 720
        self.depth_size_x = 1280
        self.depth_size_y = 720
        self.ROI_ratio = 0.3
        self.mission_ROI_ratio = 0.3
        self.max_speed = 10
        
        self.odrive_mode = 1.
        self.joy_status = False
        self.joy_stick_data = [0, 0]
        
        
        ### realsense setting ###
        self.get_logger().info("try acecess to rs")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, self.img_size_x,     self.img_size_y,    rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.depth_size_x,   self.depth_size_y,  rs.format.z16, 30)
        depth_profile = self.pipeline.start(self.config)
        
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.hole_filling_filter = rs.hole_filling_filter()
        self.get_logger().info("finish acecess to rs")
        
        #########################
        
        
        ### declare area... dont touch parameters ###
        self.L_sum = 0
        self.R_sum = 0
        
        #############################################
        
        
        
        self.ROI_y_l = 0.9
        self.ROI_y_h = 0.7
        self.ROI_x_l = 0.1
        self.ROI_x_h = 0.9
        self.ROI_y = self.ROI_y_l - self.ROI_y_h
        self.ROI_x = self.ROI_x_h - self.ROI_x_l
        
        self.max_dis = 0.9 / self.depth_scale
        self.min_dis = 0.75 / self.depth_scale
        
        self.cvbrid = CvBridge()
        
        self.color_ROI = np.zeros((int(self.ROI_y * self.img_size_y), int(self.ROI_x * self.img_size_x), 3), dtype=np.uint8)
        self.depth_ROI = np.zeros((int(self.ROI_y * self.depth_size_y), int(self.ROI_x * self.depth_size_x), 3), dtype=np.uint8)
        self.get_logger().info("ininininininit")
        

    
    def image_capture(self):
        
        frames          = self.pipeline.wait_for_frames()
        aligned_frames  = self.align.process(frames)
        
        color_frame                 = aligned_frames.get_color_frame()
        self.aligned_depth_frame    = aligned_frames.get_depth_frame()
        self.filled_depth_frame     = self.hole_filling_filter.process(self.aligned_depth_frame)
        
        self.depth_intrinsics = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        self.depth_img = np.asanyarray(self.filled_depth_frame.get_data())
        self.color_img = np.asanyarray(color_frame.get_data())
        
    def image_processing(self) :
        self.depth_ROI = self.depth_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        self.color_ROI = self.color_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        
        depth_3d = np.dstack((self.depth_ROI, self.depth_ROI, self.depth_ROI))
        depth_mask = np.where((depth_3d > self.max_dis) | (depth_3d < self.min_dis) | (depth_3d <= 0), 0, self.color_ROI)
        
        l_sum, r_sum = self.image_spliter(depth_mask)
        self.L_sum = l_sum
        self.R_sum = r_sum
        
        
        cv2.line(self.color_img, (self.cen_x, self.roi_start_row), (int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l)), (0, 0, 255), 2)
        cv2.rectangle(self.color_img, (int(self.img_size_x * self.ROI_x_l),int(self.img_size_y * self.ROI_y_h)), ((int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l))), (255,0,0),2)
        cv2.putText(self.color_img, f'L : {l_sum:.2f} ({l_sum/ (l_sum + r_sum)})   R : {r_sum:.2f} ({r_sum / (l_sum + r_sum)})', (20,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),2)
        
        cv2.imshow("color", self.color_img)
        cv2.imshow("mask", depth_mask)
        cv2.waitKey(1)
        
        
    def image_spliter(self, got_img) :
        y, x, c = got_img.shape
        
        l = got_img[:,:int(x/2)]
        r = got_img[:,int(x/2):]
        print(l.shape)
        print(r.shape)
        
        
        l_sum = np.sum(l) / 255
        r_sum = np.sum(r) / 255
        
        return l_sum, r_sum
            
    def track_tracking(self) :
        msg = Float32MultiArray()
        if self.joy_status == True :
            self.L_joy = (self.joy_stick_data[0] * self.max_speed)
            self.R_joy = (self.joy_stick_data[1] * self.max_speed)
        else :
            if (((self.L_sum < self.R_sum*1.1) & (self.L_sum > self.R_sum*0.9)) | ((self.R_sum < self.L_sum*1.1) & (self.R_sum > self.L_sum*0.9))) :
                self.L_joy = (self.max_speed / 2)
                self.R_joy = (self.max_speed / 2)
            elif ((self.L_sum < self.R_sum*0.25) | (self.R_sum < self.L_sum*0.25)) :
                self.L_joy = (self.max_speed / 1.25 ) * (0.25 if self.L_sum > self.R_sum else 1.)
                self.R_joy = (self.max_speed / 1.25 ) * (0.25 if self.L_sum < self.R_sum else 1.)
            elif ((self.L_sum > self.R_sum) | (self.R_sum > self.L_sum)) :
                self.L_joy = (self.max_speed * (self.R_sum/(self.R_sum+self.L_sum)))
                self.R_joy = (self.max_speed * (self.L_sum/(self.R_sum+self.L_sum)))
            else :
                self.L_joy = self.before_L_joy
                self.R_joy = self.before_R_joy
            
        self.get_logger().info(f'{self.L_joy}   {self.R_joy}')
        
        self.before_R_joy = self.R_joy
        self.before_L_joy = self.L_joy
        
        msg.data = [self.odrive_mode, self.L_joy, self.R_joy]
        
        
            
            
            
            
            
    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] != -1 :
            self.joy_status = False
        else :
            self.joy_status = True
            self.joy_stick_data = [axes[1], axes[4]]
        
        
            
    ############ control preset ############
    def turn_left(self) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * 0.05
        self.L_joy = - self.max_speed * 0.05
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def turn_right(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.05
        self.L_joy = self.max_speed * 0.05
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def go(self, speed_ratio) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * speed_ratio
        self.L_joy = self.max_speed * speed_ratio
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def back(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.1
        self.L_joy = - self.max_speed * 0.1
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
    node = BlueRatioCirculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()