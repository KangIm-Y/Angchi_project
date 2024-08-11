import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image


import pyrealsense2 as rs
from ultralytics import YOLO
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
            qos_profile
            )
        
        
        self.img_cir_timer = self.create_timer(1/24, self.main_circulate)
        self.publish_timer = self.create_timer(1/24, self.publish_moa)
        self.capture_timer = self.create_timer(1/24, self.image_capture)
        
        
        
        
        ### parameters ###
        # cam_num = 4
        self.U_detection_threshold = 140 ## 0~255
        self.img_size_x = 1280
        self.img_size_y = 720
        self.depth_size_x = 1280
        self.depth_size_y = 720
        self.ROI_ratio = 0.3
        self.mission_ROI_ratio = 0.2
        self.max_speed = 10
        
        self.odrive_mode = 1.
        self.joy_status = False
        
        self.model = YOLO('/home/skh/robot_ws/src/track_test_pkg/track_test_pkg/best_add_box.pt')
        
        ### state table ###
        self.state = 'S0'
        # S0 : stand by
        # S1 : track tracking and find mission
        # Spost : find post state 
        # Sarm : grip post
        # S2 : sand
        # S3 : peddle
        # S4 : water
        ###################
        
        
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
        self.get_logger().info("finish acecess to rs")
        
        #########################
        
        
        ### declare area... dont touch parameters ###
        self.joy_stick_data = [0, 0]
        self.L_joy = 0.
        self.R_joy = 0.
        self.before_R_joy = 0.
        self.before_L_joy = 0.
        self.L_sum = 0
        self.R_sum = 0
        #############################################
        # self.cap = cv2.VideoCapture(cam_num)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        
        self.postbox_ROI = [[int(self.img_size_x * 0.3), int(self.img_size_y * 0.6)],[int(self.img_size_x * 0.4), int(self.img_size_y * 0.8)]]## xy xy
        self.postbox_set = False
        
        
        
        self.cvbrid = CvBridge()
        
        self.img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.depth_img = np.zeros((self.depth_size_y, self.depth_size_x, 3), dtype=np.uint8)
        self.get_logger().info("ininininininit")
        

    def yuv_detection(self, img) :
        gaussian = cv2.GaussianBlur(img, (3, 3), 1)
        yuv_img = cv2.cvtColor(gaussian, cv2.COLOR_BGR2YUV)
        Y_img, U_img, V_img = cv2.split(yuv_img)
        
        # rescale = np.clip(U_img - V_img, 0, 255).astype(np.uint8)
        ret,U_img_treated = cv2.threshold(U_img, self.U_detection_threshold, 255, cv2.THRESH_BINARY)
        if ret :
            # filterd = cv2.bitwise_and(img, img, mask=U_img_treated)
            # cv2.imshow("UUUU", filterd)
            
            contours, _ = cv2.findContours(U_img_treated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            max_area = 0
            max_contour = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    max_contour = contour

            if max_contour is not None:
                max_contour_mask = np.zeros_like(U_img_treated)
                cv2.drawContours(max_contour_mask, [max_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
            
                filterd = cv2.bitwise_and(img, img, mask=max_contour_mask)
                cv2.imshow("UUUU", filterd)
                cv2.waitKey(1)
        
                histogram = np.sum(max_contour_mask, axis=0)
                midpoint = int(self.img_size_x / 2)
                L_histo = histogram[:midpoint]
                R_histo = histogram[midpoint:]
                
                L_sum = int(np.sum(L_histo) / 255)
                R_sum = int(np.sum(R_histo) / 255)
                
                # print(f'{self.L_sum}   {self.R_sum}')
                
                return L_sum, midpoint, R_sum
        return 1,1,1
            
    
    def image_capture(self):
        
        frames = self.pipeline.wait_for_frames()
        
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        
        self.depth_intrinsics = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        self.depth_img = np.asanyarray(self.aligned_depth_frame.get_data())
        self.img = np.asanyarray(color_frame.get_data())
        

        # msg.data = [self.L_sum, midpoint, self.R_sum]
        # self.image_publisher.publish(msg)
            
                
    
    def track_tracking(self) :
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
        
        return 0
            
    def trigger(self, got_ROI) :
        total_size = int(self.img_size_y * self.mission_ROI_ratio * self.img_size_x)
        
        if self.state == 'S1' :
            ROI_L_sum, midpoint, ROI_R_sum = self.yuv_detection(got_ROI)
            total_sum = ROI_L_sum + ROI_R_sum
            
            if (total_sum <= (total_size * 0.2)) :
                self.get_logger().info('blue track miss')
                self.state = 'S2'
                
        elif self.state == 'S2' :
            ROI_L_sum, midpoint, ROI_R_sum = self.yuv_detection(got_ROI)
            total_sum = ROI_L_sum + ROI_R_sum
            
            if (total_sum >= (total_size * 0.5)) :
                self.get_logger().info('find blue track. switch state to S1')
                self.state = 'S1'
        
        
    ### img main
    ### use timer 
    def main_circulate(self) :
        ROI = self.img[int(self.img_size_y * (1-self.ROI_ratio)):,:].copy()
        mission_ROI = self.img[int(self.img_size_y * (1-(self.ROI_ratio + self.mission_ROI_ratio))):int(self.img_size_y * (1-self.ROI_ratio)),:].copy()
        
        cv2.rectangle(self.img,(0,int(self.img_size_y * (1-self.ROI_ratio))),(self.img_size_x, self.img_size_y),(255,0,0),2)
        cv2.rectangle(self.img,(0,int(self.img_size_y * (1-self.ROI_ratio- self.mission_ROI_ratio))),(self.img_size_x, int(self.img_size_y* (1-self.ROI_ratio))),(0,255,0),2)
        cv2.line(self.img,(int(self.img_size_x / 2 ),int(self.img_size_y * (1-self.ROI_ratio))),(int(self.img_size_x / 2 ), self.img_size_y),(255,0,0),2)
        cv2.imshow("origin", self.img)
        cv2.waitKey(1)
        
        self.L_sum, midpoint, self.R_sum = self.yuv_detection(ROI)
        
        if self.joy_status == True :
            self.L_joy = (self.joy_stick_data[0] * 10)
            self.R_joy = (self.joy_stick_data[1] * 10)
        else : 
            if self.state == 'S0' :
                self.get_logger().info(f'<--------------------- stand by -------------------->')
                self.get_logger().info(f'standby')
                self.state = 'S1'
                
            elif self.state == 'S1' :
                self.get_logger().info(f'<--------------------- S1 -------------------->')
                self.track_tracking()
                self.trigger(mission_ROI)
            
            elif self.state == 'S2' :
                self.get_logger().info(f'<--------------------- S2 -------------------->')
                self.trigger(mission_ROI)
            
            else :
                return 0
        
        
        
            
    def publish_moa(self) :
        msg = Float32MultiArray()
        
        ## joysitck publish
        msg.data = [self.odrive_mode, self.L_joy, self.R_joy*0.9]
        self.control_publisher.publish(msg)
        
        ## img publish
        resized = cv2.resize(self.img, (320,180),interpolation=cv2.INTER_AREA)
        self.img_publisher.publish(self.cvbrid.cv2_to_imgmsg(resized))
        
    
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
        self.R_joy = self.max_speed * 0.1
        self.L_joy = - self.max_speed * 0.1
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def turn_right(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.1
        self.L_joy = self.max_speed * 0.1
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def go(self) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * 0.1
        self.L_joy = self.max_speed * 0.1
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