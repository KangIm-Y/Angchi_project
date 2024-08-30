import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, Joy

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge


class SpringColorChecker(Node):
    def __init__(self):
        super().__init__('spring_color_checker')
        
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=10)
        qos_profile = QoSProfile(depth=10)
        
        self.img_publisher = self.create_publisher(
            Image, 
            'col_img', 
            img_qos_profile)
        
        
        
        
        self.capture_timer = self.create_timer(1/15, self.image_capture)
        self.capture_timer = self.create_timer(1/15, self.image_processing)
        
        ### parameters ###
        self.U_detection_threshold = 130 ## 0~255
        self.img_size_x = 848
        self.img_size_y = 480
        self.depth_size_x = 848
        self.depth_size_y = 480
        
        
        ### realsense setting ###
        self.get_logger().info("try acecess to rs")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, self.img_size_x,     self.img_size_y,    rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, self.depth_size_x,   self.depth_size_y,  rs.format.z16, 15)
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
        
        
        ### declare params... dont touch parameters ###
        self.L_sum = 0
        self.R_sum = 0
        
        
        #############################################
        
        
        
        self.ROI_y_l = 0.9
        self.ROI_y_h = 0.7
        self.ROI_x_l = 0.05
        self.ROI_x_h = 0.95
        self.ROI_y = self.ROI_y_l - self.ROI_y_h
        self.ROI_x = self.ROI_x_h - self.ROI_x_l
        
        self.ROI_size = int((self.depth_size_x * (self.ROI_x_h - self.ROI_x_l)) * (self.depth_size_x * (self.ROI_y_l - self.ROI_y_h)))
        self.ROI_half_size = int(self.ROI_size / 2)
        self.get_logger().info(f'{self.ROI_size}')
        
        self.cvbrid = CvBridge()
        
        # self.color_ROI = np.zeros((int(self.ROI_y * self.img_size_y), int(self.ROI_x * self.img_size_x), 3), dtype=np.uint8)
        # self.depth_ROI = np.zeros((int(self.ROI_y * self.depth_size_y), int(self.ROI_x * self.depth_size_x), 3), dtype=np.uint8)
        self.color_img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
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
        
        
        cv2.line(self.color_img, (int(self.img_size_x/2), int(self.img_size_y * self.ROI_y_h)), (int(self.img_size_x / 2), int(self.img_size_y * self.ROI_y_l)), (0, 0, 255), 2)
        cv2.rectangle(self.color_img, (int(self.img_size_x * self.ROI_x_l),int(self.img_size_y * self.ROI_y_h)), ((int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l))), (255,0,0),2)
        
        resized = cv2.resize(self.color_img, (int(self.img_size_x/2),int(self.img_size_y/2)),interpolation=cv2.INTER_AREA)
        self.img_publisher.publish(self.cvbrid.cv2_to_imgmsg(resized))
        # cv2.imshow("color", self.color_img)
        cv2.waitKey(1)
        
   
    ########################################
            


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