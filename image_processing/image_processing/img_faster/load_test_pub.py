import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray

import numpy as np
import pyrealsense2 as rs
import time
import cv2
from cv_bridge import CvBridge


class ImageCatcher(Node):

    def __init__(self):
        super().__init__('Image_catcher')
        qos_profile = QoSProfile(depth=10)
        self.image_publisher = self.create_publisher(Image, 'img_data', qos_profile)
        self.timer = self.create_timer(1/24, self.image_capture)
        
        ### parameters ###
        
        self.img_size_x = 1920 
        self.img_size_y = 1080
        
        normal0_cam_num = 0
        normal2_cam_num = 1
        
        ##################
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        depth_profile = self.pipeline.start(self.config)
        ###align setting
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        ###align setting end

        self.cap0 = cv2.VideoCapture(normal0_cam_num)
        self.cap1 = cv2.VideoCapture(normal2_cam_num)
        self.cvbrid = CvBridge()
    
        self.cap0.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)

    def image_capture(self):
        start = time.time()
        ret0, img0 = self.cap0.read()
        ret1, img1 = self.cap1.read()
        
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        align_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        if not (ret0 | ret1 ) :
            self.get_logger().info(f'cannot detect camera {ret0} {ret1}')
        else :
            start = time.time()
            
            cv2.imshow('img0', img0)
            cv2.imshow('img1', img1)
            cv2.imshow('color_image', color_image)
            cv2.imshow('align_image', align_image)
            
            end = time.time()
            self.get_logger().info(f'{(end- start)} resize and publish complete')



def main(args=None):
    rclpy.init(args=args)
    node = ImageCatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()