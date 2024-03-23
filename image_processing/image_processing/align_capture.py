import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

import pyrealsense2 as rs
import numpy as np

import cv2
from cv_bridge import CvBridge

import time


class DepthCapture(Node):

    def __init__(self):
        super().__init__('Image_catcher')
        qos_profile = QoSProfile(depth=10)

        ##depth setting
        self.depth_frame_pub = self.create_publisher(Image, 'depth_data', qos_profile)
        self.color_frame_pub = self.create_publisher(Image, 'color_data', qos_profile)

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        depth_profile = self.pipeline.start(self.config)
        
        ###align setting
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        ###align setting end
        
        
        
        ##end setting

        self.timer = self.create_timer(1/30, self.depth_cap)
        self.cvbrid = CvBridge()

    def depth_cap(self):
        
        start_time = time.time()

        frames = self.pipeline.wait_for_frames()
        
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        align_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        grey_color = 153
        align_image_3d = np.dstack((align_image, align_image, align_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((align_image_3d > self.clipping_distance) | (align_image_3d <= 0), grey_color, color_image) #need to search what is np.where.
        
        self.depth_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(bg_removed))
        self.color_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(color_image))
        
        end_time = time.time()
        frame_rate = end_time - start_time
        self.get_logger().info(f'frame rate is {1/frame_rate}')
        


def main(args=None):
    rclpy.init(args=args)
    node = DepthCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()