import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header

import pyrealsense2 as rs
import numpy as np

import cv2
from cv_bridge import CvBridge


class DepthCapture(Node):

    def __init__(self):
        super().__init__('Image_catcher')
        qos_profile = QoSProfile(depth=10)

        ##depth setting
        self.depth_frame_pub = self.create_publisher(Image, '/camera/depth/registered/image_raw', qos_profile)
        self.color_frame_pub = self.create_publisher(Image, 'color_data', qos_profile)

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(self.config)
        ##end setting

        self.timer = self.create_timer(1/30, self.depth_cap)
        self.cvbrid = CvBridge()

    def depth_cap(self):
        depth_header = Header()
        depth_header.frame_id = "camera_link_optical"

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        msg = self.cvbrid.cv2_to_imgmsg(depth_colormap)
        msg.header = depth_header
        msg.height = 480
        msg.width = 640
        msg.encoding = 'z16'
        self.depth_frame_pub.publish(msg)
        self.color_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(color_image))



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