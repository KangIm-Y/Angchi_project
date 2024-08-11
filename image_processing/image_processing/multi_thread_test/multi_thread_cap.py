import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
import pyrealsense2 as rs

import cv2
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class MultiThreadTestCap(Node):

    def __init__(self):
        super().__init__('multithread_test_capture')
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
            
        qos_profile = QoSProfile(depth=10)
        self.image_top_pub = self.create_publisher(Image, 'img_top', qos_profile)
        self.image_bot_pub = self.create_publisher(Image, 'img_bot', qos_profile)
        self.cap_timer = self.create_timer(1/30, self.image_capture, callback_group=self.group1)
        self.pub_timer = self.create_timer(1/30, self.publishs, callback_group=self.group2)
        
        ### param ###
        
        self.img_size_x = 1280
        self.img_size_y = 720
        
        #############



        ### init ###
        
        self.img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.img_top = np.zeros((int(self.img_size_y/2), self.img_size_x, 3), dtype=np.uint8)
        self.img_bot = np.zeros((int(self.img_size_y/2), self.img_size_x, 3), dtype=np.uint8)
        
        ############


        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        
        self.cvbrid = CvBridge()
        
        

    def image_capture(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            self.img = img
            self.img_top = self.img[:int(self.img_size_y/2),:]
            self.img_bot = self.img[int(self.img_size_y/2):,:]
        
        
            
    def publishs(self) :
        self.image_top_pub.publish(self.cvbrid.cv2_to_imgmsg(self.img_top))
        self.image_bot_pub.publish(self.cvbrid.cv2_to_imgmsg(self.img_bot))
        



def main(args=None):
    rclpy.init(args=args)
    node = MultiThreadTestCap()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()