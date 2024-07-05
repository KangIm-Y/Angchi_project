import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray

import time
import cv2
from cv_bridge import CvBridge
import numpy as np


class ImageCatchSplit(Node):

    def __init__(self):
        super().__init__('Image_catcher_split')
        qos_profile = QoSProfile(depth=10)
        self.image_publisher_h = self.create_publisher(Image, 'img_data_h', qos_profile)
        self.image_publisher_l = self.create_publisher(Image, 'img_data_l', qos_profile)
        self.timer = self.create_timer(1/24, self.image_capture)
        
        ### parameters ###
        
        self.img_size_x = 1280 
        self.img_size_y = 720
        
        cam_num = 4
        
        ##################

        self.cap = cv2.VideoCapture(cam_num)
        self.cvbrid = CvBridge()
    
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)

    def image_capture(self):
        start = time.time()
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            start = time.time()
            
            odd_pixels = np.zeros_like(img)
            even_pixels = np.zeros_like(img)
            odd_pixels[::2] = img[::2]
            even_pixels[1::2] = img[1::2]
            resized = cv2.resize(img, (640,360),interpolation=cv2.INTER_AREA)
            resize_h = resized[:int(resized.shape[0]/2),:]
            resize_l = resized[int(resized.shape[0]/2):,:]
            self.image_publisher_h.publish(self.cvbrid.cv2_to_imgmsg(resize_h))
            self.image_publisher_l.publish(self.cvbrid.cv2_to_imgmsg(resize_l))
            end = time.time()
            self.get_logger().info(f'{(end- start)} resize and publish complete')



def main(args=None):
    rclpy.init(args=args)
    node = ImageCatchSplit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()