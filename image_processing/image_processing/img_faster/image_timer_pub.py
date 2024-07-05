import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray

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
            resized = cv2.resize(img, (640,360),interpolation=cv2.INTER_AREA)
            self.image_publisher.publish(self.cvbrid.cv2_to_imgmsg(resized))
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