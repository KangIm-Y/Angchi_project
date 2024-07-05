import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import numpy as np


class ImageIndicater(Node):

    def __init__(self):
        super().__init__('Image_indicater')
        qos_profile = QoSProfile(depth=10)
        self.image_subscribtion_h = self.create_subscription(
            Image,
            'img_data_h',
            self.sub_callback_h,
            qos_profile)
        
        self.image_subscribtion_l = self.create_subscription(
            Image,
            'img_data_l',
            self.sub_callback_l,
            qos_profile)
        self.cvbrid = CvBridge()
        
        self.timer = self.create_timer(1/24, self.sub_callback_main)
        
        self.image_h = np.zeros((180, 640, 3), dtype=np.uint8)
        self.image_l = np.zeros((180, 640, 3), dtype=np.uint8)

    def sub_callback_h(self, msg):
        self.image_h = self.cvbrid.imgmsg_to_cv2(msg)

    def sub_callback_l(self, msg):
        self.image_l = self.cvbrid.imgmsg_to_cv2(msg)

    def sub_callback_main(self):
        start = time.time()
        
        current_img = np.vstack((self.image_h, self.image_l))
        resized = cv2.resize(current_img, (1280,720), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("title", resized)
        cv2.waitKey(1)
        end = time.time()
        self.get_logger().info(f'{(end-start)} subscribe and resize time')



def main(args=None):
    rclpy.init(args=args)
    node = ImageIndicater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()