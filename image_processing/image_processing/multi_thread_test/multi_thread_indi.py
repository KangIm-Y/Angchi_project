import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class MultiThreadTestIndi(Node):

    def __init__(self):
        super().__init__('multithread_test_indicator')
        qos_profile = QoSProfile(depth=10)
        self.image_top_sub = self.create_subscription(
            Image,
            'img_top',
            self.top_callback,
            qos_profile)
        self.image_bot_sub = self.create_subscription(
            Image,
            'img_bot',
            self.bot_callback,
            qos_profile)
        self.cvbrid = CvBridge()
        
        ### params ###
        self.img_size_x = 1280
        self.img_size_y = 720
        
        self.img_top = np.zeros((int(self.img_size_y/2), self.img_size_x, 3), dtype=np.uint8)
        self.img_bot = np.zeros((int(self.img_size_y/2), self.img_size_x, 3), dtype=np.uint8)
        
        ##############
        
        
        self.img_indi_timer = self.create_timer(1/30, self.indi_callback)
        
    
    def top_callback(self, msg) :
        self.img_top = self.cvbrid.imgmsg_to_cv2(msg)
        
    def bot_callback(self, msg) :
        self.img_bot = self.cvbrid.imgmsg_to_cv2(msg)
        
    def indi_callback(self) :
        
        current_img = np.vstack((self.img_top, self.img_bot))

        cv2.imshow("title", current_img)
        cv2.waitKey(1)
        
    
    # def sub_callback(self, msg):

    #     current_img = self.cvbrid.imgmsg_to_cv2(msg)
    #     resized = cv2.resize(current_img, (1280,720), interpolation=cv2.INTER_CUBIC)

    #     cv2.imshow("title", resized)
    #     cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = MultiThreadTestIndi()
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