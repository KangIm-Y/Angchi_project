import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import numpy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class DepthIndicater(Node):

    def __init__(self):
        super().__init__('Image_indicater')
        qos_profile = QoSProfile(depth=10)
        self.depth_subscribtion = self.create_subscription(
            Image,
            'depth_data',
            self.depth_indi,
            qos_profile)
        self.color_subscribtion = self.create_subscription(
            Image,
            'color_data',
            self.color_indi,
            qos_profile)
        
        self.cvbrid = CvBridge()

    def depth_indi(self, msg):

        depth_img = self.cvbrid.imgmsg_to_cv2(msg)

        cv2.imshow("depth data", depth_img)
        cv2.waitKey(1)


    def color_indi(self, msg):
        color_img = self.cvbrid.imgmsg_to_cv2(msg)

        cv2.imshow("depth data", color_img)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = DepthIndicater()
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