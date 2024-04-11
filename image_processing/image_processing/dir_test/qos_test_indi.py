import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImageIndicater(Node):

    def __init__(self):
        super().__init__('Image_indicater')
        qos_profile = QoSProfile(
            # history=QoSHistoryPolicy.KEEP_LAST,
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            # durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.image_subscribtion0 = self.create_subscription(
            Image,
            'img_data0',
            self.sub_callback0,
            qos_profile)
        self.image_subscribtion1 = self.create_subscription(
            Image,
            'img_data1',
            self.sub_callback1,
            qos_profile)
        self.image_subscribtion2 = self.create_subscription(
            Image,
            'img_data2',
            self.sub_callback2,
            qos_profile)
        
        self.image_subscribtion3 = self.create_subscription(
            Image,
            'img_data3',
            self.sub_callback3,
            qos_profile)
        self.cvbrid = CvBridge()

    def sub_callback0(self, msg):
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        cv2.imshow("title0", current_img)
        cv2.waitKey(1)

    def sub_callback1(self, msg):
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        cv2.imshow("title1", current_img)
        cv2.waitKey(1)
        
    def sub_callback2(self, msg):
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        cv2.imshow("title2", current_img)
        cv2.waitKey(1)
        
    def sub_callback3(self, msg):
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        cv2.imshow("title3", current_img)
        cv2.waitKey(1)


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