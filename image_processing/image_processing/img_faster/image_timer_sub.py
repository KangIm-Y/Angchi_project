import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time


class ImageIndicater(Node):

    def __init__(self):
        super().__init__('Image_indicater')
        qos_profile = QoSProfile(depth=1)
        self.image_subscribtion = self.create_subscription(
            Image,
            'img_data',
            self.sub_callback,
            qos_profile)
        self.cvbrid = CvBridge()

    def sub_callback(self, msg):
        start = time.time()

        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        resized = cv2.resize(current_img, (720,405), interpolation=cv2.INTER_CUBIC)

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