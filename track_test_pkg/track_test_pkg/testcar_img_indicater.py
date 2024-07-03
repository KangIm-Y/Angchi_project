import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImageIndicater(Node):

    def __init__(self):
        super().__init__('Image_indicater')
        qos_profile = QoSProfile(depth=10)
        self.image_subscribtion = self.create_subscription(
            Image,
            'img_data',
            self.sub_callback,
            qos_profile)
        self.cvbrid = CvBridge()

    def sub_callback(self, msg):

        current_img = self.cvbrid.imgmsg_to_cv2(msg)

        cv2.imshow("robot_img", current_img)
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