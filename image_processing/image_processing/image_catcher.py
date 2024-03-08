import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge


class ImageCatcher(Node):

    def __init__(self):
        super().__init__('Image_catcher')
        qos_profile = QoSProfile(depth=10)
        self.image_publisher = self.create_publisher(Image, 'img_data', qos_profile)
        self.timer = self.create_timer(0.1, self.image_capture)

        self.cap = cv2.VideoCapture(0)
        self.cvbrid = CvBridge()

    def image_capture(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            self.image_publisher.publish(self.cvbrid.cv2_to_imgmsg(img))



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