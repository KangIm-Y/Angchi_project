import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy


import cv2
from cv_bridge import CvBridge


class TestCapture(Node):

    def __init__(self):
        super().__init__('communication_test_capture')
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
            img_resized = cv2.resize(img, dsize=(640,480), interpolation = cv2.INTER_AREA)
            self.image_publisher.publish(self.cvbrid.cv2_to_imgmsg(img_resized))



def main(args=None):
    rclpy.init(args=args)
    node = TestCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()