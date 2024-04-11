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
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.image_publisher0 = self.create_publisher(Image, 'img_data0', qos_profile)
        self.image_publisher1 = self.create_publisher(Image, 'img_data1', qos_profile)
        self.image_publisher2 = self.create_publisher(Image, 'img_data2', qos_profile)
        self.image_publisher3 = self.create_publisher(Image, 'img_data3', qos_profile)
        self.timer0 = self.create_timer(1/30, self.image_capture0)
        self.timer1 = self.create_timer(1/30, self.image_capture1)
        self.timer2 = self.create_timer(1/30, self.image_capture2)
        self.timer3 = self.create_timer(1/30, self.image_capture3)

        self.cap = cv2.VideoCapture(0)
        self.cvbrid = CvBridge()

    def image_capture0(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            # img_resized = cv2.resize(img, dsize=(640,480), interpolation = cv2.INTER_AREA)
            self.image_publisher0.publish(self.cvbrid.cv2_to_imgmsg(img))
            
    def image_capture1(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            # img_resized = cv2.resize(img, dsize=(640,480), interpolation = cv2.INTER_AREA)
            self.image_publisher1.publish(self.cvbrid.cv2_to_imgmsg(img))
            
    def image_capture2(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            # img_resized = cv2.resize(img, dsize=(640,480), interpolation = cv2.INTER_AREA)
            self.image_publisher2.publish(self.cvbrid.cv2_to_imgmsg(img))
            
    def image_capture3(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            # img_resized = cv2.resize(img, dsize=(640,480), interpolation = cv2.INTER_AREA)
            self.image_publisher3.publish(self.cvbrid.cv2_to_imgmsg(img))



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