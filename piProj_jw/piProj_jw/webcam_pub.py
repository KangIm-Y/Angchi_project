import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('frame_pub')
        
        qos_profile = QoSProfile(depth=10)
        self.frame_publisher = self.create_publisher(Image, 'webcam_frame', qos_profile=qos_profile)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Webcam Frame Receive Error')
        
        self.frame_publisher.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Frame Publish Start')
        cv2.imshow('frame_pub', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        webcam_publisher.get_logger().info('KeyboardInterrupt (SIGINT)')
    finally:
        webcam_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
