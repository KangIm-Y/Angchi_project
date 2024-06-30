import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray

import cv2
from cv_bridge import CvBridge


class BlueRatioCirculator(Node):

    def __init__(self):
        super().__init__('BlueRatio')
        qos_profile = QoSProfile(depth=10)
        self.image_publisher = self.create_publisher(Int32MultiArray, 'img_joy_data', qos_profile)
        self.timer = self.create_timer(1/24, self.image_capture)

        self.cap = cv2.VideoCapture(0)
        self.cvbrid = CvBridge()
        
        ### parameters ###
        cam_num = 4
        
        
        ##################
        self.cap = cv2.VideoCapture(cam_num)
        
        

    def image_capture(self):
        ret, img = self.cap.read()

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            self.image_publisher.publish(self.cvbrid.cv2_to_imgmsg(img))



def main(args=None):
    rclpy.init(args=args)
    node = BlueRatioCirculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()