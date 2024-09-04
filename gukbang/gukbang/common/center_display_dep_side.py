import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


import cv2
from cv_bridge import CvBridge



class CenterDisplay(Node):
    def __init__(self):
        super().__init__('display_node')
        
        qos_profile = QoSProfile(depth=10)
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=1)
        
        
        self.img_subscriber = self.create_subscription(
            Image,
            'img_data',
            self.img_indicater,
            img_qos_profile)
        self.image_subscribtion = self.create_subscription(
            Image,
            'side_camera',
            self.sub_callback,
            img_qos_profile)
        self.cvbrid = CvBridge()
        
    
    def img_indicater(self, msg) :
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        y,x,c = current_img.shape
        resized = cv2.resize(current_img, (int(x*1.5),int(y*1.5)), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("img_data", resized)
        cv2.waitKey(1)
        
    def sub_callback(self, msg):
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        y,x,c = current_img.shape
        resized = cv2.resize(current_img, (int(x*1.5),int(y*1.5)), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("side_camera", resized)
        cv2.waitKey(1)

        
        

def main(args=None):
    rclpy.init(args=args)
    node = CenterDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()