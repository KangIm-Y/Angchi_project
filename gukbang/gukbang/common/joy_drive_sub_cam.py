import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ArmyDetectionNode(Node):
    def __init__(self):
        super().__init__('army_detection_node')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10  
        )
        self.publisher = self.create_publisher(Image,'side_camera',qos_profile)
        
        ### parameter setting ###
        self.img_size_x = 640
        self.img_size_y = 480

        self.frame_rate = 10
        
        
        
        #########################
        
        
        self.cap0 = cv2.VideoCapture('/dev/cam0')  #cam0
        self.cap1 = cv2.VideoCapture('/dev/cam1')  #cam1
        self.cvbrid = CvBridge()
        
        self.cap0.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        self.cap0.set(cv2.CAP_PROP_FPS, self.frame_rate)
        self.cap1.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        self.timer_finder = self.create_timer(1/self.frame_rate, self.image_callback)

    def image_callback(self):
        
        ret0, img0 = self.cap0.read()
        ret1, img1 = self.cap1.read()
        
        if not (ret0 & ret1) :
            if not ret0 :
                print(f'cam0 is cannot connetion')
            if not ret1 :
                print(f'cam1 is cannot connetion')
        else :
            frame = np.vstack((img0, img1))
            
            resized = cv2.resize(frame, (int(self.img_size_x/2),int(self.img_size_y)),interpolation=cv2.INTER_AREA)
            self.publisher.publish(self.cvbrid.cv2_to_imgmsg(resized))
            # cv2.imshow("Object Detection1", frame)
            cv2.waitKey(1)  # Adjust the waitKey value for the desired frame display time

def main(args=None):
    rclpy.init(args=args)
    node = ArmyDetectionNode()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()