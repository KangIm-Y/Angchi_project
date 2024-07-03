import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy

import numpy as np
import cv2
from cv_bridge import CvBridge


class BlueRatioCirculator(Node):

    def __init__(self):
        super().__init__('BlueRatio')
        qos_profile = QoSProfile(depth=10)
        self.image_publisher = self.create_publisher(
            Int32MultiArray, 
            'img_joy_data', 
            qos_profile)
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        
        ### parameters ###
        cam_num = 0
        self.U_detection_threshold = 140 ## 0~255
        self.img_size_x = 1280
        self.img_size_y = 720
        self.ROI_ratio = 0.3
        self.max_speed = 10
        
        
        ##################
        self.cap = cv2.VideoCapture(cam_num)
        self.cvbrid = CvBridge()
    
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        
        
        self.timer = self.create_timer(1/24, self.image_capture)
        

    def yuv_detection(self, img) :
        gaussian = cv2.GaussianBlur(img, (3, 3), 1)
        yuv_img = cv2.cvtColor(gaussian, cv2.COLOR_BGR2YUV)
        Y_img, U_img, V_img = cv2.split(yuv_img)
        
        # rescale = np.clip(U_img - V_img, 0, 255).astype(np.uint8)
        ret,U_img_treated = cv2.threshold(U_img, self.U_detection_threshold, 255, cv2.THRESH_BINARY)
        histogram = np.sum(U_img_treated, axis=0 )
        L_end, midpoint, R_end = self.end_point_finder(U_img_treated,histogram)
        if ret :
            filterd = cv2.bitwise_and(img, img, mask=U_img_treated)
            cv2.imshow("UUUU", filterd)
            cv2.waitKey(1)
            
        return L_end, midpoint, R_end
            
            
    def end_point_finder(self, binary_image,histogram) :
        y,x = binary_image.shape
        midpoint = int(x/2)
        
        L_histo = histogram[:midpoint]
        R_histo = histogram[midpoint:]
        
        L_end = np.argmax(L_histo)
        R_end = self.img_size_x - np.argmax(R_histo[::-1])
        self.get_logger().info(f'L-diff {midpoint - L_end}   / R-diff {R_end - midpoint}')
        return L_end, midpoint, R_end
    
    
    ### image main
    def image_capture(self):
        msg = Int32MultiArray()
        ret, img = self.cap.read()
        L_joy = 0
        R_joy = 0

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            L_end, midpoint, R_end = self.yuv_detection(img)
            
            if((L_end < R_end*1.1) & (L_end > R_end*0.9)) | ((R_end < L_end*1.1) & (R_end > L_end*0.9)) :
                L_joy = int(self.max_speed / 2)
                R_joy = int(self.max_speed / 2)
            elif (L_end > R_end) :
                L_joy = int(L_end / R_end * self.max_speed)
            
            # msg.data = [L_end, midpoint, R_end]
            # self.image_publisher.publish(msg)

    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] != -1 :
            pass
        else :
            self.joy_stick_data = [axes[1], axes[4]]
            # self.joy_data_publish()
        

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