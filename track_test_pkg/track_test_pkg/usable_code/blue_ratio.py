import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge


class BlueRatioCirculator(Node):

    def __init__(self):
        super().__init__('BlueRatio')
        qos_profile = QoSProfile(depth=10)
        self.auto_control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        self.img_publisher = self.create_publisher(
            Image, 
            'img_data', 
            qos_profile)
        
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        ### parameters ###
        cam_num = 4
        self.U_detection_threshold = 140 ## 0~255
        self.img_size_x = 1280
        self.img_size_y = 720
        self.ROI_ratio = 0.3
        self.max_speed = 10
        
        self.odrive_mode = 1.
        self.joy_status = False
        self.joy_stick_data = [0, 0]
        
        ### dont touch parameters ###
        self.before_R_joy = 0.
        self.before_L_joy = 0.
        
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
        if ret :
            # filterd = cv2.bitwise_and(img, img, mask=U_img_treated)
            # cv2.imshow("UUUU", filterd)
            
            contours, _ = cv2.findContours(U_img_treated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            max_area = 0
            max_contour = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    max_contour = contour

            if max_contour is not None:
                max_contour_mask = np.zeros_like(U_img_treated)
                cv2.drawContours(max_contour_mask, [max_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
                
            
                filterd = cv2.bitwise_and(img, img, mask=max_contour_mask)
                cv2.imshow("UUUU", filterd)
                cv2.waitKey(1)
        
                histogram = np.sum(max_contour_mask, axis=0)
                midpoint = int(self.img_size_x / 2)
                L_histo = histogram[:midpoint]
                R_histo = histogram[midpoint:]
                # L_sum, R_sum = end_point_finder(max_contour_mask,histogram)
                
                L_sum = int(np.sum(L_histo) / 255)
                R_sum = int(np.sum(R_histo) / 255)
                
                # print(f'{L_sum}   {R_sum}')
                
                return L_sum, midpoint, R_sum
        return 1,1,1
            
            
    def end_point_finder(self, binary_image,histogram) :
        y,x = binary_image.shape
        midpoint = int(x/2)
        
        L_histo = histogram[:midpoint]
        R_histo = histogram[midpoint:]
        
        L_end = np.argmax(L_histo)
        R_end = self.img_size_x - np.argmax(R_histo[::-1])
        self.get_logger().info(f'L-diff {midpoint - L_end}   / R-diff {R_end - midpoint}')
        return L_end, midpoint, R_end
    
    
    ### img main
    def image_capture(self):
        msg = Float32MultiArray()
        ret, img = self.cap.read()
        L_joy = 0.
        R_joy = 0.

        if not ret :
            self.get_logger().info('cannot detect camera')
        else :
            
            ROI = img[int(self.img_size_y * (1-self.ROI_ratio)):,:].copy()
            cv2.rectangle(img,(0,int(self.img_size_y * (1-self.ROI_ratio))),(self.img_size_x, self.img_size_y),(255,0,0),2)
            cv2.line(img,(int(self.img_size_x / 2 ),int(self.img_size_y * (1-self.ROI_ratio))),(int(self.img_size_x / 2 ), self.img_size_y),(255,0,0),2)
            cv2.imshow("origin", img)
            cv2.waitKey(1)
            L_sum, midpoint, R_sum = self.yuv_detection(ROI)
            
            if self.joy_status == True :
                L_joy = (self.joy_stick_data[0] * 10)
                R_joy = (self.joy_stick_data[1] * 10)
            elif(((L_sum < R_sum*1.1) & (L_sum > R_sum*0.9)) | ((R_sum < L_sum*1.1) & (R_sum > L_sum*0.9))) :
                L_joy = (self.max_speed / 2)
                R_joy = (self.max_speed / 2)
            elif ((L_sum < R_sum*0.25) | (R_sum < L_sum*0.25)) :
                L_joy = (self.max_speed / 1.25 ) * (0.25 if L_sum > R_sum else 1.)
                R_joy = (self.max_speed / 1.25 ) * (0.25 if L_sum < R_sum else 1.)
            elif ((L_sum > R_sum) | (R_sum > L_sum)) :
                L_joy = (self.max_speed * (R_sum/(R_sum+L_sum)))
                R_joy = (self.max_speed * (L_sum/(R_sum+L_sum)))
            else :
                L_joy = self.before_L_joy
                R_joy = self.before_R_joy
            
            self.get_logger().info(f'{L_joy}   {R_joy}')
            
            msg.data = [self.odrive_mode, L_joy, R_joy*0.9]
            self.auto_control_publisher.publish(msg)
            
            
            resized = cv2.resize(img, (320,180),interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            self.img_publisher.publish(self.cvbrid.cv2_to_imgmsg(resized))
            
            self.before_R_joy = R_joy
            self.before_L_joy = L_joy
            
            # msg.data = [L_sum, midpoint, R_sum]
            # self.image_publisher.publish(msg)

    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] != -1 :
            self.joy_status = False
        else :
            self.joy_status = True
            self.joy_stick_data = [axes[1], axes[4]]
            
            
        

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