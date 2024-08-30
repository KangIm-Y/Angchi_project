import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import QoSProfile

import cv2.aruco as aruco

class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Image,'side_camera',qos_profile)
        
        ### parameter setting ###
        self.img_size_x = 1280
        self.img_size_y = 720
        
        
        ### aruco setting ###
        
        self.aruco_dict = aruco.Dictionary_create(9, 5)
        self.aruco_dict.bytesList = np.empty(shape=(9, 4, 4), dtype=np.uint8)
        # Define markers
        self.markers = [
            [[1,0,0,0,1],[1,0,0,1,0],[1,1,1,0,0],[1,0,0,1,0],[1,0,0,0,1]],  # K
            [[0,1,1,1,0],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]],  # O
            [[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0],[1,0,0,1,0],[1,0,0,0,1]],  # R
            [[1,1,1,1,1],[1,0,0,0,0],[1,1,1,1,0],[1,0,0,0,0],[1,1,1,1,1]],  # E
            [[0,0,1,0,0],[0,1,0,1,0],[1,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1]],  # A
            [[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0],[1,0,0,1,0],[1,0,0,0,1]],  # R
            [[1,0,0,0,1],[1,1,0,1,1],[1,0,1,0,1],[1,0,0,0,1],[1,0,0,0,1]],  # M
            [[1,0,0,0,1],[0,1,0,1,0],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0]],  # Y
            [[0,1,0,1,0],[1,1,1,1,1],[1,1,1,1,1],[0,1,1,1,0],[0,0,1,0,0]]   # heart
        ]
        
        for i, bits in enumerate(self.markers):
            mybits = np.array(bits, dtype=np.uint8)
            self.aruco_dict.bytesList[i] = aruco.Dictionary.getByteListFromBits(mybits)
        self.marker_chars = ["K", "O", "R", "E", "A", "R", "M", "Y", "Heart"]    
        self.aruco_param = aruco.DetectorParameters_create()
        
        #########################
        
        
        self.cap0 = cv2.VideoCapture(0)  #cam0
        self.cap1 = cv2.VideoCapture(2)  #cam1
        self.cvbrid = CvBridge()
        
        self.cap0.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        
        self.timer_finder = self.create_timer(1/24, self.image_callback)

    def image_callback(self):
        
        ret0, img0 = self.cap0.read()
        ret1, img1 = self.cap1.read()
        
        if not (ret0 & ret1) :
            if not ret0 :
                print(f'cam0 is cannot connetion')
            if not ret1 :
                print(f'cam1 is cannot connetion')
        else :
            gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
            # gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            
            bw0 = cv2.threshold(gray0, 100, 255, cv2.THRESH_BINARY)[1]
            # bw1 = cv2.threshold(gray1, 128, 255, cv2.THRESH_BINARY)[1]
            
            corners0, ids0, points0 = aruco.detectMarkers(bw0, self.aruco_dict, parameters=self.aruco_param)
            
            img0 = aruco.drawDetectedMarkers(img0, corners0)
            
                
            # Draw custom text
            if ids0 is not None:
                for i in range(len(ids0)):
                    corner = corners0[i][0]
                    top_left = (int(corner[0][0]), int(corner[0][1]))
                    bottom_right = (int(corner[2][0]), int(corner[2][1]))
                    # center = (int((top_left[0] + bottom_right[0]) / 2), int((top_left[1] + bottom_right[1]) / 2))
                    
                    marker_id = ids0[i][0]
                    if marker_id < len(self.marker_chars):
                        cv2.putText(img0, self.marker_chars[marker_id], top_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Display the resulting frame
        cv2.imshow('bw', bw0)
        cv2.imshow('frame', img0)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetection()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally :
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()