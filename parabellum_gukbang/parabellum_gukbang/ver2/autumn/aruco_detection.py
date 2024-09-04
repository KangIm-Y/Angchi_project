import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2.aruco as aruco

class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1  
        )
        
        self.publisher = self.create_publisher(Image,'side_camera',qos_profile)
        
        
        
        ### parameter setting ###
        self.img_size_x = 640
        self.img_size_y = 480
        self.frame_rate = 10
        
        ########################
        
        ### params ###
        self.threshold = 120
        ##############
        
        ### aruco setting ###
        
        self.aruco_dict = aruco.Dictionary_create(9, 5)
        self.aruco_dict.bytesList = np.empty(shape=(9, 4, 4), dtype=np.uint8)
        
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
            gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            
            bw0 = cv2.threshold(gray0, self.threshold, 255, cv2.THRESH_BINARY)[1]
            bw1 = cv2.threshold(gray1, self.threshold, 255, cv2.THRESH_BINARY)[1]
            
            corners0, ids0, points0 = aruco.detectMarkers(bw0, self.aruco_dict, parameters=self.aruco_param)
            corners1, ids1, points1 = aruco.detectMarkers(bw1, self.aruco_dict, parameters=self.aruco_param)
            
            img0 = aruco.drawDetectedMarkers(img0, corners0)
            img1 = aruco.drawDetectedMarkers(img1, corners0)
            
                
            # Draw custom text
            if ids0 is not None:
                for i in range(len(ids0)):
                    corner = corners0[i][0]
                    top_left = (int(corner[0][0]), int(corner[0][1]))
                    bottom_right = (int(corner[2][0]), int(corner[2][1]))
                    
                    marker_id = ids0[i][0]
                    if marker_id < len(self.marker_chars):
                        cv2.putText(img0, self.marker_chars[marker_id], top_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        
                        
            if ids1 is not None:
                for i in range(len(ids1)):
                    corner = corners1[i][0]
                    top_left = (int(corner[0][0]), int(corner[0][1]))
                    bottom_right = (int(corner[2][0]), int(corner[2][1]))
                    
                    marker_id = ids1[i][0]
                    if marker_id < len(self.marker_chars):
                        cv2.putText(img1, self.marker_chars[marker_id], top_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        
            
            frame = np.vstack((img0, img1))
            
            resized = cv2.resize(frame, (int(self.img_size_x/2),int(self.img_size_y)),interpolation=cv2.INTER_AREA)
            self.publisher.publish(self.cvbrid.cv2_to_imgmsg(resized))
        
        
        cv2.imshow('frame', frame)
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