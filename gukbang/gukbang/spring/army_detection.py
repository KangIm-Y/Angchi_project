import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import QoSProfile

class ArmyDetectionNode(Node):
    def __init__(self):
        super().__init__('army_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO('/home/skh/robot_ws/src/gukbang/gukbang/common/army.pt')  # 모델 파일 위치 .pt
        
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Image,'side_camera',qos_profile)
        
        ### parameter setting ###
        self.img_size_x = 1280
        self.img_size_y = 720
        
        
        
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
            frame = np.hstack((img0, img0))
            
            result = self.model.predict(frame, conf = 0.4, verbose=False)
            
            if len(result[0].boxes.cls) :
                for box in result[0].boxes :
                    label = box.cls
                    confidence = box.conf.item()
                    object_xyxy = np.array(box.xyxy.detach().numpy().tolist()[0], dtype='int')
                    color = [255,255,255]
                    if label == 0 :
                        color =[0,255,0]
                        cv2.putText(frame, f'Korea ARMY  {(confidence*100):.2f}%', (object_xyxy[0], object_xyxy[1] - 20),cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                    else :
                        color = [0,0,255]
                        cv2.putText(frame, f'ENEMY  {(confidence*100):.2f}%', (object_xyxy[0], object_xyxy[1] - 20), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                    cv2.rectangle(frame, (object_xyxy[0], object_xyxy[1]), (object_xyxy[2], object_xyxy[3]), color, 2)
            
            
            
            cv2.imshow("Object Detection1", frame)
            cv2.waitKey(1)  # Adjust the waitKey value for the desired frame display time

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ArmyDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()