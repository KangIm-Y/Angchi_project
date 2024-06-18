import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from ultralytics import YOLO
import math

from cv_bridge import CvBridge

img_size_x = 1280
img_size_y = 720
HFOV = 69   #degree
VFOV = 42
DFOV = 77
focal_length = 0.00193 #m
pixel_size_color = 0.0000014 #m
pixel_size_depth = 0.000003 #m

class JoyPubTestmodel(Node):
    def __init__(self):
        super().__init__('pub_for_testmodel')
        qos_profile = QoSProfile(depth=10)
        self.auto_person_tracking= self.create_publisher(Float32MultiArray, 'joy_data', qos_profile)
        
        
        
        self.cap = cv2.VideoCapture(0)
        self.cvbrid = CvBridge()
        
        ## [L수직, R수직]
        self.joy_stick_data = []
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth,img_size_x,img_size_y, rs.format.z16, 30)
        config.enable_stream(rs.stream.color,img_size_x,img_size_y, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)

        ###0.0010000000474974513
        ###fuxking this scale value means, pixel number per 1m
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        # print(profile.get_device().first_depth_sensor().get_self.depth_scale())


        clipping_distance_in_meters = 1 #1 meter
        clipping_distance = clipping_distance_in_meters / self.depth_scale
        align_to = rs.stream.color
        self.align = rs.align(align_to)


        ###yolo setting###
        # model = YOLO('yolov8n.yaml')
        self.model = YOLO('yolov8n.pt')
        
        
        self.max_power = 1.
        
        self.L_data = 0.
        self.R_data = 0.
        
        
        self.timer = self.create_timer(1/30, self.find_person)
        
        
    def find_person(self) :
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())
        
        result = self.model.predict(self.color_image, classes=[0], conf= 0.6, max_det = 1)
        annotated_img = result[0].plot()

        
        self.object_xy = 0
        if len(result[0].boxes.cls) :
            print(result[0].boxes.cls)
            self.object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
            
            # for r in result :
            #     print(r.boxes.xywh.detach().numpy().tolist()[0]) ###원래 이건데 대체했음.
            print(self.object_xy[0], self.object_xy[1]) ### 640 by 480 
            self.distance = self.depth_image[self.object_xy[1]][self.object_xy[0]] * self.depth_scale 
            annotated_img = cv2.circle(annotated_img,((self.object_xy[0]),(self.object_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
            # annotated_img = cv2.circle(annotated_img,((600),(400)),10,(255,0,0), -1, cv2.LINE_AA)
            
            self.control_box()
            
        else :
            self.get_logger().info('any object detected')
            self.stop()
            

        # cv2.imshow("title", self.color_image)
        cv2.imshow('yolo', annotated_img)
        cv2.waitKey(1)
        
    def control_box(self) :
        if len(self.object_xy) != 0 :
            target_x = self.object_xy[0]
            midpoint = self.color_image.shape[1]
            
            diff = (target_x - midpoint)
            
            ## l is -1 # r is +1
            self.direction = 0
            if diff > 0 :
                self.direction = 1
            elif diff < 0 : 
                self.direction = -1
            else :
                self.direction = 0
            
            if self.distance >1.5 :
                diff_distance = abs(self.distance * (diff/img_size_x * 1920 * pixel_size_color) /focal_length)
                theta = math.atan2(diff_distance,self.distance)
                self.go(theta)
                
                pass
            elif self.distance <1.3 :
                diff_distance = abs(self.distance * (diff/img_size_x * 1920 * pixel_size_color) /focal_length)
                theta = math.atan2(diff_distance,self.distance)
                self.back(theta)
                pass
            else :
                self.stop()
                pass
            
            
        else :
            self.stop()
        
    def go(self, theta):
        if self.direction ==1 :
            L_data = self.max_power * (theta/(math.pi/3.2))
            R_data = self.max_power * (((math.pi / 3.2) - theta)/(math.pi/3.2))
        elif self.direction ==-1 :
            L_data = self.max_power * (((math.pi / 3.2) - theta)/(math.pi/3.2)) 
            R_data = self.max_power * (theta/(math.pi/3.2))
        else : 
            self.stop()
        self.joy_stick_data = [L_data, R_data]
        
        self.joy_data_publish()
    
    def back(self, theta) :
        if self.direction ==-1 :
            L_data = self.max_power * (theta/(math.pi/3.2)) * -1
            R_data = self.max_power * (((math.pi / 3.2) - theta)/(math.pi/3.2)) * -1
        elif self.direction ==1 :
            L_data = self.max_power * (((math.pi / 3.2) - theta)/(math.pi/3.2))  * -1
            R_data = self.max_power * (theta/(math.pi/3.2)) * -1
        else : 
            self.stop()
        self.joy_stick_data = [L_data, R_data]
        
        self.joy_data_publish()
    
    def stop(self) :
        self.joy_stick_data = [0., 0.]
        
        self.joy_data_publish()
        
    def joy_data_publish(self):
        msg = Float32MultiArray()
        msg.data = self.joy_stick_data
        self.auto_person_tracking.publish(msg)
        self.get_logger().info(str(msg.data))
        


def main(args=None):
    rclpy.init(args=args)
    node = JoyPubTestmodel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()