import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

import time

class PersonDistancePub(Node):
    def __init__(self):
        super().__init__('Image_catcher')
        qos_profile = QoSProfile(depth=10)

        ##depth setting
        self.depth_frame_pub = self.create_publisher(Image, 'depth_data', qos_profile)
        self.color_frame_pub = self.create_publisher(Image, 'color_data', qos_profile)
        self.distance_data_pub = self.create_publisher(Float64, 'distance_data', qos_profile)

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        depth_profile = self.pipeline.start(self.config)
        
        ###align setting
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        ###align setting end
        
        
        ###end setting###
            
        ###yolo setting###
        self.model = YOLO('yolov8n.yaml')
        self.model = YOLO('yolov8n.pt')


        self.timer = self.create_timer(1/30, self.depth_cap)
        self.cvbrid = CvBridge()
        

    def depth_cap(self):
        distance_msg = Float64()
        
        frames = self.pipeline.wait_for_frames()
        
        aligned_frames = self.align.process(frames)
        
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        
        ###yolo's turn
        
        result = self.model.predict(color_image, classes=[0., 67.], conf= 0.5, max_det = 1)
        annotated_img = result[0].plot()
        if len(result[0].boxes.cls) :
            print(result[0].boxes.cls)
            object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
            
            print(object_xy[0], object_xy[1]) ### 640 by 480 
            
            distance = depth_image[object_xy[1]][object_xy[0]] * self.depth_scale    
            # print(f'distance between cam and object is {distance:.4f} meters')
            self.get_logger().info(f'distance between cam and object is {distance:.4f} meters')
            
            annotated_img = cv2.circle(annotated_img,((object_xy[0]),(object_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
            
            distance_msg.data = float(distance)
            self.distance_data_pub.publish(distance_msg)
            
        else :
            self.get_logger().info(f'any object detected')
        
        
        
        ###end of yolo
        
        
        grey_color = 0
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image) #need to search what is np.where.
        
        self.depth_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(bg_removed))
        self.color_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(annotated_img))
        
        # self.get_logger().info(f'frame rate is ///')
        


def main(args=None):
    rclpy.init(args=args)
    node = PersonDistancePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()