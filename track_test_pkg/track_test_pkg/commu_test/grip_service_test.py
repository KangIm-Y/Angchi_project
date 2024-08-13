import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import time
import math
import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np
import cv2
from cv_bridge import CvBridge
from custom_interfaces.srv import PositionService


class BlueRatioCirculator(Node):
    def __init__(self):
        super().__init__('BlueRatio')
        
        qos_profile = QoSProfile(depth=10)
        self.img_publisher = self.create_publisher(
            Image, 
            'img_data', 
            qos_profile)
        
        self.joy_subscriber = self.create_subscription(
            Float32MultiArray,
            'imu_data',
            self.joy_msg_sampling,
            qos_profile)
        
        
        self.capture_timer = self.create_timer(1/24, self.image_capture)
        self.mission_timer = self.create_timer(1/24, self.mission_decision)
        self.client = self.create_client(PositionService, 'object_coordinate')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        
        
        
        ### parameters ###
        # cam_num = 4
        
        self.U_detection_threshold = 130 ## 0~255
        self.img_size_x = 1280
        self.img_size_y = 720
        self.depth_size_x = 1280
        self.depth_size_y = 720
        self.ROI_ratio = 0.3
        self.mission_ROI_ratio = 0.3
        self.max_speed = 10
        
        self.odrive_mode = 1.
        self.joy_status = False
        
        self.model_post = YOLO('/home/skh/robot_ws/src/track_test_pkg/track_test_pkg/best_add_box.pt')
        self.model_chess = YOLO('/home/skh/robot_ws/src/track_test_pkg/track_test_pkg/chess_b.pt')
        
        ### state table ###
        self.state = 'S1'
        # S0 : stand by
        # S1 
        # Spost
        # Sarm
        # Sblind
        # Sfinish
        ###################
        
        
        
        ### realsense setting ###
        self.get_logger().info("try acecess to rs")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, self.img_size_x,     self.img_size_y,    rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.depth_size_x,   self.depth_size_y,  rs.format.z16, 30)
        depth_profile = self.pipeline.start(self.config)
        
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.get_logger().info("finish acecess to rs")
        
        #########################
        
        
        ### declare area... dont touch parameters ###
        self.joy_stick_data = [0, 0]
        self.L_joy = 0.
        self.R_joy = 0.
        self.before_R_joy = 0.
        self.before_L_joy = 0.
        self.L_sum = 0
        self.R_sum = 0
        
        self.mission_positioning = False
        #############################################
        # self.cap = cv2.VideoCapture(cam_num)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size_x)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size_y)
        
        self.postbox_ROI = [[int(self.img_size_x * 0.3), int(self.img_size_y * 0.6)],[int(self.img_size_x * 0.4), int(self.img_size_y * 0.8)]]## xy xy
        self.postbox_set = False
        
        
        
        self.cvbrid = CvBridge()
        self.req = PositionService.Request()
        
        self.img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.depth_img = np.zeros((self.depth_size_y, self.depth_size_x, 3), dtype=np.uint8)
        self.get_logger().info("ininininininit")
        
        
        self.angle_data = []
        self.theta = 0.
        self.call_flag = False
        self.grip_flag = 0 ## 0 is idle, 1 is true, -1 is false
        
    def joy_msg_sampling(self,msg) :
        self.angle_data = msg.data
        self.theta = self.angle_data[1]
        

    
    def image_capture(self):
        
        frames = self.pipeline.wait_for_frames()
        
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        
        self.depth_intrinsics = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        self.depth_img = np.asanyarray(self.aligned_depth_frame.get_data())
        self.img = np.asanyarray(color_frame.get_data())
        

        # msg.data = [self.L_sum, midpoint, self.R_sum]
        # self.image_publisher.publish(msg)
        
        
    def call_service(self, x,y,z):
        if self.client.service_is_ready():
            request = PositionService.Request()
            request.coordinate.x = x
            request.coordinate.y = y
            request.coordinate.z = z
            future = self.client.call_async(request)
            future.add_done_callback(self.callback_function)
        else:
            self.get_logger().warn('Service not available')

    def callback_function(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.success}')
            self.grip_flag = 1 if response.success == True else -1
            # self.get_logger().info(self.grip_flag)
            
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

      
    def mission_decision(self) :
        
        got_ROI = self.img
        ## yolo algorithom
        result = self.model_post.predict(got_ROI, conf = 0.55, verbose=False, max_det = 1)
        
        if len(result[0].boxes.cls) :
            if self.state == 'S1' :
                self.state = 'Spost'
            elif self.state == 'Spost' :
                # print(result[0].boxes.cls)
                annotated_img = result[0].plot()
                object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
                
                # for r in result :
                #     print(r.boxes.xywh.detach().numpy().tolist()[0]) ###원래 이건데 대체했음.
                # print(object_xy[0], object_xy[1]) ### 640 by 480 
                
                distance = self.depth_img[object_xy[1]][object_xy[0]] * self.depth_scale    
                # print(f'distance between cam and object is {depth_image[object_xy[1]][object_xy[0]]}')
                # print(f'distance between cam and object is {distance:.4f} meters')
                
                annotated_img = cv2.circle(annotated_img,((object_xy[0]),(object_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
                # annotated_img = cv2.circle(annotated_img,((600),(400)),10,(255,0,0), -1, cv2.LINE_AA)
                depth = self.aligned_depth_frame.get_distance(object_xy[0], object_xy[1])
                depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [object_xy[0], object_xy[1]], depth)
                cv2.putText(annotated_img, f"{depth_point[0]:.2f}m,  {depth_point[1]:.2f}m,  {depth_point[2]:.2f}m,", (30,30), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255),2)
                x_c = depth_point[0]
                y_c = depth_point[2]
                z_c = - depth_point[1]
                
                x_w = x_c
                y_w = y_c * math.cos(self.theta / 180 * math.pi) + z_c * math.sin(self.theta / 180 * math.pi)
                z_w = (-y_c *math.sin(self.theta / 180 * math.pi)) + z_c * math.cos(self.theta / 180 * math.pi)
                
                if ((self.call_flag == False)) :
                    self.call_service(x_w, y_w, z_w)
                    self.get_logger().info("call!")
                    self.call_flag = True
                elif (self.call_flag == True) &  (self.grip_flag == -1) :
                    self.call_flag = False
                    self.grip_flag = 0
                elif self.grip_flag == 1 :
                    pass
                    
                    
                    
                # print(f'{depth_point}')
                
                cv2.imshow("title", annotated_img)
                cv2.waitKey(1)
                
                
                # self.get_logger().info(f'object : {object_xy}    postbox : {self.postbox_ROI}')
                
                if (((object_xy[0] < self.postbox_ROI[1][0])& (object_xy[0] > self.postbox_ROI[0][0])) & ((object_xy[1] < self.postbox_ROI[1][1])& (object_xy[1] > self.postbox_ROI[0][0]))) :
                    self.postbox_set = True
                    self.get_logger().info(f'setting clear')
                    # self.state = 'Sarm'
                    self.state = 'Sarm'
                else : 
                    ##### fix !!!!!!!!!!!!!!!!!!!!!!
                    ##### only for debug !!!!!!!!!!!
                    self.postbox_set = False
                    pass
                
                
                # if self.postbox_set == False :
                #     ## virtical
                #     if (object_xy[0] > self.postbox_ROI[1][0]) :
                #         self.get_logger().info(f'right')
                #     elif (object_xy[0] < self.postbox_ROI[0][0]) :
                #         self.get_logger().info(f'left')
                #     else : 
                #         pass
                        
                    
                #     ## horizonal
                #     if (object_xy[1] > self.postbox_ROI[1][1]) :
                #         self.get_logger().info(f'back')
                #     elif (object_xy[1] < self.postbox_ROI[0][1]) :
                #         self.get_logger().info(f'go')
                #     else : 
                #         pass
                    
                # else : 
                #     pass
                
        else :
            cv2.imshow('title', self.img)
            cv2.waitKey(1)
                

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