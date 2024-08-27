import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, Joy

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

import time
import math
from ultralytics import YOLO
from custom_interfaces.srv import PositionService
from std_srvs.srv import SetBool


class BlueRatioCirculator(Node):
    def __init__(self):
        super().__init__('BlueRatio')
        
        qos_profile = QoSProfile(depth=10)
        self.control_publisher = self.create_publisher(
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
        
        self.imu_subscriber = self.create_subscription(
            Float32MultiArray,
            'imu_data',
            self.imu_msg_sampling,
            QoSProfile(depth= 2))
        
        
        self.capture_timer = self.create_timer(1/24, self.image_capture)
        self.process_timer = self.create_timer(1/24, self.image_processing)
        self.pub_controll = self.create_timer(1/24, self.track_tracking)
        
        
        self.client = self.create_client(PositionService, 'pos_srv')
        self.grip_client = self.create_client(SetBool,'ActiveGripper')
        while not (self.client.wait_for_service(timeout_sec=1.0) & self.grip_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Waiting for service...')
        
        
        
        ### touch parameters ###
        self.U_detection_threshold = 130 ## 0~255
        self.img_size_x = 848
        self.img_size_y = 480
        self.depth_size_x = 848
        self.depth_size_y = 480
        self.ROI_ratio = 0.3
        self.mission_ROI_ratio = 0.3
        self.max_speed = 10
        
        self.robot_roll = 0  ## -1 left, 1 right
        self.odrive_mode = 1.
        self.joy_status = False
        self.joy_stick_data = [0, 0]
        self.before_L_joy = 0.
        self.before_R_joy = 0.
        
        self.ROI_y_l = 0.9
        self.ROI_y_h = 0.7
        self.ROI_x_l = 0.05
        self.ROI_x_h = 0.95
        
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
        self.hole_filling_filter = rs.hole_filling_filter()
        self.get_logger().info("finish acecess to rs")
        
        #########################
        
        
        ### declare area... dont touch parameters ###
        self.L_sum = 0
        self.R_sum = 0
        
        #############################################
        
        #### cal paramas ####
        
        self.ROI_y = self.ROI_y_l - self.ROI_y_h
        self.ROI_x = self.ROI_x_h - self.ROI_x_l
        
        self.ROI_size = int((self.depth_size_x * (self.ROI_x_h - self.ROI_x_l)) * (self.depth_size_x * (self.ROI_y_l - self.ROI_y_h)))
        self.ROI_half_size = int(self.ROI_size / 2)
        self.get_logger().info(f'{self.ROI_size}')
        
        self.max_dis = 1.0 / self.depth_scale
        self.min_dis = 0.75 / self.depth_scale
        
        self.cvbrid = CvBridge()
        
        self.color_ROI = np.zeros((int(self.ROI_y * self.img_size_y), int(self.ROI_x * self.img_size_x), 3), dtype=np.uint8)
        self.depth_ROI = np.zeros((int(self.ROI_y * self.depth_size_y), int(self.ROI_x * self.depth_size_x), 3), dtype=np.uint8)
        self.get_logger().info("ininininininit")
        
        self.model_post = YOLO('/home/skh/robot_ws/src/track_test_pkg/track_test_pkg/best_add_box.pt')
        
        self.state = 'S'
        self.postbox_ROI = [[int(self.img_size_x * 0.3), int(self.img_size_y * 0.45)],[int(self.img_size_x * 0.4), int(self.img_size_y * 0.55)]]## xy xy
        self.postbox_set = False
        self.mani_move = 0          # gripper success or falil
        self.grip_state = 0         # 0 is idle,,,   -1 dls false,,,   1 is True
        self.mani_state = 'home'        #'far from home', 'home', 'zero'  #### manipulator position setting
        self.third_call_flag = False
        self.dropbox_mission_flag = False

    
    def image_capture(self):
        
        frames          = self.pipeline.wait_for_frames()
        aligned_frames  = self.align.process(frames)
        
        color_frame                 = aligned_frames.get_color_frame()
        self.aligned_depth_frame    = aligned_frames.get_depth_frame()
        self.filled_depth_frame     = self.hole_filling_filter.process(self.aligned_depth_frame)
        
        self.depth_intrinsics = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        self.depth_img = np.asanyarray(self.filled_depth_frame.get_data())
        self.color_img = np.asanyarray(color_frame.get_data())
        
    def image_processing(self) :
        self.depth_ROI = self.depth_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        self.color_ROI = self.color_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        
        depth_3d = np.dstack((self.depth_ROI, self.depth_ROI, self.depth_ROI))
        depth_mask = np.where((depth_3d > self.max_dis) | (depth_3d < self.min_dis) | (depth_3d <= 0), 0, (255,255,255)).astype(np.uint8)
        
        depth, _, _ =cv2.split(depth_mask) 
        contours, _ = cv2.findContours(depth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
                
            if max_contour is not None:
                max_contour_mask = np.zeros_like(depth)
                l_sum, r_sum = self.image_spliter(depth)
                self.L_sum = l_sum
                self.R_sum = r_sum
            else :
                pass
        
        
        
        # cv2.line(self.color_img, (self.cen_x, self.roi_start_row), (int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l)), (0, 0, 255), 2)
        cv2.rectangle(self.color_img, (int(self.img_size_x * self.ROI_x_l),int(self.img_size_y * self.ROI_y_h)), ((int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l))), (255,0,0),2)
        cv2.putText(self.color_img, f'L : {l_sum:.2f} ({l_sum/ (l_sum + r_sum)})   R : {r_sum:.2f} ({r_sum / (l_sum + r_sum)})', (20,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),2)
        print(l_sum + r_sum)
        
        cv2.imshow("color", self.color_img)
        cv2.imshow("mask", depth_mask)
        cv2.waitKey(1)
        
        
    def image_spliter(self, got_img) :
        y, x = got_img.shape
        
        l = got_img[:,:int(x/2)]
        r = got_img[:,int(x/2):]
        # print(l.shape)
        # print(r.shape)
        
        
        l_sum = np.sum(l) / 255
        r_sum = (np.sum(r) / 255 ) - y
        
        return l_sum, r_sum
            
    def track_tracking(self) :
        msg = Float32MultiArray()
        if self.joy_status == True :
            self.L_joy = (self.joy_stick_data[0] * self.max_speed)
            self.R_joy = (self.joy_stick_data[1] * self.max_speed)
        elif self.dropbox_mission_flag == False :
            self.mission_decision()
        else :
            detect_sum = self.L_sum + self.R_sum
            
            if (((self.L_sum < self.R_sum*1.1) & (self.L_sum > self.R_sum*0.9)) | ((self.R_sum < self.L_sum*1.1) & (self.R_sum > self.L_sum*0.9))) :
                self.L_joy = (self.max_speed / 2)
                self.R_joy = (self.max_speed / 2)
            elif ((self.L_sum < self.R_sum*0.25) | (self.R_sum < self.L_sum*0.25)) :
                self.L_joy = (self.max_speed / 1.25 ) * (0.25 if self.L_sum > self.R_sum else 1.)
                self.R_joy = (self.max_speed / 1.25 ) * (0.25 if self.L_sum < self.R_sum else 1.)
            elif ((self.L_sum > self.R_sum) | (self.R_sum > self.L_sum)) :
                self.L_joy = (self.max_speed * (self.R_sum/(self.R_sum+self.L_sum)))
                self.R_joy = (self.max_speed * (self.L_sum/(self.R_sum+self.L_sum)))
            else :
                self.L_joy = self.before_L_joy
                self.R_joy = self.before_R_joy
                
            
        self.get_logger().info(f'{self.L_joy}   {self.R_joy}')
        
        self.before_R_joy = self.R_joy
        self.before_L_joy = self.L_joy
        
        msg.data = [self.odrive_mode, self.L_joy, self.R_joy]
        
        
            
            
    #####################################################################################################################
            
            
    def imu_msg_sampling(self, msg) :
        imu_data = msg.data
        
        if imu_data <= 75 :
            self.robot_roll = -1
        elif imu_data >= 105 :
            self.robot_roll = 1
        else :
            self.robot_roll = 0
            
        self.theta = imu_data[1]
        
    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] != -1 :
            self.joy_status = False
        else :
            self.joy_status = True
            self.joy_stick_data = [axes[1], axes[4]]
        
        
        
    #####################################################################################################################
    #####################################################################################################################
    #######################################################m#a#n#i#######################################################
    #####################################################################################################################
    #####################################################################################################################
    
    
    def call_service(self, x,y,z):
        if self.client.service_is_ready():
            request = PositionService.Request()
            self.goal_x = -x -0.1
            self.goal_y = -y -0.1
            self.goal_z = z + 0.93
            
            request.coordinate.x = self.goal_x 
            request.coordinate.y = self.goal_y 
            request.coordinate.z = self.goal_z +0.3
            
            self.get_logger().info(f'world  x : {request.coordinate.x}   y : {request.coordinate.y}  z : {request.coordinate.z}')
            future = self.client.call_async(request)
            self.mani_state = 'far from home'
                
            future.add_done_callback(self.callback_function)
        else:
            self.get_logger().warn('Service not available')

    def callback_function(self, future):
        if future.done() :
            try:
                response = future.result()
                self.get_logger().info(f'first callback Result: {response.success}')
                
                if response.success == True :
                    self.second_call_service()
                else :
                    self.call_flag = False
                
                
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
            
    
    
    
    def second_call_service(self):
        if self.client.service_is_ready():
            request = PositionService.Request()
            request.coordinate.x = self.goal_x
            request.coordinate.y = self.goal_y
            request.coordinate.z = self.goal_z
            time.sleep(2)
            self.get_logger().info(f'world  x : {self.goal_x}   y : {self.goal_y}  z : {self.goal_z}')
            future = self.client.call_async(request)
            
            future.add_done_callback(self.second_callback_function)
            
        else:
            self.get_logger().warn('Service not available')

    def second_callback_function(self, future):
        if future.done() :
            try:
                response = future.result()
                self.get_logger().info(f'Result: {response.success}')
                if response.success == False :
                    self.back()
                    time.sleep(0.5)
                    self.second_call_service()
                self.mani_move = 1 if response.success == True else -1
                
                
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
                
                
    def grip_call_service(self) :
        if self.grip_client.service_is_ready():
            request = SetBool()
            request = True
            self.get_logger().info(f'gripper start !')
            future = self.grip_client.call_async(request)
            
            future.add_done_callback(self.grip_callback_function)
            
        else:
            self.get_logger().warn('Service not available')
            
    def grip_callback_function(self, future):
        if future.done() :
            try:
                response = future.result()
                self.get_logger().info(f'Result: {response.success}')
                if response.success == False :
                    self.grip_state = -1
                elif response.success == True :
                    self.grip_state = 1
                
                
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
            
    
    def third_call_service(self):
        if self.client.service_is_ready():
            request = PositionService.Request()
            if self.mani_state == 'far from home' :
                request.pose = 'zero'
            elif self.mani_state == 'zero' :
                request.pose = 'home'
            else :
                return 0
            time.sleep(2)
            future = self.client.call_async(request)
            
            future.add_done_callback(self.second_callback_function)
            
        else:
            self.get_logger().warn('Service not available')

    def third_callback_function(self, future):
        if future.done() :
            try:
                response = future.result()
                self.get_logger().info(f'Result: {response.success}')
                
                if response.success == True :
                    if self.mani_state == 'zero' or self.mani_state == 'far from home' :
                        self.third_call_service()
                    else :
                        pass
                        
                
                
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
        


    #####################################################################################################################
      
    def mission_decision(self) :
        
        ## yolo algorithom
        result = self.model_post.predict(self.color_img, conf = 0.4, verbose=False, max_det = 1)
        
        if len(result[0].boxes.cls) :
            if self.state != 'Spost' :
                # self.state = 'Spost'
                if self.postbox_set == False :
                    ## virtical
                    if (object_xy[0] > self.postbox_ROI[1][0]) :
                        self.turn_right()
                        self.get_logger().info(f'right')
                    elif (object_xy[0] < self.postbox_ROI[0][0]) :
                        self.turn_left()
                        self.get_logger().info(f'left')
                    else : 
                        pass
                        
                    
                    ## horizonal
                    if (object_xy[1] > self.postbox_ROI[1][1]) :
                        self.back()
                        self.get_logger().info(f'back')
                    elif (object_xy[1] < self.postbox_ROI[0][1]) :
                        self.go()
                        self.get_logger().info(f'go')
                    else : 
                        pass
                      
                else : 
                    pass
                
            
                if (((object_xy[0] < self.postbox_ROI[1][0])& (object_xy[0] > self.postbox_ROI[0][0])) & ((object_xy[1] < self.postbox_ROI[1][1])& (object_xy[1] > self.postbox_ROI[0][0]))) :
                    self.postbox_set = True
                    self.get_logger().info(f'setting clear')
                    # self.state = 'Sarm'
                    self.stop()
                    self.get_logger().info(f'stop')
                    time.sleep(2)
                    self.state = 'Spost'
                else : 
                    ##### fix !!!!!!!!!!!!!!!!!!!!!!
                    ##### only for debug !!!!!!!!!!!
                    self.postbox_set = False
                    pass
                
            elif self.state == 'Spost' :
                # print(result[0].boxes.cls)
                annotated_img = result[0].plot()
                object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
                
                
                distance = self.depth_img[object_xy[1]][object_xy[0]] * self.depth_scale
                
                annotated_img = cv2.circle(annotated_img,((object_xy[0]),(object_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
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
                elif (self.call_flag == True) &  (self.mani_move == -1) :
                    self.call_flag = False
                    self.mani_move = 0
                elif self.mani_move == 1 :
                    time.sleep(3)
                    self.grip_call_service()
                    if self.grip_state == 1 :
                        self.third_call_service()
                    elif self.grip_state == -1 :
                        self.third_call_service()
                        time.sleep(2)
                        if self.mani_state == 'home' :
                            self.call_flag = False
                            self.mani_move = 0
                            self.grip_state = 0
                            self.dropbox_mission_flag = True
                        else : pass
                        
                    else : 
                        pass
                else :
                    pass
                    
                    
                
                cv2.imshow("title", annotated_img)
                cv2.waitKey(1)
                
                
                # self.get_logger().info(f'object : {object_xy}    postbox : {self.postbox_ROI}')
                
                
    
    
    
            
    #################################### control preset ####################################
    def turn_left(self) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * 0.05
        self.L_joy = - self.max_speed * 0.05
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def turn_right(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.05
        self.L_joy = self.max_speed * 0.05
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def go(self, speed_ratio) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * speed_ratio
        self.L_joy = self.max_speed * speed_ratio
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def back(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.1
        self.L_joy = - self.max_speed * 0.1
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
        
    def stop(self) :
        msg = Float32MultiArray()
        self.R_joy = 0.
        self.L_joy = 0.
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
        
            
            
            
        

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