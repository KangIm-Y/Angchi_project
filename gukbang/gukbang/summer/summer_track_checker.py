import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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
        
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=1)
        
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        self.img_publisher = self.create_publisher(
            Image, 
            'img_data', 
            img_qos_profile)
        
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
        
        self.encoder_subscriber = self.create_subscription(
            Float32MultiArray,
            'Odrive_encoder',
            self.encoder_clear, 
            qos_profile
        )
        
        
        
        
        self.capture_timer = self.create_timer(1/15, self.image_capture)
        self.process_timer = self.create_timer(1/15, self.image_processing)
        self.pub_controll = self.create_timer(1/15, self.track_tracking)
        self.yolo_controll = self.create_timer(1/15, self.mission_decision)
        
        
        self.client = self.create_client(PositionService, 'pos_srv')
        self.grip_client = self.create_client(SetBool,'ActiveGripper')
        while not (self.client.wait_for_service(timeout_sec=1.0) & self.grip_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Waiting for service...')
        
        
        
        ### parameters ###
        # cam_num = 4
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
        
        
        ### realsense setting ###
        self.get_logger().info("try acecess to rs")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, self.img_size_x,     self.img_size_y,    rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, self.depth_size_x,   self.depth_size_y,  rs.format.z16, 15)
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
        
        
        self.chess_model = YOLO('/home/lattepanda/robot_ws/src/gukbang/gukbang/common/chess.pt')
        self.post_model = YOLO('/home/lattepanda/robot_ws/src/gukbang/gukbang/common/dropbox.pt')
        self.finish_ROI = [[int(self.img_size_x * 0.45), int(self.img_size_y * 0.6)],[int(self.img_size_x * 0.55), int(self.img_size_y * 0.7)]]## xy xy
        self.chess_detection_flag = False
        self.finish_flag = False
        
        ###### gripper state setting ######
        
        self.state = 'S'
        self.postbox_ROI = [[int(self.img_size_x * 0.4), int(self.img_size_y * 0.65)],[int(self.img_size_x * 0.5), int(self.img_size_y * 0.75)]]## xy xy
        self.postbox_position_set = False
        self.mani_move = 0          # gripper success or falil
        self.grip_state = 0         # 0 is idle,,,   -1 dls false,,,   1 is True
        self.mani_state = 'home'        #'far from home', 'home', 'zero'  #### manipulator position setting
        self.third_call_flag = False
        self.dropbox_mission_flag = False
        
        
        self.call_flag = False
        self.second_call_flag = False
        self.grip_flag = 0 ## 0 is idle, 1 is true, -1 is false
        self.mission_decision_flag = False
        self.grip_call_service_flag = False

        self.track_tracking_flag =False
        
        ################# for encoder #################
        self.encoder = [0.,0.]

        self.first_encoder = [0.,0.]
        self.second_encoder = [0.,0.]
        self.zeropoint_flag = False

        self.theta = 62.

        ################# for encoder #################
        
        
        
        self.ROI_y_l = 0.85
        self.ROI_y_h = 0.75
        self.ROI_x_l = 0.05
        self.ROI_x_h = 0.95
        self.ROI_y = self.ROI_y_l - self.ROI_y_h
        self.ROI_x = self.ROI_x_h - self.ROI_x_l
        
        self.ROI_size = int((self.depth_size_x * (self.ROI_x_h - self.ROI_x_l)) * (self.depth_size_x * (self.ROI_y_l - self.ROI_y_h)))
        self.ROI_half_size = int(self.ROI_size / 2)
        self.get_logger().info(f'{self.ROI_size}')
        
        self.max_dis = 0.93 / self.depth_scale
        self.min_dis = 0.75 / self.depth_scale
        
        self.cvbrid = CvBridge()
        
        self.color_ROI = np.zeros((int(self.ROI_y * self.img_size_y), int(self.ROI_x * self.img_size_x), 3), dtype=np.uint8)
        self.depth_ROI = np.zeros((int(self.ROI_y * self.depth_size_y), int(self.ROI_x * self.depth_size_x), 3), dtype=np.uint8)
        self.get_logger().info("ininininininit")
        
    def encoder_clear(self,msg) :
        self.encoder = msg.data
        
    
    def image_capture(self):
        
        frames          = self.pipeline.wait_for_frames()
        aligned_frames  = self.align.process(frames)
        
        color_frame                 = aligned_frames.get_color_frame()
        self.aligned_depth_frame    = aligned_frames.get_depth_frame()
        self.filled_depth_frame     = self.hole_filling_filter.process(self.aligned_depth_frame)
        
        self.depth_intrinsics = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        self.depth_img = np.asanyarray(self.filled_depth_frame.get_data())
        self.color_img = np.asanyarray(color_frame.get_data())



    def max_min_finder(self, got_ROI) :
        y, x = got_ROI.shape
        dis_array = got_ROI[:, int(x/2)]
        array_min = np.min(dis_array) * self.depth_scale
        array_max = np.max(dis_array) * self.depth_scale

        self.max_dis = (array_max + 0.05) / self.depth_scale
        self.min_dis = (array_max - 0.07) / self.depth_scale

        # print(array_min, array_max)


    def image_processing(self) :
        self.depth_ROI = self.depth_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        self.color_ROI = self.color_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]

        self.result = self.chess_model.predict(self.color_ROI, conf = 0.65, verbose=False, max_det=1)
        
        self.max_min_finder(self.depth_ROI)
        
        depth_3d = np.dstack((self.depth_ROI, self.depth_ROI, self.depth_ROI))
        depth_mask = np.where((depth_3d > self.max_dis) | (depth_3d < self.min_dis) | (depth_3d <= 0), 0, (255,255,255)).astype(np.uint8)
        
        depth, _, _ =cv2.split(depth_mask) 
        contours, _ = cv2.findContours(depth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        max_contour = None
        l_sum = 0
        r_sum = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            max_contour_mask = np.zeros_like(depth)
            cv2.drawContours(max_contour_mask, [max_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
            l_sum, r_sum = self.image_spliter(max_contour_mask)  # 여기에 max_contour_mask를 사용
            self.L_sum = l_sum
            self.R_sum = r_sum

            ### test!!! ###
            max_mask_3d = np.dstack((max_contour_mask, max_contour_mask, max_contour_mask))
            filter_color = cv2.bitwise_and(self.color_ROI, max_mask_3d)
            cv2.imshow("filtered_color", filter_color)
            cv2.waitKey(1)


        else:
            pass
        
        
        
        cv2.line(self.color_img, (int(self.img_size_x/2), int(self.img_size_y * self.ROI_y_h)), (int(self.img_size_x / 2), int(self.img_size_y * self.ROI_y_l)), (0, 0, 255), 2)
        cv2.rectangle(self.color_img, (int(self.img_size_x * self.ROI_x_l),int(self.img_size_y * self.ROI_y_h)), ((int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l))), (255,0,0),2)
        cv2.putText(self.color_img, f'L : {l_sum:.2f} ({l_sum/ ((l_sum + r_sum) if (l_sum + r_sum) != 0 else 1)})   R : {r_sum:.2f} ({l_sum/ ((l_sum + r_sum) if (l_sum + r_sum) != 0 else 1)})', (20,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),2)
        # print(l_sum + r_sum)
        
        cv2.imshow("color", self.color_img)
        cv2.imshow("mask", depth_mask)
        if max_contour is not None :
            cv2.imshow("mask", max_contour_mask)
        else :
            cv2.imshow("mask", depth_mask)
        cv2.waitKey(1)
        
    def image_spliter(self, got_img) :
        y, x = got_img.shape
        
        l = got_img[:,:int(x/2)]
        r = got_img[:,int(x/2):]
        
        
        l_sum = np.sum(l) / 255
        r_sum = (np.sum(r) / 255 ) - y
        
        return l_sum, r_sum
            
    def track_tracking(self) :
        msg = Float32MultiArray()
        ROI_sum = np.sum(self.depth_ROI )
        # print(ROI_sum)
        
        self.post_result = self.post_model.predict(self.color_img, conf = 0.5, verbose=False, max_det = 1)
        
        
        
        
        if self.joy_status == True :
            self.L_joy = (self.joy_stick_data[0] * self.max_speed)
            self.R_joy = (self.joy_stick_data[1] * self.max_speed)
            msg.data = [self.odrive_mode, self.L_joy, self.R_joy]   
            self.control_publisher.publish(msg)
        
        
        elif (len(self.post_result[0].boxes.cls)>0) & (self.track_tracking_flag==False):
            if self.zeropoint_flag == False :
                command = Float32MultiArray()
                command.data = [2., 0., 0.,]
                self.control_publisher.publish(command)
                self.first_encoder = self.encoder
                self.zeropoint_flag = True


            if self.state != 'Spost' :
                # self.state = 'Spost'
                if self.postbox_position_set == False :
                    self.dropbox_xy = np.array(self.post_result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
                    ## virtical
                    if (self.dropbox_xy[0] > self.postbox_ROI[1][0]) :
                        self.turn_right()
                        self.get_logger().info(f'right')
                    elif (self.dropbox_xy[0] < self.postbox_ROI[0][0]) :
                        self.turn_left()
                        self.get_logger().info(f'left')
                    else : 
                        pass
                        
                    
                    ## horizonal
                    if (self.dropbox_xy[1] > self.postbox_ROI[1][1]) :
                        self.back()
                        self.get_logger().info(f'back')
                    elif (self.dropbox_xy[1] < self.postbox_ROI[0][1]) :
                        self.go(0.3)
                        self.get_logger().info(f'go')
                    else : 
                        pass
                      
                else : 
                    self.stop()
                    print("stop")
                    pass
                
            
                if (((self.dropbox_xy[0] < self.postbox_ROI[1][0])& (self.dropbox_xy[0] > self.postbox_ROI[0][0])) & ((self.dropbox_xy[1] < self.postbox_ROI[1][1])& (self.dropbox_xy[1] > self.postbox_ROI[0][1]))) :
                    self.postbox_position_set = True
                    self.mission_decision_flag = True
                    self.get_logger().info(f'setting clear')
                    # self.state = 'Sarm'
                    self.stop()
                    self.get_logger().info(f'stop')
                    self.second_encoder = self.encoder

                    self.dropbox_xy = np.array(self.post_result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')



                    time.sleep(2)
                    self.state = 'Spost'
                else : 
                    ##### fix !!!!!!!!!!!!!!!!!!!!!!
                    ##### only for debug !!!!!!!!!!!
                    self.postbox_position_set = False
                    pass
        elif self.mission_decision_flag == True :
            self.stop()
            return
        
        
        elif len(self.result[0].boxes.cls) :
            for box in self.result[0].boxes :
                label = box.cls
                confidence = box.conf.item()
                object_xywh = np.array(box.xywh.detach().numpy().tolist()[0], dtype='int')
                self.color_img = self.result[0].plot()

                ## virtical
                if (object_xywh[0] > self.finish_ROI[1][0]) :
                    self.turn_right()
                    self.get_logger().info(f'right')
                elif (object_xywh[0] < self.finish_ROI[0][0]) :
                    self.turn_left()
                    self.get_logger().info(f'left')
                else : 
                    pass
                    
                
                ## horizonal
                if (object_xywh[1] > self.finish_ROI[1][1]) :
                    self.back()
                    self.get_logger().info(f'back')
                elif (object_xywh[1] < self.finish_ROI[0][1]) :
                    self.go(0.3)
                    self.get_logger().info(f'go')
                else : 
                    pass
                    
            else : 
                pass
            
        
            if (((object_xywh[0] < self.finish_ROI[1][0])& (object_xywh[0] > self.finish_ROI[0][0])) & ((object_xywh[1] < self.finish_ROI[1][1])& (object_xywh[1] > self.finish_ROI[0][1]))) :
                
                self.get_logger().info(f'find finish')
                self.chess_detection_flag = True
            else : 
                pass
                

        elif  ROI_sum < (self.ROI_size * 0.3) :
            print('gogogo')
            self.L_joy = self.max_speed * 0.3
            
            self.R_joy = self.max_speed * 0.3 
        else :
            detect_sum = self.L_sum + self.R_sum
            self.max_dis = 0.93 / self.depth_scale
            if (detect_sum < (self.ROI_size * 0.3) ) :
                self.L_joy = (self.max_speed / 4)
                self.R_joy = (self.max_speed / 4)
            
            elif (((self.L_sum < self.R_sum*1.1) & (self.L_sum > self.R_sum*0.9)) | ((self.R_sum < self.L_sum*1.1) & (self.R_sum > self.L_sum*0.9))) :
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
        
        if self.zeropoint_flag == True :
            pass
        else :
            self.control_publisher.publish(msg)

##########################################################################################
##########################################################################################
##########################################################################################  

   
    def call_service(self, x,y,z):
        if self.client.service_is_ready():
            request = PositionService.Request()
            self.goal_x = -x -0.1
            self.goal_y = -y -0.10 -0.03
            self.goal_z = z + 0.93 + 0.2 + 0.06
            
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
            request = SetBool.Request()
            request.data = True
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
            # if self.mani_state == 'far from home' :
            #     request.pose = 'zero'
            # elif self.mani_state == 'zero' :
            #     request.pose = 'home'
            # else :
            #     return 0
            self.state ='home'
            time.sleep(2)
            request.pose = 'home'
            future = self.client.call_async(request)
            
            future.add_done_callback(self.third_callback_function)
            
        else:
            self.get_logger().warn('Service not available')

    def third_callback_function(self, future):
        if future.done() :
            try:
                response = future.result()
                self.get_logger().info(f'Result: {response.success}')
                
                if response.success == True :
                    if self.mani_state == 'zero' or self.mani_state == 'far from home' :
                        # self.third_call_service()
                        self.get_logger().info("third ###############################################")

                    else :
                        pass
                else :
                    self.get_logger().info(f"response fail.. ")
                        
                
                
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
        



##########################################################################################
      
    def mission_decision(self) :
        
        if self.mission_decision_flag == False :
            pass
        
        else :
            
        
            ## yolo algorithom
            # result = self.post_model.predict(self.color_img, conf = 0.4, verbose=False, max_det = 1)
            
            if self.state == 'Spost' :
                # print(result[0].boxes.cls)
                annotated_img = self.post_result[0].plot()
                
                
                distance = self.depth_img[self.dropbox_xy[1]][self.dropbox_xy[0]] * self.depth_scale
                
                annotated_img = cv2.circle(annotated_img,((self.dropbox_xy[0]),(self.dropbox_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
                depth = self.aligned_depth_frame.get_distance(self.dropbox_xy[0], self.dropbox_xy[1])
                depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [self.dropbox_xy[0], self.dropbox_xy[1]], depth)
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
                    if self.grip_call_service_flag == False :
                        self.grip_call_service()
                        self.grip_call_service_flag = True 
                    else :
                        pass 
                    
                    ##success
                    if self.grip_state == 1 :
                        self.third_call_service()
                        time.sleep(2)
                        command = Float32MultiArray()
                        self.first_encoder = self.encoder
                        
                        command.data = [2., -(self.second_encoder[0] - self.first_encoder[0]), -(self.second_encoder[1] - self.first_encoder[1])]
                        self.control_publisher.publish(command)
                        self.zeropoint_flag = False

                        self.track_tracking_flag = True


                        time.sleep(3)
                        self.mission_decision_flag = False
                    elif self.grip_state == -1 :
                        self.third_call_service()
                        time.sleep(2)
                        if self.mani_state == 'home' :
                            self.call_flag = False
                            self.mani_move = 0
                            self.grip_state = 0
                            self.dropbox_mission_flag = True
                            self.grip_call_service_flag = False 

                            self.state = 'S'
                        else : pass
                        
                    else : 
                        pass
                else :
                    pass
                    
                    
                
                cv2.imshow("title", annotated_img)
                cv2.waitKey(1)
                
                
                # self.get_logger().info(f'object : {self.dropbox_xy}    postbox : {self.postbox_ROI}')      
                
            
            
            
            
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
    def imu_msg_sampling(self, msg) :
        imu_data = msg.data
        
        if imu_data[0] <= 77.5 :
            self.robot_roll = -1
        elif imu_data[0] >= 105 :
            self.robot_roll = 1
        else :
            self.robot_roll = 0

        self.theta = imu_data[1]
        
    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] == 1 :
            self.joy_status = False
        
        else :
            self.joy_status = True
            self.joy_stick_data = [axes[1], axes[4]]
        
            
    ############ control preset ############
    def turn_left(self) :
        msg = Float32MultiArray()
        self.R_joy = self.max_speed * 0.1
        self.L_joy = - self.max_speed * 0.1
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def turn_right(self) :
        msg = Float32MultiArray()
        self.R_joy = - self.max_speed * 0.1
        self.L_joy = self.max_speed * 0.1
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
    
    def soft_turn_right(self) :
        msg = Float32MultiArray()
        self.R_joy = 4.
        self.L_joy = 6.
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