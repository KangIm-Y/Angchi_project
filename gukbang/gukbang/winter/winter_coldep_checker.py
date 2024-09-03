import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, Joy

from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
import time


class SpringColorChecker(Node):
    def __init__(self):
        super().__init__('spring_color_checker')
        
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=1)
        qos_profile = QoSProfile(depth=10)
        
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
        
        
        self.capture_timer = self.create_timer(1/15, self.image_capture)
        self.process_timer = self.create_timer(1/15, self.image_processing)
        self.pub_controll = self.create_timer(1/15, self.track_tracking)
        self.roll_timer = self.create_timer(1/15, self.roll_finder)
        
        ### parameters ###
        self.U_detection_threshold = 130 ## 0~255
        self.img_size_x = 848
        self.img_size_y = 480
        self.depth_size_x = 848
        self.depth_size_y = 480
        
        self.max_speed = 12
        
        
        
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
        
        
        ### declare params... dont touch parameters ###
        self.L_sum = 0
        self.R_sum = 0
        
        self.robot_roll = 0  ## -1 left, 1 right
        self.odrive_mode = 1.
        self.joy_status = False
        self.joy_stick_data = [0, 0]
        self.before_L_joy = 0.
        self.before_R_joy = 0.
        
        #############################################
        
        self.chess_model = YOLO('/home/lattepanda/robot_ws/src/gukbang/gukbang/common/chess.pt')
        self.finish_ROI = [[int(self.img_size_x * 0.45), int(self.img_size_y * 0.6)],[int(self.img_size_x * 0.55), int(self.img_size_y * 0.7)]]## xy xy
        self.chess_detection_flag = False
        self.finish_flag = False
        
        
        ############################ ratio of ROI img #########################
        self.slant_drive_min = 0.4
        self.slant_drive_max = 0.45
        ############################ ratio of ROI img #########################
        
        
        self.ROI_y_l = 0.9
        self.ROI_y_h = 0.7
        self.ROI_x_l = 0.05
        self.ROI_x_h = 0.95
        self.ROI_y = self.ROI_y_l - self.ROI_y_h
        self.ROI_x = self.ROI_x_h - self.ROI_x_l
        
        self.ROI_size = int((self.depth_size_x * (self.ROI_x_h - self.ROI_x_l)) * (self.depth_size_x * (self.ROI_y_l - self.ROI_y_h)))
        self.ROI_half_size = int(self.ROI_size / 2)
        self.get_logger().info(f'{self.ROI_size}')
        
        self.cvbrid = CvBridge()
        
        self.color_ROI = np.zeros((int(self.ROI_y * self.img_size_y), int(self.ROI_x * self.img_size_x), 3), dtype=np.uint8)
        self.depth_ROI = np.zeros((int(self.ROI_y * self.depth_size_y), int(self.ROI_x * self.depth_size_x), 3), dtype=np.uint8)
        
        ############################ dep_checker params #########################
        
        self.max_dis = 0.93 / self.depth_scale
        self.min_dis = 0.75 / self.depth_scale
        self.ahead_roll = False
        
        
        ############################ dep_checker params #########################
        
        
        
        
        self.get_logger().info("ininininininit")
        
    
    def image_capture(self):
        
        frames          = self.pipeline.wait_for_frames()
        aligned_frames  = self.align.process(frames)
        
        color_frame                 = aligned_frames.get_color_frame()
        self.aligned_depth_frame    = aligned_frames.get_depth_frame()
        self.filled_depth_frame     = self.hole_filling_filter.process(self.aligned_depth_frame)
        
        self.depth_intrinsics = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        self.depth_img = np.asanyarray(self.filled_depth_frame.get_data())
        self.color_img = np.asanyarray(color_frame.get_data())
        


    def yuv_detection(self, img) :
        y, x, c = img.shape
        
        gaussian = cv2.GaussianBlur(img, (3, 3), 1)
        yuv_img = cv2.cvtColor(gaussian, cv2.COLOR_BGR2YUV)
        Y_img, U_img, V_img = cv2.split(yuv_img)
        
        # rescale = np.clip(U_img - V_img, 0, 255).astype(np.uint8)
        ret,U_img_treated = cv2.threshold(U_img, self.U_detection_threshold, 255, cv2.THRESH_BINARY)
        
        # resized = cv2.resize(U_img_treated, (424,240),interpolation=cv2.INTER_AREA)
        # self.img_publisher.publish(self.cvbrid.cv2_to_imgmsg(U_img_treated))
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
                
                L_sum = int(np.sum(L_histo) / 255)
                R_sum = int(np.sum(R_histo) / 255) - y
                
                # print(f'{L_sum}   {R_sum}')
                self.img_publisher.publish(self.cvbrid.cv2_to_imgmsg(filterd))
                
                return L_sum, midpoint, R_sum
        return 1,1,1
    

    def max_min_finder(self, got_ROI) :
        y, x = got_ROI.shape
        dis_array = got_ROI[:, int(x/2)]
        array_min = np.min(dis_array) * self.depth_scale
        array_max = np.max(dis_array) * self.depth_scale

        self.max_dis = (array_max + 0.05) / self.depth_scale
        self.min_dis = (array_max - 0.07) / self.depth_scale

        # print(array_min, array_max)
    
    
    def image_spliter(self, got_img) :
        y, x = got_img.shape
        
        l = got_img[:,:int(x/2)]
        r = got_img[:,int(x/2):]
        
        
        l_sum = np.sum(l) / 255
        r_sum = (np.sum(r) / 255 ) - y
        
        return l_sum, r_sum
        
    def image_processing(self) :
        self.color_ROI = self.color_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        
        self.depth_ROI = self.depth_img[int(self.img_size_y * self.ROI_y_h):int(self.img_size_y * self.ROI_y_l),int(self.img_size_x * self.ROI_x_l):int(self.img_size_x * self.ROI_x_h)]
        
        
        self.result = self.chess_model.predict(self.color_ROI, conf = 0.4, verbose=False, max_det=1)
        
        if self.robot_roll == 0 :
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
                
                # 최대 컨투어가 없는 경우 처리
                pass
            
        
        elif self.robot_roll == 1 :
            
            l_sum, midpoint, r_sum = self.yuv_detection(self.color_ROI)
            self.L_sum = l_sum
            self.R_sum = r_sum 
        
        cv2.imshow("ROI", self.color_ROI)
        
        cv2.line(self.color_img, (int(self.img_size_x/2), int(self.img_size_y * self.ROI_y_h)), (int(self.img_size_x / 2), int(self.img_size_y * self.ROI_y_l)), (0, 0, 255), 2)
        cv2.rectangle(self.color_img, (int(self.img_size_x * self.ROI_x_l),int(self.img_size_y * self.ROI_y_h)), ((int(self.img_size_x * self.ROI_x_h), int(self.img_size_y * self.ROI_y_l))), (255,0,0),2)
        cv2.putText(self.color_img, f'L : {self.L_sum:.2f} ({self.L_sum/ ((self.L_sum + self.R_sum) if (self.L_sum + self.R_sum) != 0 else 1)})   R : {self.R_sum:.2f} ({self.L_sum/ ((self.L_sum + self.R_sum) if (self.L_sum + self.R_sum) != 0 else 1)})', (20,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),2)
        
        cv2.imshow("color", self.color_img)
        cv2.waitKey(1)
        
            
    def track_tracking(self) :
        msg = Float32MultiArray()
        if self.joy_status == True :
            self.L_joy = (self.joy_stick_data[0] * self.max_speed)
            self.R_joy = (self.joy_stick_data[1] * self.max_speed)
        else :
            detect_sum = self.L_sum + self.R_sum
            if self.finish_flag == True :
                self.stop()
                
            elif self.chess_detection_flag == True :
                self.go(10)
                time.sleep(3)
                self.stop()
                self.finish_flag = True
                
                
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
                        self.go(0.5)
                        self.get_logger().info(f'go')
                    else : 
                        pass
                      
                else : 
                    pass
                
            
                if (((object_xywh[0] < self.finish_ROI[1][0])& (object_xywh[0] > self.finish_ROI[0][0])) & ((object_xywh[1] < self.finish_ROI[1][1])& (object_xywh[1] > self.finish_ROI[0][0]))) :
                    
                    self.get_logger().info(f'find finish')
                    self.chess_detection_flag = True
                else : 
                    pass
                
            elif self.robot_roll == 0 :
                ### depth 주행.
                
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
                
                
            # turn right        
            elif self.robot_roll == 1 :
                ### color 주행.

                if ((self.R_sum < (self.ROI_half_size * self.slant_drive_max)) & (self.R_sum > (self.ROI_half_size * self.slant_drive_min))) :
                    self.L_joy = (self.max_speed / 2)
                    self.R_joy = (self.max_speed / 2)
                    self.get_logger().info(f'normal')
                elif self.R_sum >= (self.ROI_half_size * self.slant_drive_max) :
                    self.L_joy = (self.max_speed / 2) + ((self.max_speed / 4) * (self.R_sum / self.ROI_half_size))
                    self.R_joy = (self.max_speed / 2) - ((self.max_speed / 4) * (self.R_sum / self.ROI_half_size))
                    self.get_logger().info(f'right')
                elif self.R_sum <= self.ROI_half_size * self.slant_drive_min :
                    self.L_joy = (self.max_speed / 2) - 6
                    self.R_joy = (self.max_speed / 2) - 2
                    self.get_logger().info(f'left')
                else :
                    self.L_joy = self.before_L_joy
                    self.R_joy = self.before_R_joy

        # self.get_logger().info(f'{self.L_joy}   {self.R_joy}')
        
        self.before_R_joy = self.R_joy
        self.before_L_joy = self.L_joy
        
        msg.data = [self.odrive_mode, self.L_joy, self.R_joy]

        self.control_publisher.publish(msg)
        
    ########################################

    def roll_finder(self) :
        l_point_arr = self.depth_img[int(self.depth_size_y *0.08):int(self.depth_size_y*0.1),int(self.depth_size_x*0.34):int(self.depth_size_x*0.35)]
        r_point_arr = self.depth_img[int(self.depth_size_y *0.08):int(self.depth_size_y*0.1),int(self.depth_size_x*0.65):int(self.depth_size_x*0.66)]

        l_mean = np.mean(l_point_arr)
        r_mean = np.mean(r_point_arr)

        if l_mean*1.15 < r_mean : 
            self.ahead_roll = True
            # self.get_logger().info(f"ahead_roll True {l_mean:.5f}   {r_mean:.5f}")
        else :
            self.ahead_roll = False
            # self.get_logger().info(f"ahead_roll False {l_mean:.5f}   {r_mean:.5f}")

    def imu_msg_sampling(self, msg) :
        imu_data = msg.data
        
        if (imu_data[0] >= 95) |(self.ahead_roll == True) :
            self.robot_roll = 1
            # self.get_logger().info("roll 1")
        else :
            self.robot_roll = 0

        print(self.robot_roll)
        
    
    
    def joy_msg_sampling(self, msg):
        axes = msg.axes
        # btn = msg.buttons

        if axes[2] == 1 :
            self.joy_status = False
        
        else :
            self.joy_status = True
            self.joy_stick_data = [axes[1], axes[4]]
        
        
    ########################################
    ############ control preset ############
    ########################################
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
        
            
            
            
        

def main(args=None):
    rclpy.init(args=args)
    node = SpringColorChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()