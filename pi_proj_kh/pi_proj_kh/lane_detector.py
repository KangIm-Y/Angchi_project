import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import pyrealsense2 as rs
import math
import numpy as np

import cv2
from cv_bridge import CvBridge


class LaneDetector(Node):

    def __init__(self):
        super().__init__('Lane_Catcher')
        qos_profile = QoSProfile(depth=10)
        self.lane_tracking = self.create_publisher(Float32MultiArray, 'joy_data', qos_profile)
        self.timer = self.create_timer(1/24, self.image_capture)

        self.cap = cv2.VideoCapture(0)
        self.cvbrid = CvBridge()
        
        ################### const parameter init ####################
        self.img_size_x = 1920
        self.img_size_y = 1080

        HFOV = 69   #degree
        VFOV = 42
        DFOV = 77
        focal_length = 1.93 #mm
        pixel_size_color = 0.0014 #mm
        pixel_size_depth = 0.003 #mm

        x_d_value = ( self.img_size_x / 2 ) / math.tan(HFOV/2/180 * np.pi)
        y_d_value = ( self.img_size_y / 2 ) / math.tan(VFOV/2/180 * np.pi)

        color_format = rs.format.bgr8
        depth_format = rs.format.z16

        #################  find good parameters ###################

        self.W_H_ratio = 0.65
        self.warped_remain_ratio = 0.4
        self.height_ratio = 2 # i used 1~4

        ###########################################################
        ################# setting parameter init ##################

        camera_height = 115 #mm

        ###########################################################
        ##################### depth setting  ######################
        self.pipeline = rs.pipeline()
        config = rs.config()
        context = rs.context()
        devices = context.query_devices()

        config.enable_stream(rs.stream.depth,1280,720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color,self.img_size_x,self.img_size_y, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)    

        color_profile = profile.get_stream(rs.stream.color)
        color_intrinsics_object = color_profile.as_video_stream_profile().get_intrinsics()
        ## [ x,  y,  ppx,  ppy,  fx,  fy]
        color_intrinsics = {'ppx' : color_intrinsics_object.ppx, 'ppy' : color_intrinsics_object.ppy, 'fx' : color_intrinsics_object.fx, 'fy' : color_intrinsics_object.fy}
        print(color_intrinsics['ppy'])

        ###########################################################
        ################### pyramid calculate  ####################
        pixel_y = (abs(self.img_size_y*(self.W_H_ratio) - color_intrinsics['ppy'])) /self.img_size_y * 1080
        D_dist = (camera_height * focal_length) / (pixel_y * pixel_size_color)
        d_dist = camera_height / math.tan(VFOV/2 /180 * np.pi)
        self.W_H = 2 * D_dist * math.tan(HFOV/2 / 180 * np.pi)

        slant_height_L = math.sqrt(d_dist **2 + camera_height **2)
        # h_H = (D_dist * focal_length / ((self.img_size_y - color_intrinsics['fy'])/self.img_size_y * 1080 * pixel_size_color))
        h_H = D_dist * math.tan(VFOV/2 /180 * np.pi)
        slant_height_H = math.sqrt(D_dist **2 + h_H **2)
        slant_HFOV = math.atan2((self.W_H/2) , slant_height_H)

        self.W_L = 2 * slant_height_L * math.tan(slant_HFOV)

        # print(f'slant_height_L  : {slant_height_L }   slant_height_H: {slant_height_H}')
        # print(f'D_dist : {D_dist}   d_dist : {d_dist}')
        # print(f'self.W_H : {self.W_H}   self.W_L : {self.W_L}')
        

        ###0.0010000000474974513
        ###fuxking this scale value means, pixel number per 1m
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        # print(profile.get_device().first_depth_sensor().get_self.depth_scale())


        clipping_distance_in_meters = 1 #1 meter
        clipping_distance = clipping_distance_in_meters / self.depth_scale
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        ################# robot state #################
        self.r_state = [0.,0.]
        self.defualt_value = 0.5
        ################################################

        

    def pre_treatment_img(self, origin_img) :
        global thresh_value, thresh_max, thresh_min, thresh_weight
        
        ### color match
        hsv_color = cv2.cvtColor(origin_img, cv2.COLOR_BGR2HSV)
        hsv_lower = np.array([80,40,40])
        hsv_upper = np.array([100,230,230])
        
        hsv_mask = cv2.inRange(hsv_color, hsv_lower, hsv_upper)
        
        hsv_filterd = cv2.bitwise_and(hsv_color, hsv_color, mask=hsv_mask)
        color_filterd = cv2.cvtColor(hsv_filterd, cv2.COLOR_HSV2BGR)
        # cv2.imshow("filterd",color_filterd)

        ### hsv or histogram
        ###
        
        gray = cv2.cvtColor(color_filterd, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (0,0),1)
        
        Y, X = gray.shape
        
        
        ###low light improvement
        Flatten_gray = np.array(gray).flatten()
        mean = np.mean(Flatten_gray)
        # print(mean)
        if mean >= 150 : 
            gray = np.clip(gray - 30., 0, 255).astype(np.uint8)
        elif mean >= 100 :
            gray = gray
        elif mean >= 75 :
            gray = np.clip(gray + 15., 0, 255).astype(np.uint8)
        elif mean >= 45 : 
            gray = cv2.medianBlur(gray,3)
            gray = np.clip(gray + 35., 0, 255).astype(np.uint8)
        else :
            gray = cv2.medianBlur(gray,5)
            gray = np.clip(gray + 60., 0, 255).astype(np.uint8)
            
        ### end   ###
        ### slice ###
        sliced_img = gray[int(Y*0.7):int(Y*0.9),int(X*0.1):int(X*0.9)]
        
        
        _, thresh = cv2. threshold(sliced_img, thresh_value, 255, cv2.THRESH_BINARY)
        # athresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
        # result = np.hstack((gray,thresh, athresh))
        
        
        ### canny
        canny_img = cv2.Canny(sliced_img, 200, 360)
        
        ###hough
        l_lines = cv2.HoughLinesP(canny_img, 1, np.pi/360, 50, None, 50, 5)
        # l_lines = cv2.HoughLines(canny_img, 1, np.pi/180, 25)
        if l_lines is not None :    
            print(len(l_lines))
            for line in l_lines :
                x1, y1, x2, y2 = line[0]
                cv2.line(origin_img, (x1 + int(X*0.1),y1 + int(Y*0.7)), (x2 + int(X*0.1),y2 + int(Y*0.7)), (0,255,0), 3)
                
                # if (thresh_value < thresh_max) and (thresh_weight < 10) :
                #     thresh_value = thresh_value +1
                #     thresh_weight = thresh_weight +1
                # else :
                #     thresh_weight = thresh_weight +1
        # else :
            # if (thresh_value > thresh_min)  and (thresh_weight < 100):
            #     thresh_value = thresh_value - 1
            # else : 
            #     thresh_weight = thresh_weight - 1
        
        # result0 = np.hstack((gray, thresh, canny_img))
        # result1 = origin_img
        
        return thresh, origin_img
            

    def warp(self, img):
        
        
        src = np.float32([[0, 0],   ##  1 2 
                        [int(self.W_H), 0],     ## 4   3
                        [int(self.W_H - (self.W_H-self.W_L)/2), int(self.img_size_y * (1-self.W_H_ratio))*self.height_ratio],
                        [int((self.W_H-self.W_L)/2), int(self.img_size_y * (1-self.W_H_ratio))*self.height_ratio]     ## (x, y)
                        ])
        dst = np.float32([[0, int(self.img_size_y * self.W_H_ratio)],
                        [self.img_size_x, int(self.img_size_y * self.W_H_ratio)],
                        [self.img_size_x, self.img_size_y],
                        [0, self.img_size_y]])
        # print(src)
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        binary_warped = cv2.warpPerspective(img, Minv, (int(self.W_H), int(self.img_size_y * (1-self.W_H_ratio)*self.height_ratio*self.warped_remain_ratio)), flags=cv2.INTER_LINEAR)####이게 필요해 !
        
    
        return binary_warped

    def warp_inv(self, img):
        
        
        src = np.float32([[0, 0],   ##  1 2 
                        [int(self.W_H), 0],     ## 4   3
                        [int(self.W_H - (self.W_H-self.W_L)/2), int(self.img_size_y * (1-self.W_H_ratio))*self.height_ratio],
                        [int((self.W_H-self.W_L)/2), int(self.img_size_y * (1-self.W_H_ratio))*self.height_ratio]     ## (x, y)
                        ])
        dst = np.float32([[0, int(self.img_size_y * self.W_H_ratio)],
                        [self.img_size_x, int(self.img_size_y * self.W_H_ratio)],
                        [self.img_size_x, self.img_size_y],
                        [0, self.img_size_y]])
        # print(src)
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        binary_warped = cv2.warpPerspective(img, M, (int(self.W_H), int(self.img_size_y * (1-self.W_H_ratio)*self.height_ratio)), flags=cv2.INTER_LINEAR)####이게 필요해 !
        
    
        return binary_warped

    ################################# pre processing
    ################################# color detection

    def color_detection(self, image, lower=[16,20,20],upper=[35,255,255]) :
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        hsv_lower = np.array(lower)
        hsv_upper = np.array(upper)
        
        hsv_mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
        
        hsv_filterd = cv2.bitwise_and(hsv_image, hsv_image, mask=hsv_mask)
        
        color_filterd = cv2.cvtColor(hsv_filterd, cv2.COLOR_HSV2BGR)

        return color_filterd



    def abs_sobel_thresh(self, image, orient='x', sobel_kernel=3, thresh=(0, 255)):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0) if orient == 'x' else np.zeros_like(gray)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1) if orient == 'y' else np.zeros_like(gray)
        abs_sobel = np.sqrt(sobelx**2 + sobely**2)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        grad_binary = np.zeros_like(scaled_sobel)
        grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 255
        return grad_binary

    def mag_thresh(self, image, sobel_kernel=3, mag_thresh=(0, 255)):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        abs_sobel = np.sqrt(sobelx**2 + sobely**2)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel)) 
        mag_binary = np.zeros_like(scaled_sobel)
        mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 255

        return mag_binary

    def dir_threshold(self, image, sobel_kernel=3, thresh=(0, np.pi/2)):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        abs_sobelx = np.absolute(sobelx)
        abs_sobely = np.absolute(sobely)
        grad_dir = np.arctan2(abs_sobely, abs_sobelx)
        dir_binary = np.zeros_like(grad_dir)

    def apply_thresholds(self, image, ksize=3):
        gradx = self.abs_sobel_thresh(image, orient='x', sobel_kernel=ksize, thresh=(20, 100))
        grady = self.abs_sobel_thresh(image, orient='y', sobel_kernel=ksize, thresh=(20, 100))
        mag_binary = self.mag_thresh(image, sobel_kernel=ksize, mag_thresh=(30, 100))
        dir_binary = self.dir_threshold(image, sobel_kernel=ksize, thresh=(0.7, 1.3))

        combined = np.zeros_like(gradx)
        combined[((gradx == 255) & (grady == 255)) | ((mag_binary == 255) & (dir_binary == 255))] = 255
        
        return combined


    def apply_color_threshold(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        s_channel = hls[:,:,2]
        s_thresh_min = 170
        s_thresh_max = 255
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 255

        return s_binary
        

    def combine_threshold(self, s_binary, combined):
        combined_binary = np.zeros_like(combined)
        combined_binary[(s_binary == 255) | (combined == 255)] = 255

        return combined_binary


    def get_histogram(self, image):
        # histogram = np.sum(image, axis=0).sum(axis=1)
        histogram = np.sum(image, axis=0)
        # ax.clear()
        # ax.plot(histogram)
        # ax.set_title("Vertical Sum Histogram")
        # plt.draw()
        # plt.pause(0.001)
        return histogram


    def color_filter(self, color_image) :
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        
        lower = (15,20,20)
        upper = (30,255,255)
        
        

    ################################### hough

    def find_lines(self, binary_warped) :
        
        histo = self.get_histogram(binary_warped)
        
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        
        midpoint = int(binary_warped.shape[1]/2)
        leftx_base = np.argmax(histo[:midpoint])
        rightx_base = np.argmax(histo[midpoint:]) + midpoint
        
        
        
        nwindows = 4
        window_height = int(binary_warped.shape[0]/nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0]) #row coordinate
        nonzerox = np.array(nonzero[1]) #col coordinate
        
        
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = int(self.img_size_x * 0.09)
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        
        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height   #from bottom to top
            win_y_high = binary_warped.shape[0] - window*window_height      ##low is top. high is bottom
            
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high), (0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high), (0,255,0), 2) 
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            ### good data is true/false. but, use nonzero, counting "Ture"
                    
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 
        
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
        except TypeError as e:
            print("TypeError:", e)
            
            info = {}
            info['leftx'] = 0
            info['rightx'] = 0
            info['left_fitx'] = 0
            info['right_fitx'] = 0
            info['ploty'] = 0

            return info
        
        ## 채우기 같은거
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )       # height
        
        ##### yellow line #####
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        out_img[np.uint32(ploty), np.uint32(left_fitx)] = [0,255,0]
        out_img[np.uint32(ploty), np.uint32(right_fitx)] = [0,255,0]
        
        #######################

        
        ##### red and blue line #####
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
        #############################
        
        
        
        info = {}
        info['leftx'] = leftx
        info['rightx'] = rightx
        info['left_fitx'] = left_fitx
        info['right_fitx'] = right_fitx
        info['ploty'] = ploty
        info['total'] = binary_warped.shape[1]
        
        # cv2.imshow("out_img",out_img)
        
        return info


    def measure_curvature(self, lines_info):
        ym_per_pix = 1
        xm_per_pix = 1 

        leftx = lines_info['left_fitx']
        rightx = lines_info['right_fitx']
        ploty = lines_info['ploty']

        leftx = leftx[::-1]  
        rightx = rightx[::-1]  

        y_eval = np.max(ploty)
        left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        # print(left_curverad, 'm', right_curverad, 'm')
        
        return left_curverad, right_curverad
    
    def image_capture(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())
        
        
        split_img = self.color_image.copy()
        split_img[:int(self.img_size_y*self.W_H_ratio),:] = 0
        
        warped_color = self.warp(split_img)
        
        
        ###find l_lines from warped
        combined = self.apply_thresholds(self.color_image) #threshold
        s_binary = self.apply_color_threshold(split_img)
        combined_binary = self.combine_threshold(s_binary, combined)
        warped_split_img = self.warp(combined_binary)
        
        cv2.imshow("combined_binary", warped_split_img)
        
        # line_window = warped_split_img[:int(warped_split_img.shape[0]*warped_remain_ratio),:]
        try :
            infos = self.find_lines(warped_split_img)   
            left_curverad, right_curverad = self.measure_curvature(infos)
            # print(left_curverad, right_curverad)
            
            l_fitx = infos["left_fitx"]
            r_fitx = infos["right_fitx"]
            
            origin_l_x = int(l_fitx[-1])
            origin_r_x = int(r_fitx[-1])
            
            # print(origin_l_x)
            # print(origin_r_x)
            cv2.circle(self.color_image, (int(origin_l_x/(l_fitx.shape[0]) * self.img_size_x), int(self.img_size_y * self.W_H_ratio + ((self.img_size_y - self.img_size_y * self.W_H_ratio) * self.warped_remain_ratio))), 5, (255,0,0), -1, cv2.LINE_AA)
            cv2.circle(self.color_image, (int(origin_r_x/(r_fitx.shape[0]) * self.img_size_x), int(self.img_size_y * self.W_H_ratio + ((self.img_size_y - self.img_size_y * self.W_H_ratio) * self.warped_remain_ratio))), 5, (255,0,0), -1, cv2.LINE_AA)
            cv2.rectangle(self.color_image, (0, int(self.img_size_y * self.W_H_ratio)), (int(self.img_size_x), int(self.img_size_y * self.W_H_ratio)), (255,0,0), 2)
            cv2.rectangle(self.color_image, (0, int(self.img_size_y * self.W_H_ratio)), (int(self.img_size_x), int(self.img_size_y * self.W_H_ratio + ((self.img_size_y - self.img_size_y * self.W_H_ratio) * self.warped_remain_ratio))), (255,0,0), 2)


            
            mid_point = int(infos['total']/2)
            L_mid = mid_point - origin_l_x
            R_mid = origin_r_x  - mid_point
            
            self.get_logger().info(f'L : {origin_l_x} R : {origin_r_x} L_mid : {L_mid} R_mid :{R_mid}    total : {mid_point}')
            
            self.r_state[0] = self.defualt_value * R_mid/L_mid
            self.r_state[1] = self.defualt_value * L_mid/R_mid
            
            
        except :
            self.r_state = self.r_state

        cv2.imshow('Origin', self.color_image)
        cv2.imshow('warp', warped_color)
        self.joy_data_publish()


    def joy_data_publish(self):
        msg = Float32MultiArray()
        msg.data = self.r_state
        self.lane_tracking.publish(msg)
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()