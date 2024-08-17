import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO # 추가한 부분

class CameraStateSubscriber(Node):
    def __init__(self):
        super().__init__('track_imu_node')
        self.subscription = self.create_subscription(
            Int32,
            'camera_state',
            self.camera_state_callback,
            10
        )
        
        self.timer = self.create_timer(1/10, self.camera_state_callback)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15) # 해상도 / 프레임
        self.pipeline.start(config)

        # 해상도 변경시 -> cen_x = 변경된 해상도의 1/2 값. / 나머지 = 변경된 값 그대로.
        self.cen_x = 212
        self.roi_start_row = int(240 * 0.35)
        self.roi_end_row = int(240 * 0.5)
        self.roi_start_col = int(424 * 0)
        self.roi_end_col = int(424 * 1)
        self.yellow_color = (0, 255, 255)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) ## 추가된 부분
        
        self.camera_state_publisher_ = self.create_publisher(
            Int32,
            'camera_state_output',
            10
        )
        self.image_publisher = self.create_publisher(
            Image,
            'camera_image',
            10
        )

        self.bridge = CvBridge()
        # 추가한 부분
        self.model = YOLO('/home/skh/robot_ws/src/track_test_pkg/track_test_pkg/chess_b.pt')  # 모델 파일 위치 .pt
        # 추가한 부분
        
    def camera_state_callback(self):
        # self.state = msg.data

        fs = self.pipeline.wait_for_frames()
        color_f = fs.get_color_frame()
        color_a = np.asanyarray(color_f.get_data())
        color_c = cv2.GaussianBlur(color_a, (5, 5), 0) # 블러 처리. 홀수만 가능.
        # color_o = cv2.morphologyEx(color_i, cv2.MORPH_OPEN, self.kernel) ### 추가된 부분
        # color_c = cv2.morphologyEx(color_o, cv2.MORPH_CLOSE, self.kernel) ### 추가된 부분
        
        hsv_i = cv2.cvtColor(color_c, cv2.COLOR_BGR2HSV)
        roi = hsv_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col]

        mask = np.zeros_like(hsv_i)
        mask[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col, :] = 255
        hsv_i = cv2.bitwise_and(hsv_i, mask)

        #dominant_color = np.mean(roi, axis=(0, 1)) # ROI 내 평균 색상
        lower_bound = np.array((17, 10, 10)) # 너무 많이 인식하면 20으로, 인식을 잘 못 하면 15로 (혹은 16)
        upper_bound = np.array((130, 255, 255))
        
        col_mask = cv2.inRange(roi, lower_bound, upper_bound)
        mask = col_mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        masked_pixel_count = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 10: ## 추가된 부분
                component_mask = np.zeros_like(mask)
                cv2.drawContours(component_mask, [contour], -1, (255, 255, 0), thickness=cv2.FILLED)
                roi[component_mask > 0] = self.yellow_color
                masked_pixel_count += area
        
        hsv_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col] = roi
        
        cv2.rectangle(hsv_i, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)
        cv2.line(hsv_i, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)

        left_mask = mask[:, :self.cen_x - self.roi_start_col].copy()
        right_mask = mask[:, self.cen_x - self.roi_start_col:].copy()

        l_area = np.sum(left_mask == 255)
        r_area = np.sum(right_mask == 255)

        cv2.putText(hsv_i, f"L Area: {l_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(hsv_i, f"R Area: {r_area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        state_msg = 0

        L_AREA = 6876
        R_AREA = 6840
        # 추가한 부분
        detection = self.model(color_c)[0]
        self.chess_detected = False
        # 추가추가한 부분
        min_chess_y = float('inf')
        chess_x_o = 0
        self.object_pisitions = []
        # 추가추가한 부분

        for data in detection.boxes.data.tolist():
            xmin, ymin, xmax, ymax = map(int, data[:4])
            label = int(data[5])
            confidence = float(data[4])
            
            if confidence > 0.6 and self.class_list[label] in ["chess"]:
                cv2.rectangle(color_c, (xmin, ymin), (xmax, ymax), (0, 255, 0), 1)
                self.chess_detected = True
                
                chess_x = (xmin + xmax) // 2
                chess_y = (ymin + ymax) // 2
                # 추가추가한 부분
                self.object_pisitions.append((chess_x, chess_y))

                if chess_y < min_chess_y:
                    min_chess_y = chess_y
                    chess_x_o = chess_x

                cv2.circle(color_c, (chess_x, chess_y), 5, (0, 0, 255), -1)

        straight_range = (self.cen_x - 20, self.cen_x + 20)

        if straight_range[0] <= chess_x_o <= straight_range[1]:
            state_msg = 0 # Straight
        elif chess_x_o < straight_range[0]:
            state_msg = -1 # Left
        else:
            state_msg = 1 # Right

        print("Object Pos: ", self.object_pisitions)
        print(chess_x_o)
        # 추가추가한 부분
        # if not self.chess_detected:
        #     # 추가한 부분
        #     if self.state == -1: # 왼쪽으로 기울어있을때
        #         roi[:, self.cen_x - self.roi_start_col:] = [0, 0, 0]
        #         if l_area < L_AREA * 0.683: # 오른쪽으로 더 가게 하고 싶으면 증가 (아래에도 있음)
        #             state_msg = 1  # Right
        #         elif L_AREA * 0.683 <= l_area < L_AREA * 0.887: # 왼쪽으로 더 가게 하고 싶으면 감소
        #             state_msg = 0  # Straight
        #         else:
        #             state_msg = -1  # Left

        #     elif self.state == 1: # 오른쪽으로 기울었을때
        #         roi[:, :self.cen_x - self.roi_start_col] = [0, 0, 0]
        #         if r_area < R_AREA * 0.97: # 왼쪽으로 더 가게 하고 싶으면 증가 (아래에도 있음) 0.96
        #             state_msg = -1  # Left
        #         elif R_AREA * 0.97 <= r_area < R_AREA * 0.98: # 오른쪽으로 더 가게 하고 싶으면 감소 0.96, 0.972
        #             state_msg = 0  # Straight
        #         else:
        #             state_msg = 1  # Right

        #     elif self.state == 0: # 평평할 때
        #         if l_area > 3000 and r_area > 3000:
        #             if l_area > r_area * 1.6: # 왼쪽으로 더 가게하고 싶으면 감소. 1로 하면 안 됨
        #                 state_msg = -3  # Left.
        #             elif r_area > l_area * 1.6: # 오른쪽으로 더 가게하고 싶으면 감소. 1로 하면 안 됨
        #                 state_msg = 3  # Right
        #             else:
        #                 state_msg = 0  # Straight
        #         else:
        #             state_msg = 2  # Stop

        self.camera_state_publisher_.publish(Int32(data=state_msg))
        ros_image = self.bridge.cv2_to_imgmsg(hsv_i, encoding="bgr8")
        self.image_publisher.publish(ros_image)

        cv2.imshow('Track', hsv_i)
        cv2.imshow('test', color_c)
        print(state_msg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pass

def main(args=None):
    rclpy.init(args=args)
    camera_state_subscriber = CameraStateSubscriber()
    rclpy.spin(camera_state_subscriber)
    camera_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()