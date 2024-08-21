#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
from itertools import combinations


class ArrowDetectorNode(Node):
    def __init__(self):
        super().__init__('arrow_detector')

        self.arrow_pub = self.create_publisher(String, 'arrow', 10)
        
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def is_triangle(self, points):
        # 화살표의 7개 포인트 확인후 -> 삼각형까지 이중 확인하기
        if len(points) != 7:
            return False
        
        
        for comb in combinations(points, 3):
            triangle = np.array(comb, dtype=np.int32)
            hull = cv2.convexHull(triangle)
            if len(hull) == 3:
                return True
        return False

    def detect_arrow(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 150])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        frame[mask != 255] = [0, 0, 0]

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            epsilon = 0.03 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            #화살표 방향&각도 검출 

            if len(approx) == 7:
                arrow_points = approx.reshape(7, 2)
                if self.is_triangle(arrow_points):
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    center = np.mean(arrow_points, axis=0).astype(int)
                    v1 = arrow_points[1] - arrow_points[0]
                    v2 = arrow_points[6] - arrow_points[0]
                    direction_vector = (v1 + v2) / 2
                    direction_vector = -direction_vector
                    angle_camera_frame_deg = np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi

                    if direction_vector[0] < 0 and direction_vector[1] < 0:
                        ANGLE = -angle_camera_frame_deg - 90
                        if ANGLE < 20:
                            text = "GO"
                        elif ANGLE >= 20 and ANGLE < 89:
                            text = "LEFT"
                        elif ANGLE == 180:
                            text = "BACK"
                        else:
                            text = "TURN"
                    else:
                        ANGLE = angle_camera_frame_deg + 90
                        if ANGLE < 20:
                            text = "GO"
                        elif ANGLE >= 20 and ANGLE < 100:
                            text = "RIGHT"
                        elif ANGLE == 180:
                            text = "BACK"
                        else:
                            text = "TURN"

                    cv2.putText(frame, text, (50, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.putText(frame, f"Angle: {ANGLE:.2f} degrees", (50, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.arrowedLine(frame, center, (center[0] + int(direction_vector[0]), center[1] + int(direction_vector[1])), (0, 0, 255), 2)
                    
                    return text
                else:
                    return "Not an Arrow"
        return "No Arrow"

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        text = self.detect_arrow(frame)

        msg = String()
        msg.data = text
        self.arrow_pub.publish(msg)

        cv2.imshow('Arrow Detection', frame)
        cv2.waitKey(1)  

def main(args=None):
    rclpy.init(args=args)
    node = ArrowDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()  
        cv2.destroyAllWindows()  
        rclpy.shutdown()

if __name__ == '__main__':
    main()
