import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamSubscriber(Node):
    def __init__(self):
        super().__init__('frame_sub')
        
        qos_profile = QoSProfile(depth=10)
        self.frame_subscriber = self.create_subscription(Image, 'webcam_frame',callback=self.webcam_callback , qos_profile=qos_profile)
        self.motor_control_publisher = self.create_publisher(String, 'Motor_control', qos_profile=qos_profile)
        self.aruco_publisher = self.create_publisher(Image, 'aruco_frame', qos_profile=qos_profile)
        self.br = CvBridge()

    def webcam_callback(self, msg):
        self.get_logger().info('Frame Subscriber Start')
        frame = self.br.imgmsg_to_cv2(msg)
        aruco_frame, aruco_ids = self.detect_aruco(frame)
        # self.aruco_publisher.publish(self.br.cv2_to_imgmsg(aruco_frame))

        motor_control = String()
        if aruco_ids == 1:
            motor_control.data = 'Left'
        elif aruco_ids == 2:
            motor_control.data = 'Right'
        else:
            motor_control.data = 'retry'

        self.motor_control_publisher.publish(motor_control)
        # cv2.imshow('frame sub', frame)
        cv2.imshow('aruco frame', aruco_frame)
        cv2.waitKey(1)

    def detect_aruco(self, frame):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        aruco_param = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_param)
        detect_markers = self.aruco_display(corners, ids, rejected, frame)
        return detect_markers, ids
    
    def aruco_display(self, corners, ids, rejected, frame):
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(frame, topLeft, topRight, (0, 255, 0), 1)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 1)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 1)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 1)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return frame


def main(args=None):
    rclpy.init(args=args)
    webcam_subscriber = WebcamSubscriber()
    try:
        rclpy.spin(webcam_subscriber)
    except KeyboardInterrupt:
        webcam_subscriber.get_logger().info('KeyboardInterrupt (SIGINT)')
    finally:
        webcam_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
