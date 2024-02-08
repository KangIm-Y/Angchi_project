import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

coco128_path = '/home/junwoo/robot_ws/src/opencv_test/yolov8_pretrained/coco128.txt'
yolov8_model_path = '/home/junwoo/robot_ws/src/opencv_test/yolov8_pretrained/yolov8n.pt'

class Yolov8(Node):
    def __init__(self):
        super().__init__('yolov8')
        qos_profile = QoSProfile(depth = 10)

        self.yolov8 = self.create_publisher(Image, 'yolov8_frame', qos_profile=qos_profile)

        time_period = 0.1
        self.timer = self.create_timer(timer_period_sec=time_period, callback=self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        coco128 = open(coco128_path, 'r')
        data = coco128.read()
        self.class_list = data.split('\n')
        coco128.close()

        self.model = YOLO(yolov8_model_path)

        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Webcam Frame Error')
        
        yolov8_result = self.yolov8_app(frame)

        cv2.imshow('yolov8 frame', yolov8_result)
        cv2.waitKey(1)

    def yolov8_app(self, frame):
        detection = self.model(frame)[0]

        for data in detection.boxes.data.tolist():
            confidence = float(data[4])
            if confidence < 0.6:
                continue

            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            label = int(data[5])
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(frame, self.class_list[label]+' '+str(round(confidence, 2))+'%', (xmin, ymin), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)

        return frame
    

def main(args=None):
    rclpy.init(args=args)
    yolov8 = Yolov8()

    try:
        rclpy.spin(yolov8)
    except KeyboardInterrupt:
        yolov8.get_logger().info('KeyboardInterrupt')
    finally:
        yolov8.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
