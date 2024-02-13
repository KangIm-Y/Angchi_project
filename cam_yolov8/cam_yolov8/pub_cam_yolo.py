import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String
import datetime
from cv_bridge import CvBridge

CONFIDENCE_THRESHOLD = 0.5
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)


class CamYoloPublisher(Node):

    def __init__(self):
        super().__init__('pub_cam_yolo')
        qos_profile = QoSProfile(depth=10)
        self.cam_yolo_pub = self.create_publisher(String, 'Gesture', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_msg)
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        coco128 = open('/home/kangim/robot_ws/src/cam_yolov8/cam_yolov8/cocoABC.txt', 'r')
        data = coco128.read()
        self.class_list = data.split('\n')
        coco128.close()
        self.model = YOLO('/home/kangim/robot_ws/src/cam_yolov8/cam_yolov8/best.pt')

        self.br = CvBridge()

    def publish_msg(self):
        
        start = datetime.datetime.now()

        ret, frame = self.cap.read()
        if not ret:
            print('Cam Error')
            return

        detection = self.model(frame)[0]
        pub_data = ""

        for data in detection.boxes.data.tolist(): # data : [xmin, ymin, xmax, ymax, confidence_score, class_id]
            confidence = float(data[4])
            if confidence < CONFIDENCE_THRESHOLD:
                continue

            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            label = int(data[5])
            if label == 21 or label == 10: # V K
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.putText(frame, 'Stop'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                pub_data = "Stop"
            
            elif label == 11: # L
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.putText(frame, 'Left'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                pub_data = "Left"

            elif label == 24: # Y
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.putText(frame, 'Right'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                pub_data = "Right"

            elif label == 22: # W
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.putText(frame, 'Go'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                pub_data = "Go"

            elif label == 1 or label == 20: # B U
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.putText(frame, 'Feedback'+' '+str(int(round(confidence, 2)*100)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
                pub_data = "Feedback"

        end = datetime.datetime.now()
            
        total = (end - start).total_seconds()
        #print(f'Time to process 1 frame: {total * 1000:.0f} milliseconds')

        fps = f'FPS: {1 / total:.2f}'
        cv2.putText(frame, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow('frame', frame)
        cv2.waitKey(1)
        send_data = String()
        send_data.data = pub_data
        if pub_data != "":
            self.cam_yolo_pub.publish(send_data)


def main(args=None):
    rclpy.init(args=args)
    node = CamYoloPublisher()
    try:
        while rclpy.ok():
            node.publish_msg()

    except KeyboardInterrupt:
        node.get_logger().info('Node Exit :: Keyboard Interrupt')
    finally:
        #CamYoloPublisher.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()